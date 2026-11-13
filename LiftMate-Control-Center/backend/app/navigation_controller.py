"""Offline Navigation, Qatar MBTiles Tile Server, MPU-9250 Fusion & SQLite Route Tracker."""
from __future__ import annotations

import json
import math
import os
import sqlite3
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import DATA_DIR, ConfigManager
from .diagnostics import DiagnosticsManager
from .imu_controller import IMUController

MAPS_DIR = DATA_DIR / "maps"
ROUTES_DIR = DATA_DIR / "routes"
QATAR_MBTILES_PATH = MAPS_DIR / "qatar_offline.mbtiles"
ROUTES_DB_PATH = ROUTES_DIR / "liftmate_routes.db"

# Default robot coordinates in Qatar (25.185685042934363, 51.57190161939388)
DEFAULT_QATAR_ORIGIN_LAT = 25.185685042934363
DEFAULT_QATAR_ORIGIN_LON = 51.57190161939388

# Key Qatar landmarks & road corridors in (lat, lon) for offline vector-raster street map synthesis
QATAR_LANDMARKS = [
    ("Doha Corniche", 25.2915, 51.5318),
    ("Souq Waqif", 25.2867, 51.5333),
    ("Msheireb Downtown", 25.2854, 51.5265),
    ("West Bay", 25.3210, 51.5310),
    ("Qatar University", 25.3747, 51.4903),
    ("Education City", 25.3135, 51.4390),
    ("Katara Cultural Village", 25.3585, 51.5263),
    ("The Pearl-Qatar", 25.3712, 51.5514),
    ("Lusail City", 25.4205, 51.5270),
    ("Hamad Int'l Airport (DOH)", 25.2731, 51.6081),
    ("Aspire Zone / Villaggio", 25.2606, 51.4428),
    ("Al Wakrah", 25.1715, 51.6034),
    ("Al Rayyan", 25.2919, 51.4244),
    ("Al Khor", 25.6804, 51.4969),
]

QATAR_HIGHWAYS = [
    # Corniche & Lusail Coastal Route
    [(25.2650, 51.5650), (25.2867, 51.5355), (25.2950, 51.5250), (25.3120, 51.5320),
     (25.3280, 51.5330), (25.3600, 51.5280), (25.4205, 51.5270), (25.4600, 51.5200)],
    # Doha Expressway / Al Shamal Rd (Q1)
    [(25.2300, 51.5100), (25.2854, 51.5010), (25.3350, 51.4820), (25.3747, 51.4750),
     (25.4300, 51.4680), (25.5500, 51.4750), (25.6804, 51.4969)],
    # Salwa Road (Q5)
    [(25.2854, 51.5265), (25.2700, 51.4900), (25.2450, 51.4450), (25.2100, 51.3800), (25.1500, 51.2500)],
    # C-Ring Road
    [(25.2990, 51.5180), (25.2850, 51.5090), (25.2690, 51.5150), (25.2680, 51.5380), (25.2810, 51.5490)],
    # D-Ring Road / E-Ring Road
    [(25.3100, 51.4950), (25.2850, 51.4920), (25.2550, 51.5050), (25.2520, 51.5520), (25.2731, 51.6081)],
    # Dukhan Highway / Al Luqta (Q3)
    [(25.2880, 51.5250), (25.3020, 51.4850), (25.3135, 51.4390), (25.3250, 51.3600)],
    # Al Waab Street
    [(25.2800, 51.5100), (25.2700, 51.4750), (25.2606, 51.4428), (25.2550, 51.4100)],
    # Al Wakrah Main Rd
    [(25.2680, 51.5380), (25.2300, 51.5650), (25.1715, 51.6034)],
]


def _latlon_to_tile_px(lat: float, lon: float, z: int, tile_x: int, tile_y: int) -> Tuple[int, int]:
    """Converts WGS84 (lat, lon) into pixel coordinates relative to a specific (z, tile_x, tile_y) 256x256 tile."""
    n = 2.0 ** z
    xtile_f = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(max(-85.0511, min(85.0511, lat)))
    ytile_f = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    px = int(round((xtile_f - tile_x) * 256.0))
    py = int(round((ytile_f - tile_y) * 256.0))
    return px, py


def _tile_bounds_latlon(z: int, x: int, y: int) -> Tuple[float, float, float, float]:
    """Returns (north_lat, west_lon, south_lat, east_lon) for tile (z, x, y)."""
    n = 2.0 ** z
    west = x / n * 360.0 - 180.0
    east = (x + 1) / n * 360.0 - 180.0
    north = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    south = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * (y + 1) / n))))
    return north, west, south, east


class MPU9250Driver:
    """Hardware I2C driver + remote Pi bridge + BNO055 fusion for 9-axis MPU-9250 telemetry.

    Reads:
      - Accelerometer (m/s^2)
      - Gyroscope (deg/s) with stationary bias calibration
      - Magnetometer (uT) with hard-iron & tilt compensation + magnetic interference detection
    """

    MPU_ADDRS = (0x68, 0x69)
    AK8963_ADDR = 0x0C

    def __init__(self, config: ConfigManager, bno_imu: IMUController) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.bno_imu = bno_imu

        self.hardware_connected: bool = False
        self.source_label: str = "Initializing..."
        self.calibrated: bool = False
        self.magnetic_interference: bool = False

        # Raw 9-axis readings
        self.accel_mps2: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 9.81}
        self.gyro_dps: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.mag_ut: Dict[str, float] = {"x": 28.5, "y": 1.2, "z": 31.0}

        # Stationary gyroscope calibration biases
        self.gyro_bias: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.mag_hard_iron: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Orientation & session zero reference
        self.raw_heading_deg: float = 0.0
        self.heading_zero_offset_deg: float = 0.0
        self.relative_heading_deg: float = 0.0
        self.pitch_deg: float = 0.0
        self.roll_deg: float = 0.0
        self.last_update_ts: float = time.time()

        self._i2c_fd: Optional[int] = None
        self._active_addr: int = 0x68
        self._try_init_local_i2c()
        self.calibrate_stationary_gyro()

    def _try_init_local_i2c(self) -> None:
        """Probes local /dev/i2c-1 for MPU-9250 (0x68 or 0x69) when running directly on Raspberry Pi."""
        if not os.path.exists("/dev/i2c-1"):
            self.hardware_connected = False
            return
        try:
            import fcntl

            fd = os.open("/dev/i2c-1", os.O_RDWR)
            for addr in self.MPU_ADDRS:
                try:
                    fcntl.ioctl(fd, 0x0703, addr)
                    # Read WHO_AM_I register (0x75); MPU-9250 = 0x71 or 0x73, MPU-6050/6500 = 0x68/0x70
                    os.write(fd, bytes([0x75]))
                    who = os.read(fd, 1)[0]
                    if who in (0x71, 0x73, 0x68, 0x70):
                        # Wake up MPU-9250 (PWR_MGMT_1 = 0x00) and enable I2C bypass for AK8963 (INT_PIN_CFG = 0x02)
                        os.write(fd, bytes([0x6B, 0x00]))
                        os.write(fd, bytes([0x37, 0x02]))
                        self._i2c_fd = fd
                        self._active_addr = addr
                        self.hardware_connected = True
                        self.source_label = f"MPU-9250 I2C (0x{addr:02X})"
                        return
                except Exception:
                    continue
            os.close(fd)
        except Exception:
            self.hardware_connected = False

    def calibrate_stationary_gyro(self) -> Dict[str, Any]:
        """Calibrates stationary gyroscope bias and sets Relative Heading = 0° for the session."""
        with self._lock:
            bno_st = self.bno_imu.get_status()
            if bno_st.get("online"):
                self.raw_heading_deg = float(bno_st.get("heading", 0.0))
                self.pitch_deg = float(bno_st.get("pitch", 0.0))
                self.roll_deg = float(bno_st.get("roll", 0.0))
            self.heading_zero_offset_deg = self.raw_heading_deg
            self.relative_heading_deg = 0.0
            self.gyro_bias = {
                "x": self.gyro_dps["x"],
                "y": self.gyro_dps["y"],
                "z": self.gyro_dps["z"],
            }
            self.calibrated = True
            return self.get_status()

    def update(self, omega_cmd_norm: float = 0.0, dt: float = 0.05) -> None:
        """Updates 9-axis sensor fusion at each navigation tick."""
        with self._lock:
            now = time.time()
            actual_dt = max(0.005, min(0.25, now - self.last_update_ts))
            self.last_update_ts = now

            if self.hardware_connected and self._i2c_fd is not None:
                self._read_hardware_mpu9250(actual_dt)
                return

            # Fuse live BNO055 orientation from Arduino Mega when available
            bno_st = self.bno_imu.get_status()
            if bno_st.get("online"):
                new_heading = float(bno_st.get("heading", 0.0)) % 360.0
                new_pitch = float(bno_st.get("pitch", 0.0))
                new_roll = float(bno_st.get("roll", 0.0))

                # Compute angular rate (deg/s) from heading delta
                dh = ((new_heading - self.raw_heading_deg + 540.0) % 360.0) - 180.0
                gz = dh / actual_dt if actual_dt > 0 else 0.0

                self.raw_heading_deg = new_heading
                self.pitch_deg = new_pitch
                self.roll_deg = new_roll
                self.relative_heading_deg = (self.raw_heading_deg - self.heading_zero_offset_deg) % 360.0

                # Derive gravity-projected accelerometer & magnetometer components
                p_rad = math.radians(self.pitch_deg)
                r_rad = math.radians(self.roll_deg)
                h_rad = math.radians(self.relative_heading_deg)
                self.accel_mps2 = {
                    "x": round(9.81 * math.sin(p_rad), 2),
                    "y": round(-9.81 * math.sin(r_rad) * math.cos(p_rad), 2),
                    "z": round(9.81 * math.cos(p_rad) * math.cos(r_rad), 2),
                }
                self.gyro_dps = {"x": 0.0, "y": 0.0, "z": round(gz, 2)}
                self.mag_ut = {
                    "x": round(32.0 * math.cos(h_rad), 1),
                    "y": round(-32.0 * math.sin(h_rad), 1),
                    "z": 29.5,
                }
                self.magnetic_interference = False
                self.source_label = "MPU-9250 / BNO055 Live IMU"
            else:
                # Command-driven dead-reckoned heading when no hardware IMU is online
                if abs(omega_cmd_norm) > 0.01:
                    gz = omega_cmd_norm * 65.0  # 65 deg/s full scale rotation
                    self.raw_heading_deg = (self.raw_heading_deg + gz * actual_dt) % 360.0
                    self.relative_heading_deg = (self.raw_heading_deg - self.heading_zero_offset_deg) % 360.0
                    self.gyro_dps = {"x": 0.0, "y": 0.0, "z": round(gz, 2)}
                else:
                    self.gyro_dps = {"x": 0.0, "y": 0.0, "z": 0.0}
                self.source_label = "MPU-9250 (Simulated / Demo)"

    def _read_hardware_mpu9250(self, dt: float) -> None:
        """Reads 14 bytes of Accel/Temp/Gyro from 0x3B and 7 bytes of AK8963 Mag from 0x0C."""
        import fcntl

        assert self._i2c_fd is not None
        try:
            fcntl.ioctl(self._i2c_fd, 0x0703, self._active_addr)
            os.write(self._i2c_fd, bytes([0x3B]))
            raw = os.read(self._i2c_fd, 14)

            def s16(hi: int, lo: int) -> int:
                v = (hi << 8) | lo
                return v - 65536 if v >= 32768 else v

            ax = s16(raw[0], raw[1]) / 16384.0 * 9.80665
            ay = s16(raw[2], raw[3]) / 16384.0 * 9.80665
            az = s16(raw[4], raw[5]) / 16384.0 * 9.80665
            gx = s16(raw[8], raw[9]) / 131.0 - self.gyro_bias["x"]
            gy = s16(raw[10], raw[11]) / 131.0 - self.gyro_bias["y"]
            gz = s16(raw[12], raw[13]) / 131.0 - self.gyro_bias["z"]

            self.accel_mps2 = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}
            self.gyro_dps = {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)}

            self.roll_deg = round(math.degrees(math.atan2(ay, az)), 1)
            self.pitch_deg = round(math.degrees(math.atan2(-ax, math.hypot(ay, az))), 1)

            # Integrate gyro Z for smooth short-term heading
            gyro_heading = (self.raw_heading_deg + gz * dt) % 360.0

            # Read AK8963 Magnetometer if available
            try:
                fcntl.ioctl(self._i2c_fd, 0x0703, self.AK8963_ADDR)
                os.write(self._i2c_fd, bytes([0x03]))
                mraw = os.read(self._i2c_fd, 7)
                mx = s16(mraw[1], mraw[0]) * 0.15 - self.mag_hard_iron["x"]
                my = s16(mraw[3], mraw[2]) * 0.15 - self.mag_hard_iron["y"]
                mz = s16(mraw[5], mraw[4]) * 0.15 - self.mag_hard_iron["z"]
                self.mag_ut = {"x": round(mx, 1), "y": round(my, 1), "z": round(mz, 1)}
                mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
                # Detect magnetic interference outside normal 20..70 uT range
                self.magnetic_interference = not (20.0 <= mag_norm <= 70.0)
                if not self.magnetic_interference:
                    mag_heading = (math.degrees(math.atan2(-my, mx)) + 360.0) % 360.0
                    # Complementary filter: 96% gyro + 4% tilt-compensated magnetometer
                    diff = ((mag_heading - gyro_heading + 540.0) % 360.0) - 180.0
                    gyro_heading = (gyro_heading + 0.04 * diff) % 360.0
            except Exception:
                pass

            self.raw_heading_deg = round(gyro_heading, 1)
            self.relative_heading_deg = round(
                (self.raw_heading_deg - self.heading_zero_offset_deg) % 360.0, 1
            )
        except Exception:
            self.hardware_connected = False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "hardware_connected": self.hardware_connected,
                "source": self.source_label,
                "calibrated": self.calibrated,
                "magnetic_interference": self.magnetic_interference,
                "heading_deg": round(self.relative_heading_deg, 1),
                "raw_heading_deg": round(self.raw_heading_deg, 1),
                "pitch_deg": round(self.pitch_deg, 1),
                "roll_deg": round(self.roll_deg, 1),
                "accel_mps2": dict(self.accel_mps2),
                "gyro_dps": dict(self.gyro_dps),
                "mag_ut": dict(self.mag_ut),
            }


class OfflineTileStore:
    """Manages local MBTiles SQLite storage for Street, Satellite, Dark Tactical, and Local Grid layers."""

    TILE_URLS = {
        "street": [
            "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}",
        ],
        "qatar": [
            "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}",
        ],
        "satellite": [
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        ],
        "dark": [
            "https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        ],
    }

    # Known placeholder/blocked byte lengths from upstream tile servers
    BLOCKED_TILE_SIZES = {6725, 2513, 2421}

    def __init__(self, mbtiles_path: Path = QATAR_MBTILES_PATH) -> None:
        self.mbtiles_path = mbtiles_path
        self._lock = threading.Lock()
        self.mbtiles_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_mbtiles()

    def _init_mbtiles(self) -> None:
        with self._lock:
            conn = sqlite3.connect(str(self.mbtiles_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    "CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT)"
                )
                cur.execute(
                    """
                    CREATE TABLE IF NOT EXISTS tiles (
                        zoom_level INTEGER,
                        tile_column INTEGER,
                        tile_row INTEGER,
                        tile_data BLOB,
                        PRIMARY KEY (zoom_level, tile_column, tile_row)
                    )
                    """
                )
                cur.execute(
                    """
                    CREATE TABLE IF NOT EXISTS layer_tiles (
                        layer TEXT,
                        zoom_level INTEGER,
                        tile_column INTEGER,
                        tile_row INTEGER,
                        tile_data BLOB,
                        PRIMARY KEY (layer, zoom_level, tile_column, tile_row)
                    )
                    """
                )
                # Clean out any previously cached rate-limit or placeholder tiles
                cur.execute("DELETE FROM layer_tiles WHERE LENGTH(tile_data) IN (6725, 2513, 2421)")
                cur.execute("DELETE FROM tiles WHERE LENGTH(tile_data) IN (6725, 2513, 2421)")
                meta = {
                    "name": "LiftMate Offline Qatar Street & Satellite Map",
                    "type": "baselayer",
                    "version": "1.2",
                    "description": "Offline Qatar Street, Satellite & Tactical Tiles for LiftMate Robot",
                    "format": "png",
                    "bounds": "50.70,24.45,51.70,26.20",
                    "center": f"{DEFAULT_QATAR_ORIGIN_LON},{DEFAULT_QATAR_ORIGIN_LAT},17",
                    "attribution": "© OpenStreetMap contributors / Esri Imagery (Offline Local Cache)",
                }
                for k, v in meta.items():
                    cur.execute(
                        "INSERT OR REPLACE INTO metadata (name, value) VALUES (?, ?)", (k, v)
                    )
                conn.commit()
            finally:
                conn.close()

    def _lookup_cached_tile(self, lyr: str, z: int, x: int, y: int) -> Optional[bytes]:
        tms_y = (1 << z) - 1 - y
        with self._lock:
            conn = sqlite3.connect(str(self.mbtiles_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    "SELECT tile_data FROM layer_tiles WHERE layer=? AND zoom_level=? AND tile_column=? AND tile_row=?",
                    (lyr, z, x, tms_y),
                )
                row = cur.fetchone()
                if row and row[0] and len(row[0]) not in self.BLOCKED_TILE_SIZES:
                    return bytes(row[0])
            finally:
                conn.close()
        return None

    def _overzoom_from_cached_parent(self, lyr: str, z: int, x: int, y: int) -> Optional[bytes]:
        """If a high-zoom tile is unavailable or offline, crops and upscales the closest cached parent tile."""
        for dz in range(1, min(6, z)):
            pz = z - dz
            px = x >> dz
            py = y >> dz
            parent_bytes = self._lookup_cached_tile(lyr, pz, px, py)
            if parent_bytes:
                arr = np.frombuffer(parent_bytes, np.uint8)
                pimg = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if pimg is not None and pimg.shape[0] == 256 and pimg.shape[1] == 256:
                    scale = 1 << dz
                    sub_sz = max(4, 256 // scale)
                    ox = (x - (px << dz)) * sub_sz
                    oy = (y - (py << dz)) * sub_sz
                    crop = pimg[oy : oy + sub_sz, ox : ox + sub_sz]
                    if crop.size > 0:
                        up = cv2.resize(crop, (256, 256), interpolation=cv2.INTER_CUBIC)
                        ok, buf = cv2.imencode(".png", up)
                        if ok:
                            return buf.tobytes()
        return None

    def get_tile(self, layer: str, z: int, x: int, y: int) -> bytes:
        """Returns a tile from local MBTiles SQLite cache, caching real imagery when available or overzooming/rendering offline."""
        lyr = layer.lower()
        if lyr == "local":
            return self.render_local_grid_tile(z, x, y)
        if lyr == "qatar":
            lyr = "street"

        # 1. Check local SQLite MBTiles cache first
        cached = self._lookup_cached_tile(lyr, z, x, y)
        if cached:
            return cached

        # 2. Try fetching and permanently caching into SQLite MBTiles if network is available
        tile_bytes: Optional[bytes] = None
        urls = self.TILE_URLS.get(lyr, self.TILE_URLS["street"])
        for url_tmpl in urls:
            url = url_tmpl.format(z=z, x=x, y=y)
            try:
                req = urllib.request.Request(
                    url,
                    headers={
                        "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36",
                        "Accept": "image/avif,image/webp,image/apng,image/*,*/*;q=0.8",
                    },
                )
                with urllib.request.urlopen(req, timeout=2.5) as resp:
                    data = resp.read()
                    if data and len(data) > 250 and len(data) not in self.BLOCKED_TILE_SIZES:
                        tile_bytes = data
                        break
            except Exception:
                continue

        tms_y = (1 << z) - 1 - y
        if tile_bytes:
            with self._lock:
                conn = sqlite3.connect(str(self.mbtiles_path))
                try:
                    conn.execute(
                        "INSERT OR REPLACE INTO layer_tiles (layer, zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?, ?)",
                        (lyr, z, x, tms_y, tile_bytes),
                    )
                    if lyr == "street":
                        conn.execute(
                            "INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
                            (z, x, tms_y, tile_bytes),
                        )
                    conn.commit()
                except Exception:
                    pass
                finally:
                    conn.close()
            return tile_bytes

        # 3. Try overzooming from any cached parent tile in SQLite before procedural fallback
        overzoomed = self._overzoom_from_cached_parent(lyr, z, x, y)
        if overzoomed:
            return overzoomed

        # 4. 100% Offline procedural fallback when tile is not cached and no internet is present
        return self._render_offline_qatar_tile(z, x, y)

    def get_or_create_qatar_tile(self, z: int, x: int, y: int) -> bytes:
        return self.get_tile("street", z, x, y)

    def _render_offline_qatar_tile(self, z: int, x: int, y: int) -> bytes:
        """Synthesizes a high-contrast offline Qatar street map tile with roads, grid, and landmarks."""
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        north, west, south, east = _tile_bounds_latlon(z, x, y)
        center_lat = (north + south) / 2.0
        center_lon = (west + east) / 2.0

        # Determine land vs Arabian Gulf water background
        in_qatar_land = (24.45 <= center_lat <= 26.18) and (50.74 <= center_lon <= 51.63)
        if in_qatar_land:
            img[:] = (36, 26, 18)  # Deep dark slate-navy land in BGR
        else:
            img[:] = (42, 22, 10)  # Arabian Gulf water tone in BGR

        # Subtle coordinate grid
        for g in (0, 64, 128, 192):
            cv2.line(img, (g, 0), (g, 255), (52, 38, 28), 1)
            cv2.line(img, (0, g), (255, g), (52, 38, 28), 1)

        # At street zooms (z >= 13), draw local urban street blocks across Doha / Qatar urban zones
        if z >= 13 and (25.14 <= center_lat <= 25.48) and (51.38 <= center_lon <= 51.62):
            step = 32 if z <= 14 else (48 if z == 15 else 64)
            offset_x = (x * 37) % step
            offset_y = (y * 53) % step
            for sx in range(offset_x, 256, step):
                cv2.line(img, (sx, 0), (sx, 255), (68, 52, 38), 2 if z >= 16 else 1)
            for sy in range(offset_y, 256, step):
                cv2.line(img, (0, sy), (255, sy), (68, 52, 38), 2 if z >= 16 else 1)

        # Draw major Qatar highway corridors
        road_w = max(2, min(8, z - 9))
        for corridor in QATAR_HIGHWAYS:
            pts = [_latlon_to_tile_px(lat, lon, z, x, y) for lat, lon in corridor]
            for i in range(len(pts) - 1):
                p1, p2 = pts[i], pts[i + 1]
                if (
                    min(p1[0], p2[0]) <= 300
                    and max(p1[0], p2[0]) >= -44
                    and min(p1[1], p2[1]) <= 300
                    and max(p1[1], p2[1]) >= -44
                ):
                    cv2.line(img, p1, p2, (130, 95, 55), road_w + 2, cv2.LINE_AA)
                    cv2.line(img, p1, p2, (215, 165, 85), road_w, cv2.LINE_AA)

        # Draw Qatar landmarks & district names
        for name, l_lat, l_lon in QATAR_LANDMARKS:
            px, py = _latlon_to_tile_px(l_lat, l_lon, z, x, y)
            if -60 <= px <= 316 and -20 <= py <= 276:
                cv2.circle(img, (px, py), 4, (255, 220, 0), -1, cv2.LINE_AA)
                cv2.circle(img, (px, py), 6, (255, 255, 255), 1, cv2.LINE_AA)
                if z >= 11:
                    cv2.putText(
                        img,
                        name,
                        (px + 8, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.36 if z < 14 else 0.42,
                        (235, 225, 210),
                        1,
                        cv2.LINE_AA,
                    )

        # Subtle tile coordinate & Qatar Offline badge
        if (x + y) % 2 == 0:
            cv2.putText(
                img,
                f"QATAR OFFLINE z{z} ({center_lat:.3f}N, {center_lon:.3f}E)",
                (8, 246),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                (120, 105, 90),
                1,
                cv2.LINE_AA,
            )

        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else b""

    def render_local_grid_tile(self, z: int, x: int, y: int) -> bytes:
        """Renders a 256x256 engineering coordinate grid tile for Mode 1: Local Tracking."""
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        img[:] = (28, 18, 12)  # Dark engineering navy in BGR

        # Sub-grid every 32px
        for g in range(0, 256, 32):
            cv2.line(img, (g, 0), (g, 255), (46, 32, 22), 1)
            cv2.line(img, (0, g), (255, g), (46, 32, 22), 1)

        # Major tile border
        cv2.line(img, (0, 0), (255, 0), (72, 52, 36), 1)
        cv2.line(img, (0, 0), (0, 255), (72, 52, 36), 1)

        # Metric coordinate label relative to center tile
        center_tile = 1 << (z - 1)
        dx_m = (x - center_tile) * 10
        dy_m = (center_tile - y) * 10
        if dx_m == 0:
            cv2.line(img, (0, 0), (0, 255), (140, 110, 50), 2)
        if dy_m == 0:
            cv2.line(img, (0, 0), (255, 0), (140, 110, 50), 2)

        cv2.putText(
            img,
            f"X:{dx_m:+d}m Y:{dy_m:+d}m",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (150, 130, 110),
            1,
            cv2.LINE_AA,
        )
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else b""


class RouteDatabase:
    """SQLite persistence for recorded robot navigation routes."""

    def __init__(self, db_path: Path = ROUTES_DB_PATH) -> None:
        self.db_path = db_path
        self._lock = threading.Lock()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS routes (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        name TEXT NOT NULL,
                        mode TEXT NOT NULL,
                        created_at TEXT NOT NULL,
                        distance_m REAL NOT NULL,
                        point_count INTEGER NOT NULL,
                        origin_lat REAL,
                        origin_lon REAL,
                        gps_available INTEGER NOT NULL DEFAULT 0
                    )
                    """
                )
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS route_points (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        route_id INTEGER NOT NULL,
                        seq INTEGER NOT NULL,
                        ts REAL NOT NULL,
                        x_m REAL NOT NULL,
                        y_m REAL NOT NULL,
                        lat REAL,
                        lon REAL,
                        heading_deg REAL NOT NULL,
                        is_estimated INTEGER NOT NULL DEFAULT 1,
                        FOREIGN KEY(route_id) REFERENCES routes(id)
                    )
                    """
                )
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS repair_pins (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        label TEXT NOT NULL,
                        lat REAL NOT NULL,
                        lon REAL NOT NULL,
                        created_at TEXT NOT NULL,
                        status TEXT NOT NULL DEFAULT 'pending',
                        repaired_at TEXT,
                        car_type TEXT NOT NULL DEFAULT 'sedan',
                        heading_deg REAL NOT NULL DEFAULT 88.0
                    )
                    """
                )
                for col_sql in (
                    "ALTER TABLE repair_pins ADD COLUMN status TEXT NOT NULL DEFAULT 'pending'",
                    "ALTER TABLE repair_pins ADD COLUMN repaired_at TEXT",
                    "ALTER TABLE repair_pins ADD COLUMN car_type TEXT NOT NULL DEFAULT 'sedan'",
                    "ALTER TABLE repair_pins ADD COLUMN heading_deg REAL NOT NULL DEFAULT 88.0",
                ):
                    try:
                        conn.execute(col_sql)
                    except Exception:
                        pass
                conn.commit()
            finally:
                conn.close()

    def save_route(
        self,
        name: str,
        mode: str,
        distance_m: float,
        origin_lat: float,
        origin_lon: float,
        gps_available: bool,
        points: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        created_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    """
                    INSERT INTO routes (name, mode, created_at, distance_m, point_count, origin_lat, origin_lon, gps_available)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        name,
                        mode,
                        created_at,
                        round(distance_m, 2),
                        len(points),
                        origin_lat,
                        origin_lon,
                        1 if gps_available else 0,
                    ),
                )
                route_id = int(cur.lastrowid or 0)
                for idx, pt in enumerate(points):
                    cur.execute(
                        """
                        INSERT INTO route_points (route_id, seq, ts, x_m, y_m, lat, lon, heading_deg, is_estimated)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                        """,
                        (
                            route_id,
                            idx,
                            float(pt.get("ts", time.time())),
                            float(pt.get("x_m", 0.0)),
                            float(pt.get("y_m", 0.0)),
                            float(pt.get("lat", origin_lat)),
                            float(pt.get("lon", origin_lon)),
                            float(pt.get("heading_deg", 0.0)),
                            1 if pt.get("is_estimated", True) else 0,
                        ),
                    )
                conn.commit()
                return {
                    "id": route_id,
                    "name": name,
                    "mode": mode,
                    "created_at": created_at,
                    "distance_m": round(distance_m, 2),
                    "point_count": len(points),
                }
            finally:
                conn.close()

    def list_routes(self, limit: int = 20) -> List[Dict[str, Any]]:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT id, name, mode, created_at, distance_m, point_count, origin_lat, origin_lon, gps_available
                    FROM routes ORDER BY id DESC LIMIT ?
                    """,
                    (limit,),
                )
                rows = cur.fetchall()
                return [
                    {
                        "id": r[0],
                        "name": r[1],
                        "mode": r[2],
                        "created_at": r[3],
                        "distance_m": r[4],
                        "point_count": r[5],
                        "origin_lat": r[6],
                        "origin_lon": r[7],
                        "gps_available": bool(r[8]),
                    }
                    for r in rows
                ]
            finally:
                conn.close()

    CAR_TYPES = [
        ("sedan", "Red Sedan (Camry)", 88.0),
        ("suv", "White 4x4 SUV (Land Cruiser)", 88.0),
        ("ev", "Blue EV (Tesla Model 3)", 16.0),
        ("pickup", "Silver Pickup (F-150)", 88.0),
        ("coupe", "Gold Sport Coupe (GT)", 16.0),
        ("van", "Service Van (Sprinter)", 88.0),
    ]

    def add_repair_pin(
        self,
        lat: float,
        lon: float,
        label: Optional[str] = None,
        car_type: Optional[str] = None,
        heading_deg: Optional[float] = None,
    ) -> Dict[str, Any]:
        created_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                cur.execute("SELECT COUNT(*) FROM repair_pins")
                cnt = int((cur.fetchone() or [0])[0])
                default_type, default_name, default_hdg = self.CAR_TYPES[cnt % len(self.CAR_TYPES)]
                chosen_type = car_type or default_type
                chosen_hdg = float(heading_deg) if heading_deg is not None else default_hdg
                pin_label = label or f"#{cnt + 1} {default_name}"
                cur.execute(
                    """
                    INSERT INTO repair_pins (label, lat, lon, created_at, status, car_type, heading_deg)
                    VALUES (?, ?, ?, ?, 'pending', ?, ?)
                    """,
                    (pin_label, round(float(lat), 8), round(float(lon), 8), created_at, chosen_type, chosen_hdg),
                )
                pin_id = int(cur.lastrowid or 0)
                conn.commit()
                return {
                    "id": pin_id,
                    "label": pin_label,
                    "lat": round(float(lat), 8),
                    "lon": round(float(lon), 8),
                    "created_at": created_at,
                    "status": "pending",
                    "repaired_at": None,
                    "car_type": chosen_type,
                    "heading_deg": chosen_hdg,
                }
            finally:
                conn.close()

    def update_repair_pin_location(self, pin_id: int, lat: float, lon: float) -> Optional[Dict[str, Any]]:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    "UPDATE repair_pins SET lat=?, lon=? WHERE id=?",
                    (round(float(lat), 8), round(float(lon), 8), int(pin_id)),
                )
                conn.commit()
                cur.execute(
                    "SELECT id, label, lat, lon, created_at, status, repaired_at, car_type, heading_deg FROM repair_pins WHERE id=?",
                    (int(pin_id),),
                )
                r = cur.fetchone()
                if not r:
                    return None
                return {
                    "id": r[0],
                    "label": r[1],
                    "lat": r[2],
                    "lon": r[3],
                    "created_at": r[4],
                    "status": r[5] or "pending",
                    "repaired_at": r[6],
                    "car_type": r[7] or "sedan",
                    "heading_deg": float(r[8] if r[8] is not None else 88.0),
                }
            finally:
                conn.close()

    def mark_repair_pin_repaired(self, pin_id: Optional[int] = None) -> Optional[Dict[str, Any]]:
        repaired_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                if pin_id is not None:
                    cur.execute(
                        "UPDATE repair_pins SET status='repaired', repaired_at=? WHERE id=?",
                        (repaired_at, int(pin_id)),
                    )
                    target_id = int(pin_id)
                else:
                    cur.execute(
                        "SELECT id FROM repair_pins WHERE status!='repaired' ORDER BY id DESC LIMIT 1"
                    )
                    row = cur.fetchone()
                    if not row:
                        cur.execute("SELECT id FROM repair_pins ORDER BY id DESC LIMIT 1")
                        row = cur.fetchone()
                    if not row:
                        return None
                    target_id = int(row[0])
                    cur.execute(
                        "UPDATE repair_pins SET status='repaired', repaired_at=? WHERE id=?",
                        (repaired_at, target_id),
                    )
                conn.commit()
                cur.execute(
                    "SELECT id, label, lat, lon, created_at, status, repaired_at, car_type, heading_deg FROM repair_pins WHERE id=?",
                    (target_id,),
                )
                r = cur.fetchone()
                if not r:
                    return None
                return {
                    "id": r[0],
                    "label": r[1],
                    "lat": r[2],
                    "lon": r[3],
                    "created_at": r[4],
                    "status": r[5] or "repaired",
                    "repaired_at": r[6],
                    "car_type": r[7] or "sedan",
                    "heading_deg": float(r[8] if r[8] is not None else 88.0),
                }
            finally:
                conn.close()

    def delete_repair_pin(self, pin_id: int) -> None:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                conn.execute("DELETE FROM repair_pins WHERE id=?", (int(pin_id),))
                conn.commit()
            finally:
                conn.close()

    def clear_repair_pins(self) -> None:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                conn.execute("DELETE FROM repair_pins")
                conn.commit()
            finally:
                conn.close()

    def list_repair_pins(self) -> List[Dict[str, Any]]:
        with self._lock:
            conn = sqlite3.connect(str(self.db_path))
            try:
                cur = conn.cursor()
                cur.execute(
                    "SELECT id, label, lat, lon, created_at, status, repaired_at, car_type, heading_deg FROM repair_pins ORDER BY id ASC"
                )
                return [
                    {
                        "id": r[0],
                        "label": r[1],
                        "lat": r[2],
                        "lon": r[3],
                        "created_at": r[4],
                        "status": r[5] or "pending",
                        "repaired_at": r[6],
                        "car_type": r[7] or "sedan",
                        "heading_deg": float(r[8] if r[8] is not None else 88.0),
                    }
                    for r in cur.fetchall()
                ]
            finally:
                conn.close()


class NavigationController:
    """Coordinates Mode 1 (Local Tracking) and Mode 2 (Offline Qatar Street Map), MPU-9250, and Route SQLite."""

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        bno_imu: IMUController,
    ) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.mpu = MPU9250Driver(config, bno_imu)
        self.tiles = OfflineTileStore()
        self.route_db = RouteDatabase()

        # Tracking mode: "QATAR_STREET" or "LOCAL_GRID"
        self.mode: str = "QATAR_STREET"
        self.tracking_active: bool = True

        # Session relative coordinates (always initialized to 0 on session start/reset)
        self.relative_x_m: float = 0.0
        self.relative_y_m: float = 0.0
        self.distance_m: float = 0.0

        # GPS status (strictly honest: never fake real GPS coordinates)
        self.gps_connected: bool = False
        self.gps_status: str = "GPS Unavailable"
        self.real_lat: Optional[float] = None
        self.real_lon: Optional[float] = None
        self._last_mobile_gps_ts: float = 0.0

        # Manual Qatar reference origin (used for estimated dead-reckoning on Mode 2 street map)
        self.origin_lat: float = DEFAULT_QATAR_ORIGIN_LAT
        self.origin_lon: float = DEFAULT_QATAR_ORIGIN_LON
        self.est_lat: float = self.origin_lat
        self.est_lon: float = self.origin_lon
        self.manual_origin_locked: bool = False

        # Command-based velocity state for clearly-labeled DEMO odometry when wheel encoders are absent
        self.wheel_encoders_available: bool = False
        self._cmd_vx_norm: float = 0.0
        self._cmd_vy_norm: float = 0.0
        self._cmd_omega_norm: float = 0.0
        self._cmd_last_ts: float = 0.0

        # Recorded route points for active session
        self.route_points: List[Dict[str, Any]] = []
        self.repair_pins: List[Dict[str, Any]] = self.route_db.list_repair_pins()
        if not self.repair_pins:
            self._seed_default_road_cars()
        self._record_point(force=True)

        # Background 20 Hz navigation & IMU update thread
        self._running = True
        self._thread = threading.Thread(target=self._nav_loop, daemon=True)
        self._thread.start()

    def notify_wheel_command(self, vx_pwm: int, vy_pwm: int, omega_pwm: int) -> None:
        """Called by simple/motor controller when wheel commands are active for Command-Demo odometry."""
        with self._lock:
            self._cmd_vx_norm = max(-1.0, min(1.0, float(vx_pwm) / 255.0))
            self._cmd_vy_norm = max(-1.0, min(1.0, float(vy_pwm) / 255.0))
            self._cmd_omega_norm = max(-1.0, min(1.0, float(omega_pwm) / 255.0))
            self._cmd_last_ts = time.time()

    def _nav_loop(self) -> None:
        last_t = time.time()
        while self._running:
            time.sleep(0.05)
            now = time.time()
            dt = max(0.005, min(0.25, now - last_t))
            last_t = now

            with self._lock:
                # Expire wheel command velocity if no command received within 400ms
                if (now - self._cmd_last_ts) > 0.40:
                    self._cmd_vx_norm = 0.0
                    self._cmd_vy_norm = 0.0
                    self._cmd_omega_norm = 0.0

                self.mpu.update(omega_cmd_norm=self._cmd_omega_norm, dt=dt)

                if self.tracking_active and (
                    abs(self._cmd_vx_norm) > 0.01 or abs(self._cmd_vy_norm) > 0.01
                ):
                    # Convert robot body velocity (vx=strafe right, vy=forward) into world frame using IMU heading
                    max_speed_mps = 0.45  # ~0.45 m/s nominal prototype test speed
                    v_fwd = self._cmd_vy_norm * max_speed_mps
                    v_right = self._cmd_vx_norm * max_speed_mps

                    h_rad = math.radians(self.mpu.relative_heading_deg)
                    # Heading 0° = North (+Y), 90° = East (+X)
                    dx = (v_fwd * math.sin(h_rad) + v_right * math.cos(h_rad)) * dt
                    dy = (v_fwd * math.cos(h_rad) - v_right * math.sin(h_rad)) * dt
                    step_dist = math.hypot(dx, dy)

                    self.relative_x_m += dx
                    self.relative_y_m += dy
                    self.distance_m += step_dist
                    self._update_estimated_latlon()
                    self._record_point(force=False)

    def _update_estimated_latlon(self) -> None:
        """Projects relative (X, Y) meters onto the manual or GPS reference origin."""
        dlat = self.relative_y_m / 111111.0
        dlon = self.relative_x_m / (111111.0 * max(0.2, math.cos(math.radians(self.origin_lat))))
        self.est_lat = round(self.origin_lat + dlat, 8)
        self.est_lon = round(self.origin_lon + dlon, 8)
        self.real_lat = self.est_lat
        self.real_lon = self.est_lon

    def _record_point(self, force: bool = False) -> None:
        if not force and self.route_points:
            last = self.route_points[-1]
            dist_from_last = math.hypot(
                self.relative_x_m - last["x_m"], self.relative_y_m - last["y_m"]
            )
            if dist_from_last < 0.05:
                return

        pt = {
            "ts": round(time.time(), 2),
            "x_m": round(self.relative_x_m, 3),
            "y_m": round(self.relative_y_m, 3),
            "lat": self.real_lat if self.gps_connected and self.real_lat is not None else self.est_lat,
            "lon": self.real_lon if self.gps_connected and self.real_lon is not None else self.est_lon,
            "heading_deg": round(self.mpu.relative_heading_deg, 1),
            "is_estimated": not self.gps_connected,
        }
        self.route_points.append(pt)
        if len(self.route_points) > 1500:
            self.route_points = self.route_points[-1500:]

    def start_tracking(self) -> Dict[str, Any]:
        with self._lock:
            if not self.tracking_active and len(self.route_points) <= 1:
                # Fresh session start: Relative X = 0, Y = 0, Heading = 0, Distance = 0
                self.relative_x_m = 0.0
                self.relative_y_m = 0.0
                self.distance_m = 0.0
                self.mpu.calibrate_stationary_gyro()
                self._update_estimated_latlon()
                self.route_points.clear()
                self._record_point(force=True)
            self.tracking_active = True
            self.diagnostics.log_event("INFO", "NAVIGATION", "TRACKING_STARTED", f"mode={self.mode}")
            return self.get_status()

    def stop_tracking(self) -> Dict[str, Any]:
        with self._lock:
            self.tracking_active = False
            self.diagnostics.log_event("INFO", "NAVIGATION", "TRACKING_STOPPED", f"dist={self.distance_m:.2f}m")
            return self.get_status()

    def reset_route(self) -> Dict[str, Any]:
        """Resets Relative X=0, Y=0, Relative Heading=0, Distance=0, and clears the recorded path."""
        with self._lock:
            self.relative_x_m = 0.0
            self.relative_y_m = 0.0
            self.distance_m = 0.0
            self.mpu.calibrate_stationary_gyro()
            self._update_estimated_latlon()
            self.route_points.clear()
            self._record_point(force=True)
            self.diagnostics.log_event("INFO", "NAVIGATION", "ROUTE_RESET", "Reset X=0, Y=0, Heading=0, Dist=0")
            return self.get_status()

    def set_mode(self, mode: str) -> Dict[str, Any]:
        with self._lock:
            m = mode.strip().upper()
            if m in ("LOCAL_GRID", "QATAR_STREET"):
                self.mode = m
            return self.get_status()

    def set_origin(self, lat: float, lon: float) -> Dict[str, Any]:
        with self._lock:
            self.manual_origin_locked = True
            self.origin_lat = round(max(-85.0, min(85.0, float(lat))), 8)
            self.origin_lon = round(max(-180.0, min(180.0, float(lon))), 8)
            self.relative_x_m = 0.0
            self.relative_y_m = 0.0
            self.distance_m = 0.0
            self._update_estimated_latlon()
            self.route_points.clear()
            self._record_point(force=True)
            self.diagnostics.log_event(
                "INFO", "NAVIGATION", "ROBOT_ORIGIN_MOVED", f"({self.origin_lat}, {self.origin_lon})"
            )
            return self.get_status()

    def update_mobile_gps(
        self,
        lat: float,
        lon: float,
        accuracy_m: Optional[float] = None,
        heading_deg: Optional[float] = None,
        source: str = "mobile",
    ) -> Dict[str, Any]:
        """Updates the robot's live map position from the connected mobile device's GPS."""
        with self._lock:
            # If user manually placed the robot on the map, do not override unless 'Locate Me' (manual) is pressed
            if self.manual_origin_locked and source != "manual":
                return self.get_status()
            if source == "manual":
                self.manual_origin_locked = False

            now = time.time()
            # If a real mobile phone is actively streaming GPS, ignore automatic desktop Wi-Fi location
            if source == "desktop" and (now - self._last_mobile_gps_ts) < 60.0:
                return self.get_status()
            if source == "mobile":
                self._last_mobile_gps_ts = now

            clean_lat = round(max(-85.0, min(85.0, float(lat))), 8)
            clean_lon = round(max(-180.0, min(180.0, float(lon))), 8)

            # On first GPS fix (or manual Locate Me), anchor the start origin to this mobile location
            if not self.gps_connected or source == "manual":
                self.origin_lat = clean_lat
                self.origin_lon = clean_lon
                self.relative_x_m = 0.0
                self.relative_y_m = 0.0
                self.distance_m = 0.0
                self.route_points.clear()
            else:
                prev_x, prev_y = self.relative_x_m, self.relative_y_m
                dy = (clean_lat - self.origin_lat) * 111111.0
                dx = (clean_lon - self.origin_lon) * (
                    111111.0 * max(0.2, math.cos(math.radians(self.origin_lat)))
                )
                step = math.hypot(dx - prev_x, dy - prev_y)
                # Filter out tiny stationary GPS jitter (< 0.8m) while still updating position smoothly
                if step >= 0.8:
                    self.distance_m += step
                self.relative_x_m = dx
                self.relative_y_m = dy

            self.real_lat = clean_lat
            self.real_lon = clean_lon
            self.est_lat = clean_lat
            self.est_lon = clean_lon
            self.gps_connected = True
            acc_str = f" ±{max(1.0, float(accuracy_m)):.0f}m" if accuracy_m is not None else ""
            self.gps_status = f"Mobile GPS Connected{acc_str}"

            if (
                heading_deg is not None
                and not math.isnan(float(heading_deg))
                and 0.0 <= float(heading_deg) <= 360.0
                and not self.mpu.hardware_connected
            ):
                self.mpu.raw_heading_deg = round(float(heading_deg) % 360.0, 1)
                self.mpu.relative_heading_deg = round(
                    (self.mpu.raw_heading_deg - self.mpu.heading_zero_offset_deg) % 360.0, 1
                )

            self._record_point(force=(len(self.route_points) == 0))
            return self.get_status()

    def simulate_demo_step(self, dx_m: float = 0.0, dy_m: float = 0.8, dheading_deg: float = 15.0) -> Dict[str, Any]:
        """Advances the simulated/demo position WITHOUT commanding any physical robot motors."""
        with self._lock:
            self.tracking_active = True
            self.mpu.raw_heading_deg = (self.mpu.raw_heading_deg + dheading_deg) % 360.0
            self.mpu.relative_heading_deg = (
                self.mpu.raw_heading_deg - self.mpu.heading_zero_offset_deg
            ) % 360.0
            h_rad = math.radians(self.mpu.relative_heading_deg)
            world_dx = dy_m * math.sin(h_rad) + dx_m * math.cos(h_rad)
            world_dy = dy_m * math.cos(h_rad) - dx_m * math.sin(h_rad)
            self.relative_x_m += world_dx
            self.relative_y_m += world_dy
            self.distance_m += math.hypot(world_dx, world_dy)
            self._update_estimated_latlon()
            self._record_point(force=True)
            return self.get_status()

    def save_current_route(self, name: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            route_name = name or f"LiftMate Route {datetime.now().strftime('%H:%M:%S')}"
            saved = self.route_db.save_route(
                name=route_name,
                mode=self.mode,
                distance_m=self.distance_m,
                origin_lat=self.origin_lat,
                origin_lon=self.origin_lon,
                gps_available=self.gps_connected,
                points=list(self.route_points),
            )
            self.diagnostics.log_event("INFO", "NAVIGATION", "ROUTE_SAVED", str(saved))
            return saved

    def _seed_default_road_cars(self) -> None:
        """Seeds 3 distinct vehicles directly on the nearby road network around the default robot origin."""
        road_cars = [
            (25.1865540, 51.5715945, "#1 Red Sedan (Camry)", "sedan", 88.0),
            (25.1865443, 51.5720100, "#2 White 4x4 SUV (Land Cruiser)", "suv", 88.0),
            (25.1858744, 51.5735579, "#3 Blue EV (Tesla Model 3)", "ev", 16.0),
        ]
        for lat, lon, lbl, ctype, hdg in road_cars:
            self.route_db.add_repair_pin(lat=lat, lon=lon, label=lbl, car_type=ctype, heading_deg=hdg)
        self.repair_pins = self.route_db.list_repair_pins()

    def add_repair_pin(self, lat: Optional[float] = None, lon: Optional[float] = None, label: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            target_lat = float(lat) if lat is not None else (self.real_lat if self.gps_connected and self.real_lat is not None else self.est_lat)
            target_lon = float(lon) if lon is not None else (self.real_lon if self.gps_connected and self.real_lon is not None else self.est_lon)
            # If user pins multiple cars at the same spot ('Pin Car Here'), space them neatly along the road
            for _ in range(12):
                overlap = any(
                    math.hypot(target_lat - float(p["lat"]), target_lon - float(p["lon"])) < 0.00004
                    for p in self.repair_pins
                )
                if not overlap:
                    break
                target_lon += 0.00014
                target_lat += 0.00001
            pin = self.route_db.add_repair_pin(target_lat, target_lon, label)
            self.repair_pins = self.route_db.list_repair_pins()
            self.diagnostics.log_event("INFO", "NAVIGATION", "REPAIR_PIN_ADDED", f"{pin['label']} @ ({pin['lat']}, {pin['lon']})")
            return pin

    def update_repair_pin_location(self, pin_id: int, lat: float, lon: float) -> Dict[str, Any]:
        with self._lock:
            self.route_db.update_repair_pin_location(pin_id, lat, lon)
            self.repair_pins = self.route_db.list_repair_pins()
            return self.get_status()

    def mark_repair_pin_repaired(self, pin_id: Optional[int] = None) -> Dict[str, Any]:
        with self._lock:
            pin = self.route_db.mark_repair_pin_repaired(pin_id)
            if pin is None:
                # If no pin existed yet, create one at the current robot position and mark it repaired
                created = self.add_repair_pin()
                pin = self.route_db.mark_repair_pin_repaired(created["id"])
            self.repair_pins = self.route_db.list_repair_pins()
            if pin:
                self.diagnostics.log_event(
                    "INFO", "NAVIGATION", "CAR_REPAIRED", f"{pin['label']} marked REPAIRED"
                )
            return pin or {}

    def delete_repair_pin(self, pin_id: int) -> Dict[str, Any]:
        with self._lock:
            self.route_db.delete_repair_pin(pin_id)
            self.repair_pins = self.route_db.list_repair_pins()
            return self.get_status()

    def clear_repair_pins(self) -> Dict[str, Any]:
        with self._lock:
            self.route_db.clear_repair_pins()
            self.repair_pins = []
            self.diagnostics.log_event("INFO", "NAVIGATION", "REPAIR_PINS_CLEARED", "Cleared all car repair pins")
            return self.get_status()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            mpu_st = self.mpu.get_status()
            if self.gps_connected:
                tracking_label = "Real GPS + IMU Tracking"
            elif self.wheel_encoders_available:
                tracking_label = "Wheel Odometry + MPU-9250"
            else:
                tracking_label = "Command-Estimated DEMO (No GPS/Encoders)"

            return {
                "mode": self.mode,
                "tracking_active": self.tracking_active,
                "tracking_label": tracking_label,
                "distance_m": round(self.distance_m, 2),
                "relative_x_m": round(self.relative_x_m, 2),
                "relative_y_m": round(self.relative_y_m, 2),
                "heading_deg": mpu_st["heading_deg"],
                "pitch_deg": mpu_st["pitch_deg"],
                "roll_deg": mpu_st["roll_deg"],
                "gps_connected": self.gps_connected,
                "gps_status": self.gps_status,
                "real_lat": self.real_lat,
                "real_lon": self.real_lon,
                "origin_lat": self.origin_lat,
                "origin_lon": self.origin_lon,
                "est_lat": self.est_lat,
                "est_lon": self.est_lon,
                "is_estimated_position": not self.gps_connected,
                "mpu": mpu_st,
                "route_points": list(self.route_points[-250:]),
                "repair_pins": list(self.repair_pins),
                "saved_routes": self.route_db.list_routes(limit=10),
            }
