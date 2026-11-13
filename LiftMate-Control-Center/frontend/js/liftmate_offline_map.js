/**
 * LiftMateOfflineMap — Professional Offline Street, Satellite & Tactical Map Component
 * Features local SQLite MBTiles caching, Satellite/Street/Dark layers, Top-View 4-Wheel Mecanum Robot SVG,
 * and 3 streamlined actions: Full Screen Map, Reset Gyro, and Locate Me.
 */
(function () {
  "use strict";

  const TOP_VIEW_ROBOT_SVG = `
    <svg viewBox="0 0 64 64" width="46" height="46" xmlns="http://www.w3.org/2000/svg">
      <!-- Outer Radar Halo -->
      <circle cx="32" cy="32" r="29" fill="rgba(0, 240, 255, 0.16)" stroke="rgba(0, 240, 255, 0.55)" stroke-width="1.5"/>
      <!-- Forward Heading Cone -->
      <path d="M32 2 L22 16 L42 16 Z" fill="#00f0ff" opacity="0.95"/>
      <!-- 4 Mecanum Wheels (Top-View with Diagonal Rollers) -->
      <g transform="translate(9, 14)">
        <rect x="0" y="0" width="8" height="14" rx="2" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.2"/>
        <line x1="1" y1="3" x2="7" y2="6" stroke="#38bdf8" stroke-width="1.3"/>
        <line x1="1" y1="8" x2="7" y2="11" stroke="#38bdf8" stroke-width="1.3"/>
      </g>
      <g transform="translate(47, 14)">
        <rect x="0" y="0" width="8" height="14" rx="2" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.2"/>
        <line x1="7" y1="3" x2="1" y2="6" stroke="#38bdf8" stroke-width="1.3"/>
        <line x1="7" y1="8" x2="1" y2="11" stroke="#38bdf8" stroke-width="1.3"/>
      </g>
      <g transform="translate(9, 36)">
        <rect x="0" y="0" width="8" height="14" rx="2" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.2"/>
        <line x1="7" y1="3" x2="1" y2="6" stroke="#38bdf8" stroke-width="1.3"/>
        <line x1="7" y1="8" x2="1" y2="11" stroke="#38bdf8" stroke-width="1.3"/>
      </g>
      <g transform="translate(47, 36)">
        <rect x="0" y="0" width="8" height="14" rx="2" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.2"/>
        <line x1="1" y1="3" x2="7" y2="6" stroke="#38bdf8" stroke-width="1.3"/>
        <line x1="1" y1="8" x2="7" y2="11" stroke="#38bdf8" stroke-width="1.3"/>
      </g>
      <!-- LiftMate Main Chassis -->
      <rect x="17" y="12" width="30" height="40" rx="5" fill="#0f172a" stroke="#6366f1" stroke-width="2.2"/>
      <!-- Lifting Jack Frame -->
      <rect x="21" y="20" width="22" height="24" rx="3" fill="#1e1b4b" stroke="#00f0ff" stroke-width="1.4"/>
      <circle cx="24" cy="23" r="2" fill="#00f0ff"/>
      <circle cx="40" cy="23" r="2" fill="#00f0ff"/>
      <circle cx="24" cy="41" r="2" fill="#a855f7"/>
      <circle cx="40" cy="41" r="2" fill="#a855f7"/>
      <!-- Front Camera & Direction Arrow -->
      <rect x="27" y="13" width="10" height="4" rx="1.5" fill="#00f0ff"/>
      <path d="M32 24 L27 35 L32 32.5 L37 35 Z" fill="#ffffff"/>
    </svg>
  `;

  const CAR_STYLES = {
    sedan: {
      name: "Red Sedan",
      body: "#ef4444",
      stroke: "#fecaca",
      roof: "#991b1b",
      glass: "#1e293b",
      extra: `<line x1="16" y1="12" x2="32" y2="12" stroke="#fca5a5" stroke-width="1"/>`,
    },
    suv: {
      name: "White 4x4 SUV",
      body: "#f8fafc",
      stroke: "#94a3b8",
      roof: "#e2e8f0",
      glass: "#0f172a",
      extra: `
        <line x1="15" y1="15" x2="15" y2="35" stroke="#475569" stroke-width="1.5"/>
        <line x1="33" y1="15" x2="33" y2="35" stroke="#475569" stroke-width="1.5"/>
        <rect x="19" y="42" width="10" height="3.5" rx="1.5" fill="#1e293b" stroke="#94a3b8" stroke-width="0.8"/>
      `,
    },
    ev: {
      name: "Blue Electric EV",
      body: "#0ea5e9",
      stroke: "#bae6fd",
      roof: "#0f172a",
      glass: "#020617",
      extra: `<rect x="17" y="16" width="14" height="18" rx="3" fill="#0f172a" stroke="#38bdf8" stroke-width="0.9"/>`,
    },
    pickup: {
      name: "Silver Pickup",
      body: "#94a3b8",
      stroke: "#f1f5f9",
      roof: "#64748b",
      glass: "#0f172a",
      extra: `
        <rect x="16" y="28" width="16" height="13" rx="1.5" fill="#334155" stroke="#cbd5e1" stroke-width="1"/>
        <line x1="19" y1="29" x2="19" y2="40" stroke="#475569" stroke-width="1"/>
        <line x1="24" y1="29" x2="24" y2="40" stroke="#475569" stroke-width="1"/>
        <line x1="29" y1="29" x2="29" y2="40" stroke="#475569" stroke-width="1"/>
      `,
    },
    coupe: {
      name: "Gold Sport Coupe",
      body: "#eab308",
      stroke: "#fef08a",
      roof: "#a16207",
      glass: "#1e293b",
      extra: `
        <line x1="22.5" y1="7" x2="22.5" y2="42" stroke="#1e293b" stroke-width="1.4"/>
        <line x1="25.5" y1="7" x2="25.5" y2="42" stroke="#1e293b" stroke-width="1.4"/>
        <rect x="15" y="40" width="18" height="2.2" rx="1" fill="#1e293b"/>
      `,
    },
    van: {
      name: "Service Van",
      body: "#8b5cf6",
      stroke: "#ddd6fe",
      roof: "#6d28d9",
      glass: "#0f172a",
      extra: `
        <rect x="19" y="19" width="10" height="7" rx="1.5" fill="#ddd6fe" opacity="0.7"/>
        <rect x="19" y="30" width="10" height="7" rx="1.5" fill="#ddd6fe" opacity="0.7"/>
      `,
    },
  };

  const CAR_KEYS = Object.keys(CAR_STYLES);

  function getCarStyle(pin) {
    if (pin && pin.car_type && CAR_STYLES[pin.car_type]) {
      return CAR_STYLES[pin.car_type];
    }
    const idx = Math.max(0, (Number(pin && pin.id ? pin.id : 1) - 1) % CAR_KEYS.length);
    return CAR_STYLES[CAR_KEYS[idx]];
  }

  function buildRoadCarSvg(pin, isRepaired) {
    const st = getCarStyle(pin);
    const hdg = Number(pin && pin.heading_deg != null ? pin.heading_deg : 88);
    const hazardColor = isRepaired ? "#10b981" : "#f59e0b";
    return `
      <svg viewBox="0 0 48 48" width="44" height="44" style="transform:rotate(${hdg}deg);overflow:visible;" xmlns="http://www.w3.org/2000/svg">
        <!-- Road Shadow & Status Halo on Road -->
        <ellipse cx="24" cy="25" rx="14" ry="22" fill="rgba(0,0,0,0.55)"/>
        <ellipse cx="24" cy="24" rx="16" ry="23" fill="none" stroke="${hazardColor}" stroke-width="1.8" stroke-dasharray="${isRepaired ? "none" : "4,3"}" opacity="0.9"/>
        <!-- Headlight Beams on Road -->
        <polygon points="16,7 11,-4 20,-4" fill="rgba(254,240,138,0.42)"/>
        <polygon points="32,7 28,-4 37,-4" fill="rgba(254,240,138,0.42)"/>
        <!-- 4 Road Tires -->
        <rect x="10.5" y="10" width="4" height="8" rx="1.5" fill="#090d16" stroke="#475569" stroke-width="0.8"/>
        <rect x="33.5" y="10" width="4" height="8" rx="1.5" fill="#090d16" stroke="#475569" stroke-width="0.8"/>
        <rect x="10.5" y="30" width="4" height="8" rx="1.5" fill="#090d16" stroke="#475569" stroke-width="0.8"/>
        <rect x="33.5" y="30" width="4" height="8" rx="1.5" fill="#090d16" stroke="#475569" stroke-width="0.8"/>
        <!-- Car Chassis on Road -->
        <rect x="13" y="6" width="22" height="36" rx="6" fill="${st.body}" stroke="${st.stroke}" stroke-width="1.5"/>
        <!-- Front Windshield -->
        <polygon points="15,13 33,13 31,19 17,19" fill="${st.glass}"/>
        <!-- Roof -->
        <rect x="16" y="19" width="16" height="12" rx="2.5" fill="${st.roof}"/>
        <!-- Rear Window -->
        <polygon points="17,31 31,31 33,36 15,36" fill="${st.glass}"/>
        ${st.extra}
        <!-- Front Headlights -->
        <rect x="15" y="6.5" width="4.5" height="2" rx="0.8" fill="#fef08a"/>
        <rect x="28.5" y="6.5" width="4.5" height="2" rx="0.8" fill="#fef08a"/>
        <!-- Rear Taillights / Hazard Blinkers -->
        <rect x="14.5" y="39.5" width="5" height="2.2" rx="0.8" fill="${isRepaired ? "#10b981" : "#fb923c"}"/>
        <rect x="28.5" y="39.5" width="5" height="2.2" rx="0.8" fill="${isRepaired ? "#10b981" : "#fb923c"}"/>
      </svg>
    `;
  }

  function buildSideCarSvg(pin, width = 34, height = 18) {
    const st = getCarStyle(pin);
    return `
      <svg viewBox="0 0 68 32" width="${width}" height="${height}" style="vertical-align:middle;flex-shrink:0;" xmlns="http://www.w3.org/2000/svg">
        <!-- Side Profile Car Body -->
        <path d="M6 22 L10 13 L23 7 L46 7 L56 14 L63 16 L64 23 L6 23 Z" fill="${st.body}" stroke="${st.stroke}" stroke-width="1.6" stroke-linejoin="round"/>
        <!-- Side Windows -->
        <polygon points="24,9 35,9 35,15 16,15" fill="${st.glass}"/>
        <polygon points="38,9 45,9 52,15 38,15" fill="${st.glass}"/>
        <!-- Headlight & Taillight -->
        <rect x="60" y="16" width="4" height="3" rx="1" fill="#fef08a"/>
        <rect x="5" y="16" width="3.5" height="4" rx="1" fill="#ef4444"/>
        <!-- Wheels -->
        <circle cx="18" cy="23" r="6" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.8"/>
        <circle cx="18" cy="23" r="2.2" fill="#94a3b8"/>
        <circle cx="50" cy="23" r="6" fill="#0f172a" stroke="#e2e8f0" stroke-width="1.8"/>
        <circle cx="50" cy="23" r="2.2" fill="#94a3b8"/>
      </svg>
    `;
  }

  class LiftMateOfflineMap {
    constructor(containerId) {
      this.containerId = containerId;
      this.map = null;
      this.robotMarker = null;
      this.startMarker = null;
      this.routePolyline = null;
      this.accuracyCircle = null;

      this.followRobot = true;
      this.isZooming = false;
      this.isDraggingRobot = false;
      this.manualPositionLocked = false;
      this.moveRobotModeActive = false;
      this.hasInitialGpsFix = false;
      this.watchId = null;
      this.mobileCompassHeading = null;
      this.pinModeActive = false;
      this.repairLayerGroup = null;
      this.repairTargetLine = null;
      this.lastPinsSignature = "";
      this.lastPointCount = 0;
      this.lastHeading = 0;
      this.lastLat = 25.185685042934363;
      this.lastLon = 51.57190161939388;
      this.ws = null;

      this.init();
    }

    init() {
      const mapEl = document.getElementById(this.containerId);
      if (!mapEl || typeof L === "undefined") return;

      const DEFAULT_LAT = 25.185685042934363;
      const DEFAULT_LON = 51.57190161939388;

      // Initialize Leaflet Map with full zoom range (1 = World, 22 = Sub-meter Close-up)
      this.map = L.map(this.containerId, {
        center: [DEFAULT_LAT, DEFAULT_LON],
        zoom: 17,
        minZoom: 1,
        maxZoom: 22,
        zoomControl: true,
        scrollWheelZoom: true,
        touchZoom: true,
        doubleClickZoom: true,
        boxZoom: true,
        attributionControl: true,
      });

      this.map.attributionControl.setPrefix("LiftMate Map");
      L.control.scale({ imperial: false, position: "bottomleft" }).addTo(this.map);

      // Add Full Zoom In (Max) & Full Zoom Out (Overview) buttons right below +/- zoom controls
      const FullZoomControl = L.Control.extend({
        options: { position: "topleft" },
        onAdd: () => {
          const container = L.DomUtil.create("div", "leaflet-bar leaflet-control");
          const btnMaxIn = L.DomUtil.create("a", "", container);
          btnMaxIn.href = "#";
          btnMaxIn.title = "Full Zoom In (Max Detail)";
          btnMaxIn.innerHTML = "⊞";
          btnMaxIn.style.fontWeight = "800";
          btnMaxIn.style.fontSize = "15px";

          const btnMaxOut = L.DomUtil.create("a", "", container);
          btnMaxOut.href = "#";
          btnMaxOut.title = "Full Zoom Out (Overview)";
          btnMaxOut.innerHTML = "⊟";
          btnMaxOut.style.fontWeight = "800";
          btnMaxOut.style.fontSize = "15px";

          L.DomEvent.disableClickPropagation(container);
          L.DomEvent.on(btnMaxIn, "click", (e) => {
            L.DomEvent.preventDefault(e);
            this.fullZoomIn();
          });
          L.DomEvent.on(btnMaxOut, "click", (e) => {
            L.DomEvent.preventDefault(e);
            this.fullZoomOut();
          });
          return container;
        },
      });
      this.map.addControl(new FullZoomControl());

      // Define Map Layers (Served & Cached via Local Backend SQLite MBTiles with Overzoom up to z=22)
      const streetLayer = L.tileLayer("/api/map/tiles/street/{z}/{x}/{y}.png?v=3", {
        minZoom: 1,
        maxZoom: 22,
        maxNativeZoom: 19,
        attribution: "© OpenStreetMap",
      });

      const satelliteLayer = L.tileLayer("/api/map/tiles/satellite/{z}/{x}/{y}.png?v=3", {
        minZoom: 1,
        maxZoom: 22,
        maxNativeZoom: 19,
        attribution: "© Esri World Imagery",
      });

      const darkLayer = L.tileLayer("/api/map/tiles/dark/{z}/{x}/{y}.png?v=3", {
        minZoom: 1,
        maxZoom: 22,
        maxNativeZoom: 16,
        attribution: "© Esri Dark Gray",
      });

      const localGridLayer = L.tileLayer("/api/map/tiles/local/{z}/{x}/{y}.png?v=3", {
        minZoom: 1,
        maxZoom: 22,
        maxNativeZoom: 20,
        attribution: "LiftMate Local Grid",
      });

      // Default to Satellite + Street Layers selector
      satelliteLayer.addTo(this.map);

      // Create Start Position Marker (Green)
      const startIcon = L.divIcon({
        className: "liftmate-start-marker",
        html: `<div style="width:14px;height:14px;border-radius:50%;background:#10b981;border:2.5px solid #ffffff;box-shadow:0 0 12px rgba(16,185,129,0.95);"></div>`,
        iconSize: [14, 14],
        iconAnchor: [7, 7],
      });
      this.startMarker = L.marker([DEFAULT_LAT, DEFAULT_LON], { icon: startIcon }).addTo(this.map);

      // Create Recorded Path Polyline (Continuous Blue Route)
      this.routePolyline = L.polyline([[DEFAULT_LAT, DEFAULT_LON]], {
        color: "#00f0ff",
        weight: 4,
        opacity: 0.92,
        lineJoin: "round",
      }).addTo(this.map);

      // Create Live Top-View Mecanum Robot Marker (Draggable to any start location on the map)
      const robotIcon = L.divIcon({
        className: "liftmate-robot-marker",
        html: `<div id="robotSvgRotator" style="width:46px;height:46px;cursor:grab;transition:transform 0.08s linear;transform:rotate(0deg);">${TOP_VIEW_ROBOT_SVG}</div>`,
        iconSize: [46, 46],
        iconAnchor: [23, 23],
      });
      this.robotMarker = L.marker([DEFAULT_LAT, DEFAULT_LON], {
        icon: robotIcon,
        draggable: true,
        zIndexOffset: 1000,
      }).addTo(this.map);

      this.robotMarker.on("dragstart", () => {
        this.isDraggingRobot = true;
      });
      this.robotMarker.on("drag", (e) => {
        const ll = e.target.getLatLng();
        if (this.startMarker) this.startMarker.setLatLng(ll);
      });
      this.robotMarker.on("dragend", (e) => {
        this.isDraggingRobot = false;
        const ll = e.target.getLatLng();
        this.setRobotManualLocation(ll.lat, ll.lng);
      });

      // Car Repair Pins Layer Group & Target Line
      this.repairLayerGroup = L.layerGroup().addTo(this.map);
      this.repairTargetLine = L.polyline([], {
        color: "#f59e0b",
        weight: 3,
        dashArray: "7, 7",
        opacity: 0.9,
      }).addTo(this.map);

      // Add Professional Layer Switcher Control (Top-Right of Map)
      const baseLayers = {
        "🛰️ Satellite": satelliteLayer,
        "🗺️ Street Map": streetLayer,
        "🌑 Dark Tactical": darkLayer,
        "📐 Local Grid": localGridLayer,
      };
      const overlays = {
        "🔧 Car Repair Pins": this.repairLayerGroup,
        "🔵 Route Trail": this.routePolyline,
        "🟢 Start Point": this.startMarker,
      };
      L.control.layers(baseLayers, overlays, { position: "topright", collapsed: true }).addTo(this.map);

      // Tap-on-map handler for moving the robot or pinning a car repair
      this.map.on("click", (e) => {
        if (!e || !e.latlng) return;
        if (this.moveRobotModeActive) {
          this.setRobotManualLocation(e.latlng.lat, e.latlng.lng);
        } else if (this.pinModeActive) {
          this.addRepairPinAt(e.latlng.lat, e.latlng.lng);
        }
      });

      // Track zoom & drag state so WebSocket updates never interrupt user zooming or panning
      this.map.on("dragstart", () => {
        this.followRobot = false;
      });
      this.map.on("zoomstart", () => {
        this.isZooming = true;
      });
      this.map.on("zoomend", () => {
        this.isZooming = false;
      });

      this.connectWebSocket();
      this.fetchInitialStatus();
      this.startMobileGpsSync();

      // Re-trigger mobile GPS sync on first user tap (for iOS Safari / Android gesture requirements)
      const onFirstGesture = () => {
        if (!this.hasInitialGpsFix) {
          this.startMobileGpsSync();
        }
      };
      document.addEventListener("touchstart", onFirstGesture, { once: true, passive: true });
      document.addEventListener("click", onFirstGesture, { once: true, passive: true });
    }

    isMobileDevice() {
      return (
        /Android|iPhone|iPad|iPod|Mobile/i.test(navigator.userAgent || "") ||
        (typeof navigator.maxTouchPoints === "number" && navigator.maxTouchPoints > 1)
      );
    }

    async pushMobileGpsToBackend(lat, lon, accuracy, heading, sourceOverride) {
      if (this.manualPositionLocked && sourceOverride !== "manual") {
        return;
      }
      if (sourceOverride === "manual") {
        this.manualPositionLocked = false;
      }
      const source = sourceOverride || (this.isMobileDevice() ? "mobile" : "desktop");
      const validHeading =
        typeof heading === "number" && !Number.isNaN(heading)
          ? heading
          : this.mobileCompassHeading;

      // Immediately update local robot marker & accuracy halo for zero-latency feedback
      if (this.robotMarker) {
        this.robotMarker.setLatLng([lat, lon]);
      }
      if (this.map && typeof accuracy === "number" && accuracy > 0) {
        const radius = Math.min(80, Math.max(2, accuracy));
        if (!this.accuracyCircle) {
          this.accuracyCircle = L.circle([lat, lon], {
            radius,
            color: "#00f0ff",
            weight: 1,
            fillColor: "#00f0ff",
            fillOpacity: 0.12,
          }).addTo(this.map);
        } else {
          this.accuracyCircle.setLatLng([lat, lon]);
          this.accuracyCircle.setRadius(radius);
        }
      }

      if (!this.hasInitialGpsFix && this.map) {
        this.hasInitialGpsFix = true;
        this.lastLat = lat;
        this.lastLon = lon;
        this.map.setView([lat, lon], Math.max(this.map.getZoom(), 18), { animate: true });
      }

      try {
        const resp = await fetch("/api/navigation/mobile_gps", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            lat,
            lon,
            accuracy: typeof accuracy === "number" ? accuracy : null,
            heading: typeof validHeading === "number" ? validHeading : null,
            source,
          }),
        });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
    }

    startMobileGpsSync() {
      if (!navigator.geolocation) return;

      // Listen to mobile compass heading if available
      if (window.DeviceOrientationEvent && !this._compassBound) {
        this._compassBound = true;
        window.addEventListener(
          "deviceorientation",
          (ev) => {
            if (typeof ev.webkitCompassHeading === "number") {
              this.mobileCompassHeading = ev.webkitCompassHeading;
            } else if (ev.absolute && typeof ev.alpha === "number") {
              this.mobileCompassHeading = (360 - ev.alpha) % 360;
            }
          },
          true
        );
      }

      const geoOpts = {
        enableHighAccuracy: true,
        maximumAge: 1500,
        timeout: 10000,
      };

      navigator.geolocation.getCurrentPosition(
        (pos) => {
          this.pushMobileGpsToBackend(
            pos.coords.latitude,
            pos.coords.longitude,
            pos.coords.accuracy,
            pos.coords.heading
          );
        },
        () => {},
        geoOpts
      );

      if (this.watchId === null) {
        this.watchId = navigator.geolocation.watchPosition(
          (pos) => {
            this.pushMobileGpsToBackend(
              pos.coords.latitude,
              pos.coords.longitude,
              pos.coords.accuracy,
              pos.coords.heading
            );
          },
          () => {},
          geoOpts
        );
      }
    }

    async fetchInitialStatus() {
      try {
        const resp = await fetch("/api/navigation/status");
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
    }

    connectWebSocket() {
      const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
      const wsUrl = `${proto}//${window.location.host}/ws/telemetry`;
      try {
        this.ws = new WebSocket(wsUrl);
        this.ws.onmessage = (event) => {
          try {
            const msg = JSON.parse(event.data);
            if (msg.type === "STATE_UPDATE" && msg.data && msg.data.navigation) {
              this.updateFromTelemetry(msg.data.navigation);
            }
          } catch (_) {}
        };
        this.ws.onclose = () => {
          setTimeout(() => this.connectWebSocket(), 2000);
        };
      } catch (_) {}
    }

    updateFromTelemetry(nav) {
      const heading = Number(nav.heading_deg || 0);
      const rotator = document.getElementById("robotSvgRotator");
      if (rotator && Math.abs(heading - this.lastHeading) >= 0.2) {
        rotator.style.transform = `rotate(${heading.toFixed(1)}deg)`;
        this.lastHeading = heading;
      }

      const lat = Number(nav.est_lat || nav.real_lat || 25.185685042934363);
      const lon = Number(nav.est_lon || nav.real_lon || 51.57190161939388);

      if (this.robotMarker && !this.isDraggingRobot) {
        this.robotMarker.setLatLng([lat, lon]);
      }

      if (this.startMarker && !this.isDraggingRobot) {
        this.startMarker.setLatLng([
          Number(nav.origin_lat || 25.185685042934363),
          Number(nav.origin_lon || 51.57190161939388),
        ]);
      }

      const pts = Array.isArray(nav.route_points) ? nav.route_points : [];
      if (this.routePolyline && pts.length !== this.lastPointCount) {
        this.lastPointCount = pts.length;
        const latlngs = pts.map((p) => [Number(p.lat), Number(p.lon)]);
        if (latlngs.length > 0) {
          this.routePolyline.setLatLngs(latlngs);
        }
      }

      const pins = Array.isArray(nav.repair_pins) ? nav.repair_pins : [];
      this.renderRepairPins(pins, lat, lon);

      const moved =
        Math.abs(lat - this.lastLat) > 1e-7 || Math.abs(lon - this.lastLon) > 1e-7;
      this.lastLat = lat;
      this.lastLon = lon;

      if (this.followRobot && moved && !this.isZooming && this.map) {
        this.map.panTo([lat, lon], { animate: false });
      }
    }

    renderRepairPins(pins, robotLat, robotLon) {
      if (!this.repairLayerGroup) return;
      this.currentPins = pins;
      const sig = JSON.stringify(pins);
      if (sig !== this.lastPinsSignature) {
        this.lastPinsSignature = sig;
        this.repairLayerGroup.clearLayers();

        pins.forEach((pin) => {
          const pLat = Number(pin.lat);
          const pLon = Number(pin.lon);
          const isRepaired = pin.status === "repaired";
          const distM = this.map
            ? this.map.distance([robotLat, robotLon], [pLat, pLon]).toFixed(1)
            : "0.0";

          const badgeBg = isRepaired
            ? "linear-gradient(135deg,#10b981,#059669)"
            : "linear-gradient(135deg,#f59e0b,#ea580c)";
          const stemColor = isRepaired ? "#10b981" : "#f59e0b";
          const statusIcon = isRepaired ? "✅" : "🔧";
          const statusText = isRepaired ? "REPAIRED (Sustainable)" : "On Road — Needs Repair";
          const sideCarSvg = buildSideCarSvg(pin, 26, 13);
          const roadCarSvg = buildRoadCarSvg(pin, isRepaired);
          const popupCarSvg = buildSideCarSvg(pin, 64, 30);

          const pinIcon = L.divIcon({
            className: "liftmate-repair-pin",
            html: `
              <div style="display:flex;flex-direction:column;align-items:center;pointer-events:auto;">
                <!-- Floating Label Badge with Side Car Image -->
                <div style="display:inline-flex;align-items:center;gap:4px;padding:2px 8px;border-radius:999px;background:${badgeBg};color:#fff;font-size:9.5px;font-weight:900;white-space:nowrap;border:1.5px solid #fff;box-shadow:0 3px 10px rgba(0,0,0,0.55);margin-bottom:1px;">
                  ${sideCarSvg}
                  <span>${statusIcon} ${pin.label || "Car Repair"}</span>
                </div>
                <div style="width:2px;height:5px;background:${stemColor};"></div>
                <!-- Top-View Realistic Car Image Directly on the Road -->
                <div style="width:44px;height:44px;display:flex;align-items:center;justify-content:center;">
                  ${roadCarSvg}
                </div>
              </div>
            `,
            iconSize: [140, 72],
            iconAnchor: [70, 50],
          });

          const marker = L.marker([pLat, pLon], {
            icon: pinIcon,
            draggable: true,
            zIndexOffset: 900,
          });
          marker.on("dragend", async (ev) => {
            const newLl = ev.target.getLatLng();
            try {
              const resp = await fetch(`/api/navigation/repair_pins/${pin.id}/move`, {
                method: "PUT",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ lat: newLl.lat, lon: newLl.lng }),
              });
              const data = await resp.json();
              if (data.ok && data.navigation) {
                this.updateFromTelemetry(data.navigation);
              }
            } catch (_) {}
          });
          marker.bindPopup(`
            <div style="font-family:sans-serif;min-width:205px;padding:3px;">
              <div style="display:flex;align-items:center;gap:8px;margin-bottom:5px;">
                <div style="padding:3px 6px;border-radius:8px;background:#0f172a;border:1px solid rgba(255,255,255,0.15);">
                  ${popupCarSvg}
                </div>
                <div>
                  <div style="font-weight:900;font-size:12.5px;color:${isRepaired ? "#10b981" : "#ea580c"};">
                    ${statusIcon} ${pin.label || "Car Repair"}
                  </div>
                  <div style="font-size:10.5px;font-weight:700;color:${isRepaired ? "#10b981" : "#f59e0b"};">
                    ${statusText}
                  </div>
                </div>
              </div>
              <div style="font-size:11px;margin-bottom:2px;"><b>Distance from Robot:</b> ${distM} m</div>
              <div style="font-size:10px;opacity:0.75;margin-bottom:8px;">Road Pos: ${pLat.toFixed(6)}, ${pLon.toFixed(6)}</div>
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;">
                <button onclick="window.liftmateMap && window.liftmateMap.markPinRepaired(${pin.id})"
                  style="padding:6px 6px;border:none;border-radius:7px;background:linear-gradient(135deg,#10b981,#059669);color:#fff;font-size:11px;font-weight:800;cursor:pointer;">
                  ✅ Repaired
                </button>
                <button onclick="window.liftmateMap && window.liftmateMap.deleteRepairPin(${pin.id})"
                  style="padding:6px 6px;border:none;border-radius:7px;background:#ef4444;color:#fff;font-size:11px;font-weight:800;cursor:pointer;">
                  🗑️ Remove
                </button>
              </div>
            </div>
          `);
          marker.addTo(this.repairLayerGroup);
        });
      }

      if (this.repairTargetLine) {
        const pending = pins.filter((p) => p.status !== "repaired");
        if (pending.length > 0) {
          const latest = pending[pending.length - 1];
          this.repairTargetLine.setLatLngs([
            [robotLat, robotLon],
            [Number(latest.lat), Number(latest.lon)],
          ]);
        } else {
          this.repairTargetLine.setLatLngs([]);
        }
      }
    }

    async markPinRepaired(pinId) {
      try {
        const url = pinId
          ? `/api/navigation/repair_pins/${pinId}/repaired`
          : `/api/navigation/repair_pins/repaired_latest`;
        const resp = await fetch(url, { method: "POST" });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
          this.showSustainableRepairModal(data.pin, data.navigation.repair_pins || []);
        }
      } catch (_) {}
    }

    async markLatestCarRepaired() {
      const btn = document.getElementById("btnCarRepaired");
      if (btn) btn.classList.add("active");
      await this.markPinRepaired(null);
      setTimeout(() => {
        if (btn) btn.classList.remove("active");
      }, 350);
    }

    showSustainableRepairModal(pin, allPins) {
      const modal = document.getElementById("sustainableRepairModal");
      if (!modal) return;
      const repairedCount = Math.max(
        1,
        (Array.isArray(allPins) ? allPins : []).filter((p) => p.status === "repaired").length
      );
      const co2SavedKg = (repairedCount * 4.2).toFixed(1);
      const labelEl = document.getElementById("sustModalPinLabel");
      const co2El = document.getElementById("sustModalCo2");
      const countEl = document.getElementById("sustModalCount");
      const removeBtn = document.getElementById("sustModalRemoveBtn");

      if (labelEl) {
        const carIcon = buildSideCarSvg(pin || { id: 1 }, 44, 22);
        const text = pin && pin.label ? `${pin.label} — Repaired` : "Car Repair Completed";
        labelEl.innerHTML = `${carIcon} <span>${text}</span>`;
      }
      if (co2El) {
        co2El.textContent = `${co2SavedKg} kg CO₂ Saved`;
      }
      if (countEl) {
        countEl.textContent = `${repairedCount} Car${repairedCount > 1 ? "s" : ""} Repaired`;
      }
      if (removeBtn) {
        removeBtn.onclick = () => {
          if (pin && pin.id) {
            this.deleteRepairPin(pin.id);
          }
          this.closeSustainableRepairModal();
        };
      }
      modal.classList.add("open");
    }

    closeSustainableRepairModal() {
      const modal = document.getElementById("sustainableRepairModal");
      if (modal) modal.classList.remove("open");
    }

    async addRepairPinAt(lat, lon, label) {
      try {
        const resp = await fetch("/api/navigation/repair_pins", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ lat, lon, label: label || null }),
        });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
    }

    async pinCarRepairHere() {
      const btn = document.getElementById("btnPinCarHere");
      if (btn) btn.classList.add("active");
      const ll = this.map
        ? this.map.getCenter()
        : this.robotMarker
        ? this.robotMarker.getLatLng()
        : { lat: 25.185685042934363, lng: 51.57190161939388 };
      await this.addRepairPinAt(ll.lat, ll.lng);
      setTimeout(() => {
        if (btn) btn.classList.remove("active");
      }, 350);
    }

    async setRobotManualLocation(lat, lon) {
      this.manualPositionLocked = true;
      this.lastPointCount = 0;
      this.lastLat = lat;
      this.lastLon = lon;

      if (this.robotMarker) this.robotMarker.setLatLng([lat, lon]);
      if (this.startMarker) this.startMarker.setLatLng([lat, lon]);
      if (this.routePolyline) this.routePolyline.setLatLngs([[lat, lon]]);
      if (this.accuracyCircle) this.accuracyCircle.setLatLng([lat, lon]);

      try {
        const resp = await fetch("/api/navigation/set_origin", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ lat, lon }),
        });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
    }

    toggleMoveRobotMode() {
      this.moveRobotModeActive = !this.moveRobotModeActive;
      if (this.moveRobotModeActive && this.pinModeActive) {
        this.pinModeActive = false;
        const pinBtn = document.getElementById("btnTapPinCar");
        if (pinBtn) {
          pinBtn.classList.remove("active");
          pinBtn.textContent = "📍 Tap Map to Pin";
        }
      }
      const btn = document.getElementById("btnMoveRobot");
      const mapEl = document.getElementById(this.containerId);
      if (btn) {
        btn.classList.toggle("active", this.moveRobotModeActive);
        btn.textContent = this.moveRobotModeActive ? "🤖 Tap Map: ON" : "🤖 Move Robot";
      }
      if (mapEl) {
        mapEl.style.cursor =
          this.moveRobotModeActive || this.pinModeActive ? "crosshair" : "";
      }
    }

    toggleTapPinMode() {
      this.pinModeActive = !this.pinModeActive;
      if (this.pinModeActive && this.moveRobotModeActive) {
        this.moveRobotModeActive = false;
        const moveBtn = document.getElementById("btnMoveRobot");
        if (moveBtn) {
          moveBtn.classList.remove("active");
          moveBtn.textContent = "🤖 Move Robot";
        }
      }
      const btn = document.getElementById("btnTapPinCar");
      const mapEl = document.getElementById(this.containerId);
      if (btn) {
        btn.classList.toggle("active", this.pinModeActive);
        btn.textContent = this.pinModeActive ? "📍 Tap Map: ON" : "📍 Tap Map to Pin";
      }
      if (mapEl) {
        mapEl.style.cursor =
          this.pinModeActive || this.moveRobotModeActive ? "crosshair" : "";
      }
    }

    async deleteRepairPin(pinId) {
      try {
        const resp = await fetch(`/api/navigation/repair_pins/${pinId}`, { method: "DELETE" });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
    }

    async clearRepairPins() {
      const btn = document.getElementById("btnClearRepairPins");
      if (btn) btn.classList.add("active");
      try {
        const resp = await fetch("/api/navigation/repair_pins/clear", { method: "POST" });
        const data = await resp.json();
        if (data.ok && data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
      setTimeout(() => {
        if (btn) btn.classList.remove("active");
      }, 350);
    }

    fullZoomIn() {
      if (!this.map) return;
      const ll = this.robotMarker
        ? this.robotMarker.getLatLng()
        : { lat: 25.185685042934363, lng: 51.57190161939388 };
      this.map.flyTo([ll.lat, ll.lng], 21, { duration: 0.5 });
    }

    fullZoomOut() {
      if (!this.map) return;
      const ll = this.robotMarker
        ? this.robotMarker.getLatLng()
        : { lat: 25.185685042934363, lng: 51.57190161939388 };
      const targetZoom = this.map.getZoom() <= 10 ? 3 : 10;
      this.map.flyTo([ll.lat, ll.lng], targetZoom, { duration: 0.5 });
    }

    toggleFullscreen() {
      const card = document.getElementById("offlineMapCard");
      const btn = document.getElementById("btnFullScreenMap");
      if (!card) return;
      const isFull = card.classList.toggle("map-fullscreen");
      if (btn) {
        btn.textContent = isFull ? "✕ Exit Full Screen" : "⛶ Full Screen Map";
      }
      setTimeout(() => {
        if (this.map) this.map.invalidateSize();
      }, 150);
    }

    async resetGyro() {
      this.lastPointCount = 0;
      const btn = document.getElementById("btnResetGyro");
      if (btn) btn.classList.add("active");
      try {
        await fetch("/api/navigation/calibrate_imu", { method: "POST" });
        const resp = await fetch("/api/navigation/reset", { method: "POST" });
        const data = await resp.json();
        if (data.navigation) {
          this.updateFromTelemetry(data.navigation);
        }
      } catch (_) {}
      setTimeout(() => {
        if (btn) btn.classList.remove("active");
      }, 350);
    }

    locateMe() {
      this.followRobot = true;
      const btn = document.getElementById("btnLocateMe");
      if (btn) btn.classList.add("active");

      const finishFly = (lat, lon) => {
        if (this.map) {
          this.map.flyTo([lat, lon], Math.max(this.map.getZoom(), 18), { duration: 0.6 });
        }
        setTimeout(() => {
          if (btn) btn.classList.remove("active");
        }, 400);
      };

      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
          async (pos) => {
            const lat = pos.coords.latitude;
            const lon = pos.coords.longitude;
            await this.pushMobileGpsToBackend(
              lat,
              lon,
              pos.coords.accuracy,
              pos.coords.heading,
              "manual"
            );
            finishFly(lat, lon);
          },
          () => {
            const ll = this.robotMarker
              ? this.robotMarker.getLatLng()
              : { lat: 25.185685042934363, lng: 51.57190161939388 };
            finishFly(ll.lat, ll.lng);
          },
          { enableHighAccuracy: true, maximumAge: 0, timeout: 4000 }
        );
      } else {
        const ll = this.robotMarker
          ? this.robotMarker.getLatLng()
          : { lat: 25.185685042934363, lng: 51.57190161939388 };
        finishFly(ll.lat, ll.lng);
      }
    }
  }

  window.LiftMateOfflineMap = LiftMateOfflineMap;
})();
