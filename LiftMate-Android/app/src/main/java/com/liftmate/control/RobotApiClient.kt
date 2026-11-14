package com.liftmate.control

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import org.json.JSONArray
import org.json.JSONObject
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference

/**
 * Binds process network sockets to the active Wi-Fi interface when Android connects
 * to a no-internet robot AP such as "LiftMate" (10.42.0.1), preventing cellular routing.
 */
class WifiNetworkBinder(context: Context) {
    private val connectivityManager =
        context.applicationContext.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    private var registered = false

    private val callback = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) {
            try {
                connectivityManager.bindProcessToNetwork(network)
            } catch (_: Exception) {
            }
        }

        override fun onLost(network: Network) {
            try {
                val activeWifi = findActiveWifiNetwork()
                connectivityManager.bindProcessToNetwork(activeWifi)
            } catch (_: Exception) {
            }
        }
    }

    private fun findActiveWifiNetwork(): Network? {
        return try {
            val active = connectivityManager.activeNetwork ?: return null
            val caps = connectivityManager.getNetworkCapabilities(active)
            if (caps != null && caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) active else null
        } catch (_: Exception) {
            null
        }
    }

    fun start() {
        if (registered) return
        try {
            val req = NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .build()
            connectivityManager.registerNetworkCallback(req, callback)
            registered = true
        } catch (_: Exception) {
        }
    }

    fun stop() {
        if (!registered) return
        try {
            connectivityManager.unregisterNetworkCallback(callback)
        } catch (_: Exception) {
        } finally {
            registered = false
            try {
                connectivityManager.bindProcessToNetwork(null)
            } catch (_: Exception) {
            }
        }
    }
}

/**
 * Non-blocking HTTP REST + Camera client with automatic LiftMate robot discovery.
 */
class RobotApiClient(private val context: Context) {
    private val prefs = context.getSharedPreferences("liftmate_prefs", Context.MODE_PRIVATE)
    private val activeBaseUrlRef = AtomicReference<String?>(null)
    private val probeExecutor = Executors.newCachedThreadPool()

    var customHost: String
        get() = prefs.getString("custom_host", "") ?: ""
        set(value) {
            prefs.edit().putString("custom_host", value.trim()).apply()
        }

    var lastKnownUrl: String
        get() = prefs.getString("last_known_url", "") ?: ""
        private set(value) {
            prefs.edit().putString("last_known_url", value).apply()
        }

    val activeBaseUrl: String?
        get() = activeBaseUrlRef.get()

    fun setPreferredUrl(url: String) {
        HostCandidatePlanner.normalizeBaseUrl(url)?.let {
            activeBaseUrlRef.set(it)
            lastKnownUrl = it
        }
    }

    /**
     * Automatically probes all known LiftMate addresses and locks onto the first live server.
     */
    fun discoverAndPollStatus(): RobotTelemetry? {
        val current = activeBaseUrlRef.get()
        if (current != null) {
            val direct = fetchAiStatus(current, timeoutMs = 850)
            if (direct != null) return direct
            activeBaseUrlRef.set(null)
        }

        val candidates = HostCandidatePlanner.orderedCandidates(customHost, lastKnownUrl)
        val futures = candidates.map { candidate ->
            probeExecutor.submit<Pair<String, RobotTelemetry?>?> {
                candidate to fetchAiStatus(candidate, timeoutMs = 750)
            }
        }
        for (future in futures) {
            val res = try {
                future.get()
            } catch (_: Exception) {
                null
            }
            if (res?.second != null) {
                activeBaseUrlRef.set(res.first)
                lastKnownUrl = res.first
                return res.second
            }
        }
        return null
    }

    fun fetchAiStatus(baseUrl: String? = activeBaseUrlRef.get(), timeoutMs: Int = 900): RobotTelemetry? {
        val target = baseUrl ?: return null
        val t0 = System.currentTimeMillis()
        val body = httpGetText("$target/api/ai/status", timeoutMs) ?: return null
        val latency = System.currentTimeMillis() - t0
        return parseAiStatusPayload(body, target, latency)
    }

    fun enrichWithFullState(current: RobotTelemetry): RobotTelemetry {
        val target = current.activeBaseUrl.ifEmpty { activeBaseUrlRef.get() ?: return current }
        val body = httpGetText("$target/api/state", 1100) ?: return current
        return try {
            val root = JSONObject(body)
            val safety = root.optJSONObject("safety")
            val serial = root.optJSONObject("serial")
            val cam = root.optJSONObject("camera")
            val imu = root.optJSONObject("imu")
            val motors = root.optJSONObject("motors")?.optJSONArray("motors")
            val pwms = ArrayList<Int>(4)
            if (motors != null) {
                for (i in 0 until minOf(4, motors.length())) {
                    pwms.add(motors.optJSONObject(i)?.optInt("commanded_pwm", 0) ?: 0)
                }
            }
            while (pwms.size < 4) pwms.add(0)

            current.copy(
                serialConnected = serial?.optBoolean("connected", current.serialConnected) ?: current.serialConnected,
                serialPort = serial?.optString("port_name", current.serialPort) ?: current.serialPort,
                safetyState = safety?.optString("state", current.safetyState) ?: current.safetyState,
                estopLatched = safety?.optBoolean("estop_latched", current.estopLatched) ?: current.estopLatched,
                cameraStreaming = cam?.optBoolean("streaming", current.cameraStreaming) ?: current.cameraStreaming,
                cameraFps = cam?.optDouble("measured_fps", current.cameraFps) ?: current.cameraFps,
                headingDeg = imu?.optDouble("heading_deg", current.headingDeg) ?: current.headingDeg,
                pitchDeg = imu?.optDouble("pitch_deg", current.pitchDeg) ?: current.pitchDeg,
                rollDeg = imu?.optDouble("roll_deg", current.rollDeg) ?: current.rollDeg,
                wheelPwms = pwms,
            )
        } catch (_: Exception) {
            current
        }
    }

    fun sendSimpleControl(jsonBody: String): JSONObject? =
        postJson("/api/simple/control", jsonBody, timeoutMs = 800)

    fun setAutonomousMode(enabled: Boolean, speedPwm: Int): JSONObject? =
        postJson(
            "/api/autonomous/mode",
            ControlPayloadFormatter.buildAutonomousJson(enabled, speedPwm),
            timeoutMs = 1100,
        )

    fun setAiConfig(enabled: Boolean? = null, mode: String? = null): JSONObject? {
        val obj = JSONObject()
        if (enabled != null) obj.put("enabled", enabled)
        if (mode != null) obj.put("mode", mode)
        return postJson("/api/ai/config", obj.toString(), timeoutMs = 950)
    }

    fun startCamera(source: String = "remote_pi"): JSONObject? =
        postJson("/api/camera/start", """{"source":"$source","width":320,"height":240}""", timeoutMs = 1500)

    fun postEmpty(path: String): JSONObject? =
        postJson(path, "{}", timeoutMs = 950)

    fun shutdownSystem(cutUsbPower: Boolean = true): JSONObject? =
        postJson(
            "/api/system/shutdown",
            ControlPayloadFormatter.buildSystemShutdownJson(cutUsbPower = cutUsbPower),
            timeoutMs = 2500,
        )

    fun fetchCameraFrame(): Bitmap? {
        val base = activeBaseUrlRef.get() ?: return null
        var conn: HttpURLConnection? = null
        return try {
            val url = URL("$base/api/camera/frame.jpg?t=${System.currentTimeMillis()}")
            conn = (url.openConnection() as HttpURLConnection).apply {
                connectTimeout = 900
                readTimeout = 1300
                useCaches = false
                requestMethod = "GET"
            }
            if (conn.responseCode != 200) return null
            conn.inputStream.use { stream ->
                BitmapFactory.decodeStream(stream)
            }
        } catch (_: Exception) {
            null
        } finally {
            conn?.disconnect()
        }
    }

    private fun postJson(path: String, jsonBody: String, timeoutMs: Int): JSONObject? {
        val base = activeBaseUrlRef.get() ?: return null
        var conn: HttpURLConnection? = null
        return try {
            val url = URL("$base$path")
            conn = (url.openConnection() as HttpURLConnection).apply {
                requestMethod = "POST"
                connectTimeout = timeoutMs
                readTimeout = timeoutMs
                doOutput = true
                useCaches = false
                setRequestProperty("Content-Type", "application/json")
            }
            OutputStreamWriter(conn.outputStream, Charsets.UTF_8).use { writer ->
                writer.write(jsonBody)
                writer.flush()
            }
            val code = conn.responseCode
            val stream = if (code in 200..299) conn.inputStream else conn.errorStream
            val text = stream?.let { BufferedReader(InputStreamReader(it, Charsets.UTF_8)).readText() } ?: return null
            JSONObject(text)
        } catch (_: Exception) {
            null
        } finally {
            conn?.disconnect()
        }
    }

    private fun httpGetText(fullUrl: String, timeoutMs: Int): String? {
        var conn: HttpURLConnection? = null
        return try {
            val url = URL(fullUrl)
            conn = (url.openConnection() as HttpURLConnection).apply {
                requestMethod = "GET"
                connectTimeout = timeoutMs
                readTimeout = timeoutMs
                useCaches = false
            }
            if (conn.responseCode != 200) return null
            BufferedReader(InputStreamReader(conn.inputStream, Charsets.UTF_8)).use { it.readText() }
        } catch (_: Exception) {
            null
        } finally {
            conn?.disconnect()
        }
    }

    private fun parseAiStatusPayload(body: String, baseUrl: String, latencyMs: Long): RobotTelemetry? {
        return try {
            val root = JSONObject(body)
            val aiJson = root.optJSONObject("ai_vision")
            val autoJson = root.optJSONObject("autonomous")

            val regionsMap = LinkedHashMap<String, RegionState>()
            val regionsJson = aiJson?.optJSONObject("regions")
            if (regionsJson != null) {
                for (zone in listOf("LEFT", "CENTER", "RIGHT")) {
                    val rz = regionsJson.optJSONObject(zone)
                    regionsMap[zone] = RegionState(
                        name = zone,
                        status = rz?.optString("status", "UNKNOWN") ?: "UNKNOWN",
                        risk = rz?.optString("risk", "UNKNOWN") ?: "UNKNOWN",
                        blocked = rz?.optBoolean("blocked", false) ?: false,
                    )
                }
            }

            val warnings = ArrayList<String>()
            val warnArr: JSONArray? = aiJson?.optJSONArray("warnings")
            if (warnArr != null) {
                for (i in 0 until warnArr.length()) {
                    warnings.add(warnArr.optString(i))
                }
            }

            val boxes = ArrayList<DetectionBox>()
            val detArr = aiJson?.optJSONArray("detections")
            if (detArr != null) {
                for (i in 0 until detArr.length()) {
                    val d = detArr.optJSONObject(i) ?: continue
                    val bboxArr = d.optJSONArray("bbox_norm") ?: d.optJSONArray("bbox") ?: continue
                    if (bboxArr.length() < 4) continue
                    val norm = listOf(
                        bboxArr.optDouble(0, 0.0).toFloat(),
                        bboxArr.optDouble(1, 0.0).toFloat(),
                        bboxArr.optDouble(2, 0.0).toFloat(),
                        bboxArr.optDouble(3, 0.0).toFloat(),
                    )
                    val rawLabel = d.optString("label", "")
                        .ifBlank { d.optString("category", "") }
                        .ifBlank { d.optString("raw_class", "object") }
                    val rawConf = d.optDouble("confidence", 0.0)
                    val confNorm = if (rawConf > 1.0) rawConf / 100.0 else rawConf
                    val labelLower = rawLabel.lowercase()
                    val isVehicle = d.optBoolean(
                        "is_car",
                        labelLower in setOf("car", "truck", "bus", "vehicle"),
                    )
                    val isWallBox = labelLower in setOf("wall", "barrier")
                    boxes.add(
                        DetectionBox(
                            label = rawLabel,
                            confidence = confNorm,
                            bboxNorm = norm,
                            region = d.optString("region", "CENTER"),
                            isCar = isVehicle,
                            isWall = isWallBox,
                        ),
                    )
                }
            }

            val corridorScoresList = ArrayList<Float>()
            val corridorArr = aiJson?.optJSONArray("corridor_scores")
            if (corridorArr != null) {
                for (i in 0 until corridorArr.length()) {
                    corridorScoresList.add(corridorArr.optDouble(i, 1.0).toFloat().coerceIn(0f, 1f))
                }
            }

            val counts = aiJson?.optJSONObject("counts")
            val aiSnap = AiVisionSnapshot(
                enabled = aiJson?.optBoolean("enabled", false) ?: false,
                mode = aiJson?.optString("mode", "DETECTION") ?: "DETECTION",
                modelReady = aiJson?.optBoolean("model_ready", false) ?: false,
                visionStatus = aiJson?.optString("vision_status", "UNKNOWN") ?: "UNKNOWN",
                regions = regionsMap,
                warnings = warnings,
                detections = boxes,
                personCount = counts?.optInt("person", 0) ?: aiJson?.optInt("person_count", 0) ?: 0,
                carCount = counts?.optInt("car", 0) ?: aiJson?.optInt("car_count", 0) ?: 0,
                corridorScores = corridorScoresList,
                bestHeading = aiJson?.optDouble("best_heading", 0.0)?.toFloat()?.coerceIn(-1f, 1f) ?: 0f,
                openSpaceRatio = aiJson?.optDouble("open_space_ratio", 1.0)?.toFloat()?.coerceIn(0f, 1f) ?: 1f,
                wallDetected = aiJson?.optBoolean("wall_detected", false) ?: false,
                recommendedPath = aiJson?.optString("recommended_path", "FORWARD") ?: "FORWARD",
            )

            val autoSnap = AutonomousSnapshot(
                enabled = autoJson?.optBoolean("enabled", false) ?: false,
                mode = autoJson?.optString("mode", "MANUAL") ?: "MANUAL",
                action = autoJson?.optString("action", "MANUAL") ?: "MANUAL",
                reason = autoJson?.optString("reason", "Manual operator control active")
                    ?: "Manual operator control active",
                speedPwm = autoJson?.optInt("speed_pwm", 90) ?: 90,
            )

            RobotTelemetry(
                connectedToRobot = true,
                activeBaseUrl = baseUrl,
                serialConnected = root.optBoolean("serial_connected", false),
                serialPort = root.optString("serial_port", "DISCONNECTED"),
                safetyState = root.optString("safety_state", "DISCONNECTED"),
                aiVision = aiSnap,
                autonomous = autoSnap,
                latencyMs = latencyMs,
            )
        } catch (_: Exception) {
            null
        }
    }
}
