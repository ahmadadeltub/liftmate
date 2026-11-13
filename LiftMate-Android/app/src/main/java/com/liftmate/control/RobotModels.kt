package com.liftmate.control

/**
 * Pure Kotlin models and host discovery helpers for LiftMate robot communication.
 */
data class RegionState(
    val name: String,
    val status: String = "UNKNOWN",
    val risk: String = "UNKNOWN",
    val blocked: Boolean = false,
)

data class DetectionBox(
    val label: String,
    val confidence: Double,
    val bboxNorm: List<Float>, // [x1, y1, x2, y2] in 0..1
    val region: String,
    val isCar: Boolean = false,
)

data class AiVisionSnapshot(
    val enabled: Boolean = false,
    val mode: String = "DETECTION",
    val modelReady: Boolean = false,
    val visionStatus: String = "UNKNOWN",
    val regions: Map<String, RegionState> = emptyMap(),
    val warnings: List<String> = emptyList(),
    val detections: List<DetectionBox> = emptyList(),
    val personCount: Int = 0,
    val carCount: Int = 0,
)

data class AutonomousSnapshot(
    val enabled: Boolean = false,
    val mode: String = "MANUAL",
    val action: String = "MANUAL",
    val reason: String = "Manual operator control active",
    val speedPwm: Int = 90,
)

data class RobotTelemetry(
    val connectedToRobot: Boolean = false,
    val activeBaseUrl: String = "",
    val serialConnected: Boolean = false,
    val serialPort: String = "DISCONNECTED",
    val safetyState: String = "DISCONNECTED",
    val estopLatched: Boolean = false,
    val cameraStreaming: Boolean = false,
    val cameraFps: Double = 0.0,
    val headingDeg: Double = 0.0,
    val pitchDeg: Double = 0.0,
    val rollDeg: Double = 0.0,
    val wheelPwms: List<Int> = listOf(0, 0, 0, 0),
    val aiVision: AiVisionSnapshot = AiVisionSnapshot(),
    val autonomous: AutonomousSnapshot = AutonomousSnapshot(),
    val latencyMs: Long = -1L,
)

object HostCandidatePlanner {
    private val DEFAULT_CANDIDATES = listOf(
        "http://10.42.0.1",
        "http://10.42.0.1:8000",
        "http://192.168.18.36:8000",
        "http://192.168.18.36",
    )

    fun normalizeBaseUrl(raw: String?): String? {
        val trimmed = raw?.trim().orEmpty()
        if (trimmed.isEmpty()) return null
        val withScheme = if (trimmed.startsWith("http://") || trimmed.startsWith("https://")) {
            trimmed
        } else {
            "http://$trimmed"
        }
        return withScheme.trimEnd('/')
    }

    fun orderedCandidates(customHost: String? = null, lastKnownUrl: String? = null): List<String> {
        val seen = LinkedHashSet<String>()
        normalizeBaseUrl(lastKnownUrl)?.let { seen.add(it) }
        normalizeBaseUrl(customHost)?.let { seen.add(it) }
        for (item in DEFAULT_CANDIDATES) {
            seen.add(item)
        }
        return seen.toList()
    }
}

object ControlPayloadFormatter {
    fun clampPwm(value: Int, minVal: Int = -255, maxVal: Int = 255): Int =
        value.coerceIn(minVal, maxVal)

    fun clampServoDeg(angle: Int): Int = angle.coerceIn(6, 130)

    fun clampAutonomousSpeed(speedPwm: Int): Int = speedPwm.coerceIn(55, 125)

    fun buildWheelJson(vy: Int, vx: Int, omega: Int): String =
        """{"mode":"WHEELS","vy":${clampPwm(vy)},"vx":${clampPwm(vx)},"omega":${clampPwm(omega)}}"""

    fun buildActuatorJson(upperPwm: Int, lowerPwm: Int): String =
        """{"mode":"ACTUATORS","upper_pwm":${clampPwm(upperPwm)},"lower_pwm":${clampPwm(lowerPwm)}}"""

    fun buildServoJson(angleDeg: Int): String =
        """{"mode":"SERVO","servo_angle":${clampServoDeg(angleDeg)}}"""

    fun buildSimpleModeJson(mode: String): String =
        """{"mode":"$mode"}"""

    fun buildAutonomousJson(enabled: Boolean, speedPwm: Int): String =
        """{"enabled":$enabled,"speed_pwm":${clampAutonomousSpeed(speedPwm)}}"""
}
