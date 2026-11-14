package com.liftmate.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class RobotModelsTest {

    @Test
    fun hostDiscoveryPrioritizesLastKnownCustomAndDefaultAp() {
        val candidates = HostCandidatePlanner.orderedCandidates(
            customHost = "192.168.1.50:8000/",
            lastKnownUrl = "http://10.42.0.1",
        )
        assertEquals(
            listOf(
                "http://10.42.0.1",
                "http://192.168.1.50:8000",
                "http://10.42.0.1:8000",
                "http://192.168.18.36:8000",
                "http://192.168.18.36",
            ),
            candidates,
        )
        assertNull(HostCandidatePlanner.normalizeBaseUrl("   "))
    }

    @Test
    fun controlPayloadsClampSafetyLimitsCleanly() {
        assertEquals(
            """{"mode":"WHEELS","vy":255,"vx":-255,"omega":120}""",
            ControlPayloadFormatter.buildWheelJson(999, -999, 120),
        )
        assertEquals(
            """{"mode":"SERVO","servo_angle":130}""",
            ControlPayloadFormatter.buildServoJson(180),
        )
        assertEquals(
            """{"mode":"SERVO","servo_angle":6}""",
            ControlPayloadFormatter.buildServoJson(-10),
        )
        assertEquals(
            """{"enabled":true,"speed_pwm":125}""",
            ControlPayloadFormatter.buildAutonomousJson(true, 220),
        )
        assertEquals(
            """{"enabled":false,"speed_pwm":55}""",
            ControlPayloadFormatter.buildAutonomousJson(false, 10),
        )
        assertEquals(
            """{"confirm":true,"cut_usb_power":true,"reason":"Mobile operator shutdown"}""",
            ControlPayloadFormatter.buildSystemShutdownJson(),
        )
    }

    @Test
    fun themePalettesExposeDistinctDarkAndLightModes() {
        assertEquals(false, ThemePalettes.DARK.isLight)
        assertEquals(true, ThemePalettes.LIGHT.isLight)
    }

    @Test
    fun aiVisionSnapshotTracksWallsAndCorridors() {
        val snap = AiVisionSnapshot(
            enabled = true,
            corridorScores = listOf(0.9f, 0.85f, 0.2f),
            bestHeading = -0.6f,
            openSpaceRatio = 0.65f,
            wallDetected = true,
            recommendedPath = "VEER_LEFT",
            detections = listOf(DetectionBox("WALL", 0.92, listOf(0.5f, 0.2f, 0.95f, 0.9f), "RIGHT", isWall = true)),
        )
        assertEquals(true, snap.wallDetected)
        assertEquals(-0.6f, snap.bestHeading, 0.001f)
        assertEquals("VEER_LEFT", snap.recommendedPath)
        assertEquals(true, snap.detections.first().isWall)
    }
}
