package com.liftmate.control.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import com.liftmate.control.RegionState
import kotlin.math.abs

/**
 * Live top-down Mecanum robot visualization showing 4 wheel PWM bars,
 * front AI obstacle sectors (Left/Center/Right), camera tilt needle, and safety state.
 */
class RobotChassisView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    private var wheelPwms: List<Int> = listOf(0, 0, 0, 0)
    private var regions: Map<String, RegionState> = emptyMap()
    private var servoDeg: Int = 68
    private var autonomousOn: Boolean = false
    private var lightTheme: Boolean = false

    private val rect = RectF()
    private val bodyPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }
    private val wheelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val sectorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textAlign = Paint.Align.CENTER
        textSize = 20f
        isFakeBoldText = true
    }
    private val conePath = Path()

    fun setLightTheme(isLight: Boolean) {
        lightTheme = isLight
        invalidate()
    }

    fun updateState(
        pwms: List<Int>,
        regionsMap: Map<String, RegionState>,
        servoAngleDeg: Int,
        autoEnabled: Boolean,
    ) {
        wheelPwms = if (pwms.size >= 4) pwms else listOf(0, 0, 0, 0)
        regions = regionsMap
        servoDeg = servoAngleDeg
        autonomousOn = autoEnabled
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 20f || h <= 20f) return

        val cx = w / 2f
        val cy = h * 0.56f
        val bodyW = minOf(w * 0.42f, h * 0.48f)
        val bodyH = bodyW * 1.26f

        // Forward AI radar sectors (Left / Center / Right)
        val sectorR = minOf(cx - 8f, cy - 10f)
        val arcRect = RectF(cx - sectorR, cy - sectorR, cx + sectorR, cy + sectorR)
        val zones = listOf("LEFT" to 215f, "CENTER" to 250f, "RIGHT" to 285f)
        for ((zone, startAng) in zones) {
            val reg = regions[zone] ?: RegionState(zone)
            sectorPaint.color = radarColor(reg)
            canvas.drawArc(arcRect, startAng, 32f, true, sectorPaint)
        }

        // Chassis body
        bodyPaint.color = if (lightTheme) Color.parseColor("#E2E8F0") else Color.parseColor("#0F172A")
        borderPaint.color = when {
            autonomousOn -> Color.parseColor("#10B981")
            lightTheme -> Color.parseColor("#4F46E5")
            else -> Color.parseColor("#00F0FF")
        }
        rect.set(cx - bodyW / 2f, cy - bodyH / 2f, cx + bodyW / 2f, cy + bodyH / 2f)
        canvas.drawRoundRect(rect, 14f, 14f, bodyPaint)
        canvas.drawRoundRect(rect, 14f, 14f, borderPaint)

        // Inner Jack Frame
        borderPaint.strokeWidth = 2f
        borderPaint.color = if (lightTheme) Color.parseColor("#0284C7") else Color.parseColor("#38BDF8")
        rect.inset(bodyW * 0.22f, bodyH * 0.22f)
        canvas.drawRoundRect(rect, 8f, 8f, borderPaint)

        // Forward heading chevron inside body
        conePath.reset()
        conePath.moveTo(cx, cy - bodyH * 0.16f)
        conePath.lineTo(cx - bodyW * 0.16f, cy + bodyH * 0.14f)
        conePath.lineTo(cx, cy + bodyH * 0.06f)
        conePath.lineTo(cx + bodyW * 0.16f, cy + bodyH * 0.14f)
        conePath.close()
        wheelPaint.color = borderPaint.color
        canvas.drawPath(conePath, wheelPaint)

        // Four Mecanum wheels around chassis
        val ww = bodyW * 0.24f
        val wh = bodyH * 0.28f
        val wheelCoords = listOf(
            // Rear-Left / Rear-Right / Front-Left / Front-Right matching motor channels 1..4
            Triple(cx - bodyW / 2f - ww - 4f, cy + bodyH * 0.18f, wheelPwms.getOrElse(0) { 0 }),
            Triple(cx + bodyW / 2f + 4f, cy + bodyH * 0.18f, wheelPwms.getOrElse(1) { 0 }),
            Triple(cx - bodyW / 2f - ww - 4f, cy - bodyH * 0.44f, wheelPwms.getOrElse(2) { 0 }),
            Triple(cx + bodyW / 2f + 4f, cy - bodyH * 0.44f, wheelPwms.getOrElse(3) { 0 }),
        )
        for ((wx, wy, pwm) in wheelCoords) {
            wheelPaint.color = when {
                pwm > 8 -> Color.parseColor("#10B981")
                pwm < -8 -> Color.parseColor("#F59E0B")
                lightTheme -> Color.parseColor("#475569")
                else -> Color.parseColor("#334155")
            }
            rect.set(wx, wy, wx + ww, wy + wh)
            canvas.drawRoundRect(rect, 6f, 6f, wheelPaint)
            if (abs(pwm) > 8) {
                labelPaint.color = wheelPaint.color
                labelPaint.textSize = 18f
                val tag = if (pwm > 0) "+$pwm" else "$pwm"
                val tx = if (wx < cx) wx - 16f else wx + ww + 16f
                canvas.drawText(tag, tx, wy + wh * 0.65f, labelPaint)
            }
        }

        // Camera Servo label at top edge
        labelPaint.color = if (lightTheme) Color.parseColor("#0F172A") else Color.parseColor("#CBD5E1")
        labelPaint.textSize = 20f
        canvas.drawText("CAM ${servoDeg}°", cx, 24f, labelPaint)
    }

    private fun radarColor(r: RegionState): Int {
        val statusU = r.status.uppercase()
        val riskU = r.risk.uppercase()
        val base = when {
            r.blocked || statusU.contains("BLOCKED") || riskU == "HIGH" -> Color.parseColor("#EF4444")
            statusU.contains("CAUTION") || riskU == "MEDIUM" || statusU.contains("OBSTACLE") -> Color.parseColor("#F59E0B")
            statusU.contains("CLEAR") || riskU == "LOW" || riskU == "NONE" -> Color.parseColor("#10B981")
            else -> Color.parseColor("#64748B")
        }
        return Color.argb(if (lightTheme) 70 else 85, Color.red(base), Color.green(base), Color.blue(base))
    }
}
