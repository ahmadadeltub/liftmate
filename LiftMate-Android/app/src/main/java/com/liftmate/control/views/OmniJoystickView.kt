package com.liftmate.control.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.Shader
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * 360-degree omnidirectional Mecanum joystick pad.
 * Up = +vy (forward), Right = +vx (strafe right).
 */
class OmniJoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    var onVectorChanged: ((vy: Float, vx: Float, dragging: Boolean) -> Unit)? = null

    private var knobX = 0f
    private var knobY = 0f
    private var isDragging = false
    private var lightTheme = false

    fun setLightTheme(isLight: Boolean) {
        lightTheme = isLight
        ringPaint.color = if (isLight) Color.parseColor("#770284C7") else Color.parseColor("#5500F0FF")
        innerRingPaint.color = if (isLight) Color.parseColor("#4464748B") else Color.parseColor("#2A94A3B8")
        crosshairPaint.color = if (isLight) Color.parseColor("#33475569") else Color.parseColor("#2694A3B8")
        beamPaint.color = if (isLight) Color.parseColor("#990284C7") else Color.parseColor("#9900F0FF")
        labelPaint.color = if (isLight) Color.parseColor("#475569") else Color.parseColor("#8FA3BF")
        invalidate()
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        val h = MeasureSpec.getSize(heightMeasureSpec)
        val size = if (MeasureSpec.getMode(heightMeasureSpec) == MeasureSpec.UNSPECIFIED) {
            w
        } else {
            minOf(w, h)
        }
        setMeasuredDimension(size, size)
    }

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = Color.parseColor("#5500F0FF")
    }
    private val innerRingPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.6f
        color = Color.parseColor("#2A94A3B8")
    }
    private val crosshairPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.5f
        color = Color.parseColor("#2694A3B8")
    }
    private val beamPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 6f
        strokeCap = Paint.Cap.ROUND
        color = Color.parseColor("#9900F0FF")
    }
    private val knobPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.parseColor("#00F0FF")
    }
    private val knobStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 4f
        color = Color.WHITE
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#8FA3BF")
        textAlign = Paint.Align.CENTER
        textSize = 26f
        isFakeBoldText = true
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val radius = (minOf(width, height) / 2f) - 18f
        if (radius <= 10f) return

        val colors = if (lightTheme) {
            intArrayOf(
                Color.parseColor("#FFFFFF"),
                Color.parseColor("#E2E8F0"),
                Color.parseColor("#CBD5E1"),
            )
        } else {
            intArrayOf(
                Color.parseColor("#182542"),
                Color.parseColor("#0F172A"),
                Color.parseColor("#090F1E"),
            )
        }
        bgPaint.shader = RadialGradient(
            cx,
            cy,
            radius,
            colors,
            floatArrayOf(0f, 0.72f, 1f),
            Shader.TileMode.CLAMP,
        )
        canvas.drawCircle(cx, cy, radius, bgPaint)
        canvas.drawCircle(cx, cy, radius, ringPaint)
        canvas.drawCircle(cx, cy, radius * 0.62f, innerRingPaint)
        canvas.drawCircle(cx, cy, radius * 0.25f, innerRingPaint)

        canvas.drawLine(cx - radius * 0.85f, cy, cx + radius * 0.85f, cy, crosshairPaint)
        canvas.drawLine(cx, cy - radius * 0.85f, cx, cy + radius * 0.85f, crosshairPaint)

        // Cardinal directional hints
        val textOffset = labelPaint.textSize * 0.35f
        canvas.drawText("FWD", cx, cy - radius * 0.76f, labelPaint)
        canvas.drawText("REV", cx, cy + radius * 0.83f, labelPaint)
        canvas.drawText("L", cx - radius * 0.80f, cy + textOffset, labelPaint)
        canvas.drawText("R", cx + radius * 0.80f, cy + textOffset, labelPaint)

        val maxTravel = radius * 0.74f
        val kx = cx + knobX * maxTravel
        val ky = cy - knobY * maxTravel

        if (hypot(knobX.toDouble(), knobY.toDouble()) > 0.04) {
            canvas.drawLine(cx, cy, kx, ky, beamPaint)
        }

        val knobRadius = (radius * 0.23f).coerceIn(24f, 44f)
        knobPaint.color = if (isDragging) Color.parseColor("#00F0FF") else Color.parseColor("#38BDF8")
        canvas.drawCircle(kx, ky, knobRadius, knobPaint)
        canvas.drawCircle(kx, ky, knobRadius, knobStrokePaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val cx = width / 2f
        val cy = height / 2f
        val radius = (minOf(width, height) / 2f) - 18f
        val maxTravel = (radius * 0.74f).coerceAtLeast(10f)

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                isDragging = true
                val dx = event.x - cx
                val dy = -(event.y - cy) // up is positive vy
                val dist = hypot(dx.toDouble(), dy.toDouble()).toFloat()
                val clampedDist = minOf(dist, maxTravel)
                val normMag = clampedDist / maxTravel
                val angle = atan2(dy.toDouble(), dx.toDouble())

                // Apply tiny 6% deadzone for clean neutral stability
                if (normMag < 0.06f) {
                    knobX = 0f
                    knobY = 0f
                } else {
                    knobX = (normMag * cos(angle)).toFloat()
                    knobY = (normMag * sin(angle)).toFloat()
                }
                invalidate()
                onVectorChanged?.invoke(knobY, knobX, true)
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                parent?.requestDisallowInterceptTouchEvent(false)
                isDragging = false
                knobX = 0f
                knobY = 0f
                invalidate()
                onVectorChanged?.invoke(0f, 0f, false)
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    fun resetCenter() {
        isDragging = false
        knobX = 0f
        knobY = 0f
        invalidate()
    }
}
