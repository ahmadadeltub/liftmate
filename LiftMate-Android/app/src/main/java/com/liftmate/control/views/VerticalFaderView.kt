package com.liftmate.control.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Shader
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View

/**
 * Vertical robotics fader slider.
 * - Spring mode (isSticky = false): value in -1..+1 (0 at center, springs back on release)
 * - Sticky mode (isSticky = true): value in 0..1 (stays at target angle, e.g. camera tilt)
 */
class VerticalFaderView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    var isSticky: Boolean = false
        set(value) {
            field = value
            if (!value) normalizedValue = 0f else if (normalizedValue < 0f) normalizedValue = 0f
            invalidate()
        }

    var accentStartHex: String = "#00F0FF"
    var accentEndHex: String = "#6366F1"

    /** Normalized value: -1..+1 for spring actuators, 0..1 for sticky servo. */
    var normalizedValue: Float = 0f
        private set

    var onValueChanged: ((value: Float, active: Boolean) -> Unit)? = null

    private var activeTouch = false
    private var lightTheme = false
    private val trackRect = RectF()

    fun setLightTheme(isLight: Boolean) {
        lightTheme = isLight
        trackPaint.color = if (isLight) Color.parseColor("#E2E8F0") else Color.parseColor("#0B1120")
        trackStroke.color = if (isLight) Color.parseColor("#CBD5E1") else Color.parseColor("#3394A3B8")
        centerTickPaint.color = if (isLight) Color.parseColor("#94A3B8") else Color.parseColor("#475569")
        invalidate()
    }
    private val fillRect = RectF()
    private val thumbRect = RectF()

    private val trackPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#0B1120")
        style = Paint.Style.FILL
    }
    private val trackStroke = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#3394A3B8")
        style = Paint.Style.STROKE
        strokeWidth = 2.5f
    }
    private val centerTickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#475569")
        strokeWidth = 2f
    }
    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val thumbPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val thumbStroke = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }
    private val thumbBarPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        strokeWidth = 4f
        strokeCap = Paint.Cap.ROUND
    }

    fun setStickyFraction(fraction: Float) {
        if (!isSticky) return
        normalizedValue = fraction.coerceIn(0f, 1f)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 10f || h <= 20f) return

        val trackW = minOf(w * 0.72f, 84f)
        val left = (w - trackW) / 2f
        val right = left + trackW
        val topPad = 14f
        val bottomPad = h - 14f
        trackRect.set(left, topPad, right, bottomPad)

        val cornerR = trackW / 2f
        canvas.drawRoundRect(trackRect, cornerR, cornerR, trackPaint)
        canvas.drawRoundRect(trackRect, cornerR, cornerR, trackStroke)

        val thumbH = 38f
        val usableTop = topPad + thumbH / 2f + 4f
        val usableBot = bottomPad - thumbH / 2f - 4f
        val centerY = (usableTop + usableBot) / 2f

        canvas.drawLine(left + 12f, centerY, right - 12f, centerY, centerTickPaint)

        val fracFromBottom = if (isSticky) normalizedValue.coerceIn(0f, 1f) else ((normalizedValue + 1f) / 2f).coerceIn(0f, 1f)
        val thumbCenterY = usableBot - fracFromBottom * (usableBot - usableTop)

        fillPaint.shader = LinearGradient(
            left,
            topPad,
            right,
            bottomPad,
            Color.parseColor(accentStartHex),
            Color.parseColor(accentEndHex),
            Shader.TileMode.CLAMP,
        )

        if (isSticky) {
            fillRect.set(left + 3f, thumbCenterY, right - 3f, bottomPad - 3f)
            canvas.drawRoundRect(fillRect, cornerR, cornerR, fillPaint)
        } else if (kotlin.math.abs(normalizedValue) > 0.05f) {
            val fy1 = minOf(centerY, thumbCenterY)
            val fy2 = maxOf(centerY, thumbCenterY)
            fillRect.set(left + 4f, fy1, right - 4f, fy2)
            canvas.drawRoundRect(fillRect, cornerR * 0.7f, cornerR * 0.7f, fillPaint)
        }

        val thumbPad = 4f
        thumbRect.set(left - thumbPad, thumbCenterY - thumbH / 2f, right + thumbPad, thumbCenterY + thumbH / 2f)
        thumbPaint.shader = LinearGradient(
            thumbRect.left,
            thumbRect.top,
            thumbRect.right,
            thumbRect.bottom,
            Color.parseColor(accentStartHex),
            Color.parseColor(accentEndHex),
            Shader.TileMode.CLAMP,
        )
        canvas.drawRoundRect(thumbRect, thumbH / 2f, thumbH / 2f, thumbPaint)
        canvas.drawRoundRect(thumbRect, thumbH / 2f, thumbH / 2f, thumbStroke)

        val notchW = thumbRect.width() * 0.30f
        canvas.drawLine(
            thumbRect.centerX() - notchW / 2f,
            thumbCenterY,
            thumbRect.centerX() + notchW / 2f,
            thumbCenterY,
            thumbBarPaint,
        )
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val h = height.toFloat()
        val thumbH = 38f
        val usableTop = 14f + thumbH / 2f + 4f
        val usableBot = (h - 14f) - thumbH / 2f - 4f
        val span = (usableBot - usableTop).coerceAtLeast(10f)

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                activeTouch = true
                val y = event.y.coerceIn(usableTop, usableBot)
                val fracFromBottom = (usableBot - y) / span
                normalizedValue = if (isSticky) {
                    fracFromBottom.coerceIn(0f, 1f)
                } else {
                    val v = (fracFromBottom * 2f - 1f).coerceIn(-1f, 1f)
                    if (kotlin.math.abs(v) < 0.08f) 0f else v
                }
                invalidate()
                onValueChanged?.invoke(normalizedValue, true)
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                parent?.requestDisallowInterceptTouchEvent(false)
                activeTouch = false
                if (!isSticky) {
                    normalizedValue = 0f
                    invalidate()
                }
                onValueChanged?.invoke(normalizedValue, false)
                return true
            }
        }
        return super.onTouchEvent(event)
    }
}
