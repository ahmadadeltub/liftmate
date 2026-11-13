package com.liftmate.control.views

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import com.liftmate.control.AiVisionSnapshot
import com.liftmate.control.AutonomousSnapshot
import com.liftmate.control.RegionState

/**
 * Renders live robot camera frames with crisp minimal AI obstacle zone indicators and boxes.
 */
class CameraOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    private var frameBitmap: Bitmap? = null
    private var aiSnapshot: AiVisionSnapshot = AiVisionSnapshot()
    private var autoSnapshot: AutonomousSnapshot = AutonomousSnapshot()
    private var fillCrop: Boolean = false
    private val destRect = RectF()

    fun setFillCrop(fill: Boolean) {
        fillCrop = fill
        invalidate()
    }
    private val boxRect = RectF()

    private val framePaint = Paint(Paint.ANTI_ALIAS_FLAG or Paint.FILTER_BITMAP_FLAG)
    private val dividerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#33FFFFFF")
        strokeWidth = 2f
        pathEffect = DashPathEffect(floatArrayOf(8f, 10f), 0f)
    }
    private val boxPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
    }
    private val tagBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val tagTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 24f
        isFakeBoldText = true
    }
    private val placeholderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#94A3B8")
        textAlign = Paint.Align.CENTER
        textSize = 30f
    }
    private val zoneBarPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val zoneTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textAlign = Paint.Align.CENTER
        textSize = 22f
        isFakeBoldText = true
    }

    fun updateFrame(bitmap: Bitmap?) {
        frameBitmap = bitmap
        invalidate()
    }

    fun updateTelemetry(ai: AiVisionSnapshot, auto: AutonomousSnapshot) {
        aiSnapshot = ai
        autoSnapshot = auto
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val vw = width.toFloat()
        val vh = height.toFloat()
        if (vw <= 0f || vh <= 0f) return

        canvas.drawColor(Color.parseColor("#060B16"))

        val bmp = frameBitmap
        if (bmp == null || bmp.isRecycled) {
            canvas.drawText("Waiting for live robot camera stream…", vw / 2f, vh / 2f, placeholderPaint)
            return
        }

        // Fit maintaining aspect ratio without stretching (or optional fill-crop)
        val bw = bmp.width.toFloat().coerceAtLeast(1f)
        val bh = bmp.height.toFloat().coerceAtLeast(1f)
        val scale = if (fillCrop) maxOf(vw / bw, vh / bh) else minOf(vw / bw, vh / bh)
        val dw = bw * scale
        val dh = bh * scale
        val dx = (vw - dw) / 2f
        val dy = (vh - dh) / 2f
        destRect.set(dx, dy, dx + dw, dy + dh)

        canvas.drawBitmap(bmp, null, destRect, framePaint)

        // Subtle 1/3 vertical obstacle sector dividers (keeps center view open)
        if (aiSnapshot.enabled || autoSnapshot.enabled) {
            val thirdW = dw / 3f
            canvas.drawLine(dx + thirdW, dy + 32f, dx + thirdW, dy + dh - 8f, dividerPaint)
            canvas.drawLine(dx + thirdW * 2f, dy + 32f, dx + thirdW * 2f, dy + dh - 8f, dividerPaint)

            // Minimal top-edge zone pills (L / C / R)
            val zones = listOf("LEFT", "CENTER", "RIGHT")
            for ((idx, key) in zones.withIndex()) {
                val region = aiSnapshot.regions[key] ?: RegionState(key)
                zoneBarPaint.color = regionColor(region)
                val segLeft = dx + idx * thirdW + 6f
                val segRight = dx + (idx + 1) * thirdW - 6f
                boxRect.set(segLeft, dy + 6f, segRight, dy + 30f)
                canvas.drawRoundRect(boxRect, 8f, 8f, zoneBarPaint)
                val shortLabel = "${key.take(1)}: ${shortRegionTag(region)}"
                canvas.drawText(shortLabel, boxRect.centerX(), boxRect.centerY() + 7f, zoneTextPaint)
            }

            // Bounding boxes
            for (det in aiSnapshot.detections) {
                if (det.bboxNorm.size < 4) continue
                val x1 = dx + det.bboxNorm[0].coerceIn(0f, 1f) * dw
                val y1 = dy + det.bboxNorm[1].coerceIn(0f, 1f) * dh
                val x2 = dx + det.bboxNorm[2].coerceIn(0f, 1f) * dw
                val y2 = dy + det.bboxNorm[3].coerceIn(0f, 1f) * dh
                val col = if (det.label.equals("person", ignoreCase = true)) {
                    Color.parseColor("#EF4444")
                } else if (det.isCar) {
                    Color.parseColor("#F59E0B")
                } else {
                    Color.parseColor("#00F0FF")
                }
                boxPaint.color = col
                boxRect.set(x1, y1, x2, y2)
                canvas.drawRoundRect(boxRect, 8f, 8f, boxPaint)

                val pct = (det.confidence * 100.0).toInt()
                val label = "${det.label} $pct%"
                val textW = tagTextPaint.measureText(label) + 14f
                val tagTop = (y1 - 30f).coerceAtLeast(dy + 34f)
                tagBgPaint.color = Color.argb(200, 15, 23, 42)
                canvas.drawRoundRect(x1, tagTop, x1 + textW, tagTop + 26f, 6f, 6f, tagBgPaint)
                canvas.drawText(label, x1 + 7f, tagTop + 19f, tagTextPaint)
            }
        }
    }

    private fun shortRegionTag(r: RegionState): String {
        val statusU = r.status.uppercase()
        val riskU = r.risk.uppercase()
        return when {
            r.blocked || statusU.contains("BLOCKED") || riskU == "HIGH" -> "BLOCK"
            statusU.contains("CAUTION") || riskU == "MEDIUM" || statusU.contains("OBSTACLE") -> "WARN"
            statusU.contains("CLEAR") || riskU == "LOW" || riskU == "NONE" -> "CLEAR"
            else -> "WAIT"
        }
    }

    private fun regionColor(r: RegionState): Int {
        val statusU = r.status.uppercase()
        val riskU = r.risk.uppercase()
        return when {
            r.blocked || statusU.contains("BLOCKED") || riskU == "HIGH" -> Color.argb(190, 239, 68, 68)
            statusU.contains("CAUTION") || riskU == "MEDIUM" || statusU.contains("OBSTACLE") -> Color.argb(185, 245, 158, 11)
            statusU.contains("CLEAR") || riskU == "LOW" || riskU == "NONE" -> Color.argb(165, 16, 185, 129)
            else -> Color.argb(150, 71, 85, 105)
        }
    }
}
