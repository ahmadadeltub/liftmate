package com.liftmate.control.views

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
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
    private val corridorBarPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val pathCurvePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 5f
        strokeCap = Paint.Cap.ROUND
        color = Color.parseColor("#00F0FF")
    }
    private val pathNodePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.parseColor("#00F0FF")
    }
    private val boxFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val guidancePath = Path()

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

            // Free-space 9-bin floor corridor heatmap + glowing cyan steering path curve
            val corridor = aiSnapshot.corridorScores
            if (corridor.size >= 3) {
                val nBins = corridor.size
                val binW = dw / nBins.toFloat()
                val barTop = dy + dh * 0.78f
                val barBottom = barTop + 11f
                for (i in 0 until nBins) {
                    val s = corridor[i].coerceIn(0f, 1f)
                    corridorBarPaint.color = when {
                        s >= 0.68f -> Color.argb(175, 16, 185, 129)
                        s >= 0.40f -> Color.argb(190, 245, 158, 11)
                        else -> Color.argb(205, 239, 68, 68)
                    }
                    val leftB = dx + i * binW + 3f
                    val rightB = dx + (i + 1) * binW - 3f
                    canvas.drawRoundRect(leftB, barTop, rightB, barBottom, 4f, 4f, corridorBarPaint)
                }

                val hClamped = aiSnapshot.bestHeading.coerceIn(-1f, 1f)
                val startX = dx + dw * 0.5f
                val startY = dy + dh * 0.88f
                val ctrlY = dy + dh * 0.72f
                val endX = dx + (0.5f + hClamped * 0.42f) * dw
                val endY = dy + dh * 0.56f
                guidancePath.reset()
                guidancePath.moveTo(startX, startY)
                guidancePath.quadTo(startX, ctrlY, endX, endY)
                canvas.drawPath(guidancePath, pathCurvePaint)
                canvas.drawCircle(endX, endY, 8f, pathNodePaint)

                val openPct = (aiSnapshot.openSpaceRatio * 100f).toInt().coerceIn(0, 100)
                val pathBadge = "OPEN $openPct% · ${aiSnapshot.recommendedPath}"
                val pW = zoneTextPaint.measureText(pathBadge) + 20f
                val badgeLeft = (dx + (dw - pW) * 0.5f).coerceIn(dx + 8f, (dx + dw - pW - 8f).coerceAtLeast(dx + 8f))
                val badgeBottom = (barTop - 8f).coerceAtLeast(dy + 68f)
                val badgeTop = badgeBottom - 30f
                tagBgPaint.color = Color.argb(195, 6, 11, 23)
                canvas.drawRoundRect(badgeLeft, badgeTop, badgeLeft + pW, badgeBottom, 10f, 10f, tagBgPaint)
                zoneTextPaint.textAlign = Paint.Align.LEFT
                canvas.drawText(pathBadge, badgeLeft + 10f, badgeBottom - 8f, zoneTextPaint)
                zoneTextPaint.textAlign = Paint.Align.CENTER
            }

            // Corner-reticle bounding boxes (including WALL / BARRIER structural obstacles)
            for (det in aiSnapshot.detections) {
                if (det.bboxNorm.size < 4) continue
                val x1 = dx + det.bboxNorm[0].coerceIn(0f, 1f) * dw
                val y1 = dy + det.bboxNorm[1].coerceIn(0f, 1f) * dh
                val x2 = dx + det.bboxNorm[2].coerceIn(0f, 1f) * dw
                val y2 = dy + det.bboxNorm[3].coerceIn(0f, 1f) * dh
                val col = when {
                    det.isWall -> Color.parseColor("#F97316")
                    det.label.equals("person", ignoreCase = true) -> Color.parseColor("#EF4444")
                    det.isCar -> Color.parseColor("#F59E0B")
                    else -> Color.parseColor("#00F0FF")
                }
                boxFillPaint.color = Color.argb(if (det.isWall) 36 else 22, Color.red(col), Color.green(col), Color.blue(col))
                boxRect.set(x1, y1, x2, y2)
                canvas.drawRoundRect(boxRect, 8f, 8f, boxFillPaint)

                boxPaint.color = col
                val cornerLen = minOf(x2 - x1, y2 - y1).times(0.26f).coerceIn(12f, 28f)
                // Top-left bracket
                canvas.drawLine(x1, y1 + cornerLen, x1, y1, boxPaint)
                canvas.drawLine(x1, y1, x1 + cornerLen, y1, boxPaint)
                // Top-right bracket
                canvas.drawLine(x2 - cornerLen, y1, x2, y1, boxPaint)
                canvas.drawLine(x2, y1, x2, y1 + cornerLen, boxPaint)
                // Bottom-left bracket
                canvas.drawLine(x1, y2 - cornerLen, x1, y2, boxPaint)
                canvas.drawLine(x1, y2, x1 + cornerLen, y2, boxPaint)
                // Bottom-right bracket
                canvas.drawLine(x2 - cornerLen, y2, x2, y2, boxPaint)
                canvas.drawLine(x2, y2, x2, y2 - cornerLen, boxPaint)

                val pct = (det.confidence * 100.0).toInt()
                val label = "${det.label} $pct%"
                val textW = tagTextPaint.measureText(label) + 14f
                val tagTop = (y1 - 30f).coerceAtLeast(dy + 34f)
                tagBgPaint.color = Color.argb(215, 10, 17, 32)
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
