package com.liftmate.control

import android.content.res.ColorStateList
import android.graphics.Color
import android.graphics.drawable.Drawable
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.RippleDrawable

/**
 * Unified Dark and Light color palettes + dynamic vector drawables for LiftMate Android UI.
 */
data class ThemePalette(
    val isLight: Boolean,
    val bgTop: Int,
    val bgBottom: Int,
    val headerBg: Int,
    val cardBg: Int,
    val cardBorder: Int,
    val autoCardOnStart: Int,
    val autoCardOnEnd: Int,
    val autoCardBorderOn: Int,
    val pillBg: Int,
    val pillBorder: Int,
    val textPrimary: Int,
    val textSecondary: Int,
    val textMuted: Int,
    val cyan: Int,
    val indigo: Int,
    val emerald: Int,
    val amber: Int,
    val red: Int,
    val cameraBg: Int,
) {
    fun cardDrawable(cornerRadiusPx: Float = 42f, borderColor: Int = cardBorder, strokePx: Int = 2): Drawable =
        GradientDrawable().apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerRadiusPx
            setColor(cardBg)
            setStroke(strokePx, borderColor)
        }

    fun autoCardOnDrawable(cornerRadiusPx: Float = 42f): Drawable =
        GradientDrawable(
            GradientDrawable.Orientation.TL_BR,
            intArrayOf(autoCardOnStart, autoCardOnEnd),
        ).apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerRadiusPx
            setStroke(3, autoCardBorderOn)
        }

    fun screenBackground(): Drawable =
        GradientDrawable(
            GradientDrawable.Orientation.TL_BR,
            intArrayOf(bgTop, bgBottom),
        )

    fun pillDrawable(cornerRadiusPx: Float = 26f): Drawable {
        val shape = GradientDrawable().apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerRadiusPx
            setColor(pillBg)
            setStroke(2, pillBorder)
        }
        val rippleColor = if (isLight) Color.parseColor("#220284C7") else Color.parseColor("#3300F0FF")
        return RippleDrawable(ColorStateList.valueOf(rippleColor), shape, shape)
    }

    fun gradientPillDrawable(startHex: String, endHex: String, borderHex: String, cornerRadiusPx: Float = 26f): Drawable {
        val shape = GradientDrawable(
            GradientDrawable.Orientation.TL_BR,
            intArrayOf(Color.parseColor(startHex), Color.parseColor(endHex)),
        ).apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerRadiusPx
            setStroke(2, Color.parseColor(borderHex))
        }
        return RippleDrawable(ColorStateList.valueOf(Color.parseColor("#44FFFFFF")), shape, shape)
    }

    fun badgeDrawable(cornerRadiusPx: Float = 24f): Drawable =
        GradientDrawable(
            GradientDrawable.Orientation.TL_BR,
            intArrayOf(cyan, indigo),
        ).apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerRadiusPx
        }
}

object ThemePalettes {
    val DARK = ThemePalette(
        isLight = false,
        bgTop = Color.parseColor("#040813"),
        bgBottom = Color.parseColor("#0B162C"),
        headerBg = Color.parseColor("#D9081020"),
        cardBg = Color.parseColor("#D90E1930"),
        cardBorder = Color.parseColor("#4038BDF8"),
        autoCardOnStart = Color.parseColor("#E6063B2F"),
        autoCardOnEnd = Color.parseColor("#E60E1F3A"),
        autoCardBorderOn = Color.parseColor("#10B981"),
        pillBg = Color.parseColor("#16233E"),
        pillBorder = Color.parseColor("#4794A3B8"),
        textPrimary = Color.parseColor("#F8FAFC"),
        textSecondary = Color.parseColor("#94A3B8"),
        textMuted = Color.parseColor("#64748B"),
        cyan = Color.parseColor("#00F0FF"),
        indigo = Color.parseColor("#6366F1"),
        emerald = Color.parseColor("#10B981"),
        amber = Color.parseColor("#F59E0B"),
        red = Color.parseColor("#EF4444"),
        cameraBg = Color.parseColor("#040813"),
    )

    val LIGHT = ThemePalette(
        isLight = true,
        bgTop = Color.parseColor("#F8FBFF"),
        bgBottom = Color.parseColor("#DFE8F6"),
        headerBg = Color.parseColor("#F5FFFFFF"),
        cardBg = Color.parseColor("#F5FFFFFF"),
        cardBorder = Color.parseColor("#BFDBFE"),
        autoCardOnStart = Color.parseColor("#ECFDF5"),
        autoCardOnEnd = Color.parseColor("#D1FAE5"),
        autoCardBorderOn = Color.parseColor("#059669"),
        pillBg = Color.parseColor("#EEF4FC"),
        pillBorder = Color.parseColor("#CBD5E1"),
        textPrimary = Color.parseColor("#0F172A"),
        textSecondary = Color.parseColor("#475569"),
        textMuted = Color.parseColor("#64748B"),
        cyan = Color.parseColor("#0284C7"),
        indigo = Color.parseColor("#4F46E5"),
        emerald = Color.parseColor("#059669"),
        amber = Color.parseColor("#D97706"),
        red = Color.parseColor("#DC2626"),
        cameraBg = Color.parseColor("#0F172A"),
    )
}
