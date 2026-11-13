package com.liftmate.control

import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioTrack
import kotlin.math.PI
import kotlin.math.sin

/**
 * Generates a loud dual-tone piezoelectric safety siren while linear actuators (Jack/Holder)
 * are actively moving, alerting nearby people cleanly on mobile without external audio files.
 */
class JackAlarmPlayer {
    @Volatile
    var muted: Boolean = false
        set(value) {
            field = value
            if (value) stop()
        }

    @Volatile
    private var playing: Boolean = false
    private var workerThread: Thread? = null

    fun setActive(active: Boolean) {
        if (muted || !active) {
            stop()
            return
        }
        start()
    }

    @Synchronized
    fun start() {
        if (muted || playing) return
        playing = true
        workerThread = Thread({
            val sampleRate = 22050
            val cycleMs = 340
            val numSamples = (sampleRate * cycleMs) / 1000
            val pcm = ShortArray(numSamples)
            for (i in 0 until numSamples) {
                val t = i.toDouble() / sampleRate
                val inBeep = i < (numSamples * 7) / 10
                pcm[i] = if (inBeep) {
                    val f = if (i < (numSamples * 35) / 100) 980.0 else 1420.0
                    val sample = 0.85 * sin(2.0 * PI * f * t) + 0.15 * sin(2.0 * PI * f * 2.0 * t)
                    (sample.coerceIn(-1.0, 1.0) * Short.MAX_VALUE).toInt().toShort()
                } else {
                    0
                }
            }

            var track: AudioTrack? = null
            try {
                val minBuf = AudioTrack.getMinBufferSize(
                    sampleRate,
                    AudioFormat.CHANNEL_OUT_MONO,
                    AudioFormat.ENCODING_PCM_16BIT,
                ).coerceAtLeast(pcm.size * 2)

                track = AudioTrack.Builder()
                    .setAudioAttributes(
                        AudioAttributes.Builder()
                            .setUsage(AudioAttributes.USAGE_ALARM)
                            .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                            .build(),
                    )
                    .setAudioFormat(
                        AudioFormat.Builder()
                            .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                            .setSampleRate(sampleRate)
                            .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                            .build(),
                    )
                    .setTransferMode(AudioTrack.MODE_STREAM)
                    .setBufferSizeInBytes(minBuf)
                    .build()

                track.play()
                while (playing && !muted) {
                    track.write(pcm, 0, pcm.size)
                }
            } catch (_: Exception) {
            } finally {
                try {
                    track?.stop()
                    track?.release()
                } catch (_: Exception) {
                }
            }
        }, "LiftMateJackAlarm").apply {
            isDaemon = true
            start()
        }
    }

    @Synchronized
    fun stop() {
        playing = false
        workerThread?.interrupt()
        workerThread = null
    }
}
