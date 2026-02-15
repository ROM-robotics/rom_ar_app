package com.arrobot.controller

import android.content.Context
import android.webkit.JavascriptInterface
import android.widget.Toast

/**
 * JavaScript ↔ Android Bridge
 *
 * Allows the web app to call native Android features:
 * - Haptic feedback (vibration)
 * - Toast messages
 * - Get/save connection settings
 * - Native emergency stop
 *
 * Usage from JavaScript:
 *   AndroidBridge.vibrate(50);
 *   AndroidBridge.showToast("Connected!");
 *   AndroidBridge.getRobotIP();
 */
class WebAppInterface(private val context: Context) {

    private val prefs = context.getSharedPreferences("ar_robot_prefs", Context.MODE_PRIVATE)

    @JavascriptInterface
    fun showToast(message: String) {
        Toast.makeText(context, message, Toast.LENGTH_SHORT).show()
    }

    @JavascriptInterface
    fun vibrate(durationMs: Long) {
        val vibrator = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            val vibratorManager = context.getSystemService(Context.VIBRATOR_MANAGER_SERVICE)
                as android.os.VibratorManager
            vibratorManager.defaultVibrator
        } else {
            @Suppress("DEPRECATION")
            context.getSystemService(Context.VIBRATOR_SERVICE) as android.os.Vibrator
        }

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
            vibrator.vibrate(
                android.os.VibrationEffect.createOneShot(
                    durationMs,
                    android.os.VibrationEffect.DEFAULT_AMPLITUDE
                )
            )
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(durationMs)
        }
    }

    @JavascriptInterface
    fun vibratePattern(pattern: String) {
        // Pattern format: "0,50,100,50" (delay, vibrate, delay, vibrate...)
        val timings = pattern.split(",").mapNotNull { it.trim().toLongOrNull() }.toLongArray()
        if (timings.isEmpty()) return

        val vibrator = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            val vibratorManager = context.getSystemService(Context.VIBRATOR_MANAGER_SERVICE)
                as android.os.VibratorManager
            vibratorManager.defaultVibrator
        } else {
            @Suppress("DEPRECATION")
            context.getSystemService(Context.VIBRATOR_SERVICE) as android.os.Vibrator
        }

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
            vibrator.vibrate(android.os.VibrationEffect.createWaveform(timings, -1))
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(timings, -1)
        }
    }

    @JavascriptInterface
    fun getRobotIP(): String {
        return prefs.getString("robot_ip", "192.168.1.100") ?: "192.168.1.100"
    }

    @JavascriptInterface
    fun getWSPort(): String {
        return prefs.getString("ws_port", "9090") ?: "9090"
    }

    @JavascriptInterface
    fun getROSBridgeURL(): String {
        val ip = getRobotIP()
        val port = getWSPort()
        return "ws://$ip:$port"
    }

    @JavascriptInterface
    fun saveSettings(robotIp: String, wsPort: String) {
        prefs.edit()
            .putString("robot_ip", robotIp)
            .putString("ws_port", wsPort)
            .apply()
    }

    @JavascriptInterface
    fun isAndroidApp(): Boolean {
        return true
    }

    @JavascriptInterface
    fun getAppVersion(): String {
        return try {
            val pInfo = context.packageManager.getPackageInfo(context.packageName, 0)
            pInfo.versionName ?: "1.0.0"
        } catch (e: Exception) {
            "1.0.0"
        }
    }

    @JavascriptInterface
    fun log(message: String) {
        android.util.Log.d("ARRobotController", message)
    }
}
