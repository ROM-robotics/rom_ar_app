package com.arrobot.controller

import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.os.Bundle
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.arrobot.controller.databinding.ActivityConnectionBinding

/**
 * Connection Settings Activity
 *
 * Allows the user to configure:
 * - Robot IP address
 * - WebSocket port (rosbridge)
 * - Web server port
 */
class ConnectionActivity : AppCompatActivity() {

    private lateinit var binding: ActivityConnectionBinding
    private lateinit var prefs: SharedPreferences

    companion object {
        private const val PREFS_NAME = "ar_robot_prefs"
        private const val KEY_ROBOT_IP = "robot_ip"
        private const val KEY_WS_PORT = "ws_port"
        private const val DEFAULT_ROBOT_IP = "192.168.1.100"
        private const val DEFAULT_WS_PORT = "9090"
    }

    // Track whether this was launched from MainActivity settings
    private var isFromSettings = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityConnectionBinding.inflate(layoutInflater)
        setContentView(binding.root)

        prefs = getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

        // Check if launched from MainActivity settings button
        isFromSettings = intent.getBooleanExtra("from_settings", false)

        // Set toolbar
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(isFromSettings)
        supportActionBar?.title = if (isFromSettings) "Connection Settings" else "AR Robot Controller"

        // Load saved values
        loadSettings()

        // Save button
        binding.btnSave.setOnClickListener {
            saveSettings()
        }

        // Reset button
        binding.btnReset.setOnClickListener {
            resetDefaults()
        }

        // Test connection button
        binding.btnTestConnection.setOnClickListener {
            testConnection()
        }
    }

    private fun loadSettings() {
        binding.editRobotIp.setText(
            prefs.getString(KEY_ROBOT_IP, DEFAULT_ROBOT_IP)
        )
        binding.editWsPort.setText(
            prefs.getString(KEY_WS_PORT, DEFAULT_WS_PORT)
        )

        updatePreview()
    }

    private fun saveSettings() {
        val ip = binding.editRobotIp.text.toString().trim()
        val wsPort = binding.editWsPort.text.toString().trim()

        // Validate
        if (ip.isEmpty()) {
            binding.layoutRobotIp.error = "IP address is required"
            return
        }
        if (wsPort.isEmpty()) {
            binding.layoutWsPort.error = "WebSocket port is required"
            return
        }

        // Clear errors
        binding.layoutRobotIp.error = null
        binding.layoutWsPort.error = null

        // Save
        prefs.edit()
            .putString(KEY_ROBOT_IP, ip)
            .putString(KEY_WS_PORT, wsPort)
            .apply()

        Toast.makeText(this, "✅ Settings saved!", Toast.LENGTH_SHORT).show()

        if (isFromSettings) {
            // Return to MainActivity which will reload with new settings
            setResult(RESULT_OK)
            finish()
        } else {
            // Launch MainActivity (first time / from launcher)
            val intent = Intent(this, MainActivity::class.java)
            startActivity(intent)
            finish()
        }
    }

    private fun resetDefaults() {
        binding.editRobotIp.setText(DEFAULT_ROBOT_IP)
        binding.editWsPort.setText(DEFAULT_WS_PORT)
        updatePreview()

        Toast.makeText(this, "Reset to defaults", Toast.LENGTH_SHORT).show()
    }

    private fun testConnection() {
        val ip = binding.editRobotIp.text.toString().trim()

        binding.btnTestConnection.isEnabled = false
        binding.btnTestConnection.text = "Testing..."

        // Simple reachability test using a background thread
        Thread {
            try {
                val address = java.net.InetAddress.getByName(ip)
                val reachable = address.isReachable(3000)

                runOnUiThread {
                    binding.btnTestConnection.isEnabled = true
                    binding.btnTestConnection.text = "Test Connection"

                    if (reachable) {
                        Toast.makeText(this, "✅ Robot PC is reachable!", Toast.LENGTH_SHORT).show()
                    } else {
                        Toast.makeText(this, "❌ Cannot reach $ip", Toast.LENGTH_SHORT).show()
                    }
                }
            } catch (e: Exception) {
                runOnUiThread {
                    binding.btnTestConnection.isEnabled = true
                    binding.btnTestConnection.text = "Test Connection"
                    Toast.makeText(this, "❌ Error: ${e.message}", Toast.LENGTH_SHORT).show()
                }
            }
        }.start()
    }

    private fun updatePreview() {
        val ip = binding.editRobotIp.text.toString().trim()
        val wsPort = binding.editWsPort.text.toString().trim()

        binding.textPreview.text =
            "Web App: Local (bundled in app)\nROS Bridge: ws://$ip:$wsPort"
    }

    override fun onSupportNavigateUp(): Boolean {
        finish()
        return true
    }
}
