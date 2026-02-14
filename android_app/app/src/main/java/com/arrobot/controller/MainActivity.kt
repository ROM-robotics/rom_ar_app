package com.arrobot.controller

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.webkit.*
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.arrobot.controller.databinding.ActivityMainBinding

/**
 * Main Activity
 *
 * Hosts a full-screen WebView that loads the AR Robot Controller web app.
 * Handles camera permissions, WebView configuration, JavaScript bridge,
 * and immersive fullscreen mode.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var prefs: SharedPreferences

    private var fileUploadCallback: ValueCallback<Array<Uri>>? = null

    companion object {
        private const val CAMERA_PERMISSION_CODE = 100
        private const val FILE_CHOOSER_CODE = 200
        private const val SETTINGS_REQUEST_CODE = 300
        private const val PREFS_NAME = "ar_robot_prefs"
        private const val KEY_ROBOT_IP = "robot_ip"
        private const val KEY_WS_PORT = "ws_port"
        private const val KEY_WEB_PORT = "web_port"
        private const val DEFAULT_ROBOT_IP = "192.168.1.100"
        private const val DEFAULT_WS_PORT = "9090"
        private const val DEFAULT_WEB_PORT = "8080"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        prefs = getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

        // Keep screen on
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Setup UI
        setupFullscreen()
        setupButtons()

        // Check camera permission then load
        if (hasCameraPermission()) {
            setupWebView()
            loadApp()
        } else {
            requestCameraPermission()
        }
    }

    // ============================================
    // Fullscreen Immersive
    // ============================================

    private fun setupFullscreen() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.insetsController?.let {
                it.hide(WindowInsets.Type.statusBars() or WindowInsets.Type.navigationBars())
                it.systemBarsBehavior =
                    WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                    or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                    or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                    or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                    or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                    or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                )
        }
    }

    // ============================================
    // Buttons
    // ============================================

    private fun setupButtons() {
        // Settings button (only native button — all other controls are in the web app)
        binding.btnSettings.setOnClickListener {
            val intent = Intent(this, ConnectionActivity::class.java)
            intent.putExtra("from_settings", true)
            @Suppress("DEPRECATION")
            startActivityForResult(intent, SETTINGS_REQUEST_CODE)
        }
    }

    // ============================================
    // WebView Setup
    // ============================================

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupWebView() {
        val webView = binding.webView

        // WebView settings
        webView.settings.apply {
            javaScriptEnabled = true
            domStorageEnabled = true
            mediaPlaybackRequiresUserGesture = false
            allowContentAccess = true
            allowFileAccess = true
            databaseEnabled = true
            setSupportMultipleWindows(false)
            loadWithOverviewMode = true
            useWideViewPort = true
            builtInZoomControls = false
            displayZoomControls = false
            cacheMode = WebSettings.LOAD_DEFAULT
            mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW
        }

        // JavaScript interface for native features
        webView.addJavascriptInterface(WebAppInterface(this), "AndroidBridge")

        // WebView client
        webView.webViewClient = object : WebViewClient() {
            override fun onPageFinished(view: WebView?, url: String?) {
                super.onPageFinished(view, url)
                binding.loadingOverlay.visibility = View.GONE
                // Inject robot IP into the web app
                injectConnectionConfig()
            }

            override fun onReceivedError(
                view: WebView?, request: WebResourceRequest?, error: WebResourceError?
            ) {
                super.onReceivedError(view, request, error)
                if (request?.isForMainFrame == true) {
                    showConnectionError()
                }
            }
        }

        // WebChrome client (camera permission for WebRTC / getUserMedia)
        webView.webChromeClient = object : WebChromeClient() {
            override fun onPermissionRequest(request: PermissionRequest?) {
                request?.let {
                    val resources = it.resources
                    val grantedResources = mutableListOf<String>()

                    for (resource in resources) {
                        when (resource) {
                            PermissionRequest.RESOURCE_VIDEO_CAPTURE -> {
                                if (hasCameraPermission()) {
                                    grantedResources.add(resource)
                                }
                            }
                            PermissionRequest.RESOURCE_AUDIO_CAPTURE -> {
                                grantedResources.add(resource)
                            }
                        }
                    }

                    if (grantedResources.isNotEmpty()) {
                        it.grant(grantedResources.toTypedArray())
                    } else {
                        it.deny()
                    }
                }
            }

            override fun onConsoleMessage(consoleMessage: ConsoleMessage?): Boolean {
                consoleMessage?.let {
                    android.util.Log.d("WebView", "${it.message()} [${it.lineNumber()}]")
                }
                return true
            }

            override fun onShowFileChooser(
                webView: WebView?,
                filePathCallback: ValueCallback<Array<Uri>>?,
                fileChooserParams: FileChooserParams?
            ): Boolean {
                fileUploadCallback = filePathCallback
                val intent = fileChooserParams?.createIntent()
                try {
                    if (intent != null) {
                        startActivityForResult(intent, FILE_CHOOSER_CODE)
                    } else {
                        fileUploadCallback = null
                        return false
                    }
                } catch (e: Exception) {
                    fileUploadCallback = null
                    return false
                }
                return true
            }
        }
    }

    // ============================================
    // Load Web App
    // ============================================

    private fun loadApp() {
        binding.loadingOverlay.visibility = View.VISIBLE
        binding.errorOverlay.visibility = View.GONE

        val robotIp = prefs.getString(KEY_ROBOT_IP, DEFAULT_ROBOT_IP) ?: DEFAULT_ROBOT_IP
        val webPort = prefs.getString(KEY_WEB_PORT, DEFAULT_WEB_PORT) ?: DEFAULT_WEB_PORT

        val url = "http://$robotIp:$webPort"
        binding.webView.loadUrl(url)
    }

    private fun injectConnectionConfig() {
        val robotIp = prefs.getString(KEY_ROBOT_IP, DEFAULT_ROBOT_IP) ?: DEFAULT_ROBOT_IP
        val wsPort = prefs.getString(KEY_WS_PORT, DEFAULT_WS_PORT) ?: DEFAULT_WS_PORT
        val wsUrl = "ws://$robotIp:$wsPort"

        // Inject the ROS bridge URL into the web app
        val js = """
            (function() {
                // Update ROS bridge URL
                var rosUrlInput = document.getElementById('ros-url');
                if (rosUrlInput) {
                    rosUrlInput.value = '$wsUrl';
                }
                // Auto-reconnect with new URL
                if (typeof reconnectROS === 'function') {
                    reconnectROS();
                } else if (typeof rosBridge !== 'undefined') {
                    rosBridge.connect('$wsUrl');
                }
                console.log('Android: Injected ROS URL: $wsUrl');
            })();
        """.trimIndent()

        binding.webView.evaluateJavascript(js, null)
    }

    private fun showConnectionError() {
        binding.loadingOverlay.visibility = View.GONE
        binding.errorOverlay.visibility = View.VISIBLE

        binding.btnRetry.setOnClickListener {
            loadApp()
        }

        binding.btnOpenSettings.setOnClickListener {
            val intent = Intent(this, ConnectionActivity::class.java)
            startActivity(intent)
        }
    }

    // ============================================
    // Camera Permission
    // ============================================

    private fun hasCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            this, Manifest.permission.CAMERA
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestCameraPermission() {
        ActivityCompat.requestPermissions(
            this,
            arrayOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO),
            CAMERA_PERMISSION_CODE
        )
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (requestCode == CAMERA_PERMISSION_CODE) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                setupWebView()
                loadApp()
            } else {
                Toast.makeText(
                    this,
                    "Camera permission is required for hand tracking",
                    Toast.LENGTH_LONG
                ).show()
                // Load anyway (joystick mode still works)
                setupWebView()
                loadApp()
            }
        }
    }

    // ============================================
    // Haptic Feedback
    // ============================================

    private fun vibrate() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val vibratorManager = getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as VibratorManager
            val vibrator = vibratorManager.defaultVibrator
            vibrator.vibrate(VibrationEffect.createOneShot(50, VibrationEffect.DEFAULT_AMPLITUDE))
        } else {
            @Suppress("DEPRECATION")
            val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                vibrator.vibrate(VibrationEffect.createOneShot(50, VibrationEffect.DEFAULT_AMPLITUDE))
            } else {
                @Suppress("DEPRECATION")
                vibrator.vibrate(50)
            }
        }
    }

    // ============================================
    // Lifecycle
    // ============================================

    override fun onResume() {
        super.onResume()
        setupFullscreen()
    }

    override fun onBackPressed() {
        if (binding.webView.canGoBack()) {
            binding.webView.goBack()
        } else {
            super.onBackPressed()
        }
    }

    override fun onDestroy() {
        binding.webView.destroy()
        super.onDestroy()
    }

    @Deprecated("Deprecated in Java")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == FILE_CHOOSER_CODE) {
            fileUploadCallback?.onReceiveValue(
                WebChromeClient.FileChooserParams.parseResult(resultCode, data)
            )
            fileUploadCallback = null
        } else if (requestCode == SETTINGS_REQUEST_CODE) {
            // Reload web app with new connection settings
            loadApp()
        }
    }
}
