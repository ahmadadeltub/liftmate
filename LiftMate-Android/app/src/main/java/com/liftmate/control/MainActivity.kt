package com.liftmate.control

import android.annotation.SuppressLint
import android.app.AlertDialog
import android.app.Dialog
import android.content.Intent
import android.content.res.Configuration
import android.graphics.Color
import android.graphics.Outline
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.VibrationEffect
import android.os.Vibrator
import android.provider.Settings
import android.text.InputType
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.view.ViewOutlineProvider
import android.view.WindowManager
import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.liftmate.control.views.CameraOverlayView
import com.liftmate.control.views.OmniJoystickView
import com.liftmate.control.views.RobotChassisView
import com.liftmate.control.views.TouchableMapWebView
import com.liftmate.control.views.VerticalFaderView
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.roundToInt

class MainActivity : ComponentActivity() {

    private enum class WorkspaceMode { ALL_CONSOLE, FIT_COCKPIT, FULL_MAP }

    private lateinit var wifiBinder: WifiNetworkBinder
    private lateinit var apiClient: RobotApiClient
    private val jackAlarm = JackAlarmPlayer()
    private val mainHandler = Handler(Looper.getMainLooper())

    private var statusScheduler: ScheduledExecutorService? = null
    private val commandExecutor = Executors.newSingleThreadExecutor()
    private val cameraRunning = AtomicBoolean(false)
    private var cameraThread: Thread? = null

    // Theme & Workspace state
    private var isLightTheme = false
    private var palette: ThemePalette = ThemePalettes.DARK
    private var workspaceMode: WorkspaceMode = WorkspaceMode.ALL_CONSOLE
    private var moveRobotModeActive = false
    private var tapPinModeActive = false
    private var camFillCrop = false

    // Root & Header Views
    private lateinit var rootContainer: LinearLayout
    private lateinit var topHeaderBar: LinearLayout
    private lateinit var tvBrandBadge: TextView
    private lateinit var tvBrandTitle: TextView
    private lateinit var tvTopStatus: TextView
    private lateinit var btnPowerOff: Button
    private lateinit var btnThemeToggle: Button
    private lateinit var btnResetAll: Button
    private lateinit var btnStopAll: Button

    private lateinit var tabAllConsole: Button
    private lateinit var tabFitCockpit: Button
    private lateinit var tabFullMap: Button
    private lateinit var mainScrollView: ScrollView

    // Banner & Cards
    private lateinit var bannerConnectionAlert: LinearLayout
    private lateinit var tvConnectionBannerMsg: TextView
    private lateinit var btnOpenWifi: Button

    private lateinit var cardAutonomous: LinearLayout
    private lateinit var tvAutoHeaderLabel: TextView
    private lateinit var tvAutoTitle: TextView
    private lateinit var tvAutoReason: TextView
    private lateinit var tvAutoSpeedLabel: TextView
    private lateinit var rowAutoSpeed: LinearLayout
    private lateinit var btnToggleAutonomous: Button
    private lateinit var btnAutoSlow: Button
    private lateinit var btnAutoCruise: Button
    private lateinit var btnAutoFast: Button

    private lateinit var cardCamera: LinearLayout
    private lateinit var cameraStageRow: LinearLayout
    private lateinit var cameraFrameWrap: FrameLayout
    private lateinit var cameraServoDock: LinearLayout
    private lateinit var cameraOverlayView: CameraOverlayView
    private lateinit var tvCameraWarning: TextView
    private lateinit var rowCamTools: LinearLayout
    private lateinit var btnAiToggle: Button
    private lateinit var btnAiDetect: Button
    private lateinit var btnAiSafety: Button
    private lateinit var btnMuteSound: Button
    private lateinit var btnCamFitToggle: Button
    private lateinit var btnFullCamera: Button

    // Live Map Card
    private lateinit var cardMap: LinearLayout
    private lateinit var tvMapHeader: TextView
    private lateinit var tvMapStatusChip: TextView
    private lateinit var mapFrameWrap: FrameLayout
    private lateinit var mapWebView: TouchableMapWebView
    private lateinit var mapOfflineOverlay: LinearLayout
    private lateinit var tvMapOverlayMsg: TextView
    private lateinit var btnMapFullscreen: Button
    private lateinit var btnMapResetGyro: Button
    private lateinit var btnMapLocateMe: Button
    private lateinit var btnMapMoveRobot: Button
    private lateinit var btnMapPinHere: Button
    private lateinit var btnMapTapPin: Button
    private lateinit var btnMapCarRepaired: Button
    private lateinit var btnMapClearPins: Button

    // Wheels Card
    private lateinit var cardWheels: LinearLayout
    private lateinit var tvWheelsHeader: TextView
    private lateinit var tvWheelSpeed: TextView
    private lateinit var omniJoystick: OmniJoystickView
    private lateinit var btnRotateLeft: Button
    private lateinit var btnRotateRight: Button
    private lateinit var rowWheelSpeedBar: LinearLayout
    private lateinit var tvSpeedLow: TextView
    private lateinit var tvSpeedMax: TextView
    private lateinit var seekWheelSpeed: SeekBar

    // Actuators Card
    private lateinit var cardActuators: LinearLayout
    private lateinit var tvActHeader: TextView
    private lateinit var tvServoReadout: TextView
    private lateinit var tvLblJack: TextView
    private lateinit var tvLblHolder: TextView
    private lateinit var tvLblBoth: TextView
    private lateinit var tvLblCamera: TextView
    private lateinit var faderJack: VerticalFaderView
    private lateinit var faderHolder: VerticalFaderView
    private lateinit var faderBoth: VerticalFaderView
    private lateinit var faderServo: VerticalFaderView

    // Telemetry & Robot Chassis Card
    private lateinit var cardTelemetry: LinearLayout
    private lateinit var tvTelHeader: TextView
    private lateinit var robotChassisView: RobotChassisView
    private lateinit var tvTelemetryDetails: TextView
    private lateinit var btnConfigureHost: Button
    private lateinit var btnOpenSpareWeb: Button
    private lateinit var btnTelemetryPowerOff: Button

    // Interactive robotics control state
    @Volatile
    private var lastTelemetry = RobotTelemetry()
    private var wheelSpeedPwm = 155
    private var autoSpeedPwm = 90
    private var servoAngleDeg = 68

    private var joyVy = 0f
    private var joyVx = 0f
    private var joyDragging = false
    private var rotateDir = 0

    private var upperActPwm = 0
    private var lowerActPwm = 0

    private val lastWheelTxMs = AtomicLong(0L)
    private val lastActTxMs = AtomicLong(0L)
    private val lastServoTxMs = AtomicLong(0L)
    private val pollTickCounter = AtomicInteger(0)
    private var cameraStartAttemptedForUrl = ""
    private var loadedMapBaseUrl = ""
    private var mapLoadFailed = false

    private var fullscreenDialog: Dialog? = null
    private var fullscreenCamView: CameraOverlayView? = null
    private var fullscreenAutoBtn: Button? = null
    private var fullscreenServoFader: VerticalFaderView? = null
    private var fullscreenServoReadout: TextView? = null

    private val keepaliveRunnable = object : Runnable {
        override fun run() {
            val vy = (joyVy * wheelSpeedPwm).roundToInt()
            val vx = (joyVx * wheelSpeedPwm).roundToInt()
            val omega = rotateDir * wheelSpeedPwm
            if (joyDragging || rotateDir != 0) {
                dispatchWheel(vy, vx, omega, force = false)
            }
            if (upperActPwm != 0 || lowerActPwm != 0) {
                dispatchActuators(upperActPwm, lowerActPwm, force = false)
            }
            mainHandler.postDelayed(this, 130L)
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)

        val prefs = getSharedPreferences("liftmate_prefs", MODE_PRIVATE)
        isLightTheme = prefs.getBoolean("is_light_theme", false)

        wifiBinder = WifiNetworkBinder(this)
        apiClient = RobotApiClient(this)

        bindViews()

        ViewCompat.setOnApplyWindowInsetsListener(rootContainer) { v, insets ->
            val sysBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(sysBars.left, sysBars.top, sysBars.right, sysBars.bottom)
            insets
        }

        setupRoundedFrameClips()
        setupMapWebView()
        setupControls()
        applyTheme(isLightTheme)
        applyWorkspaceMode(workspaceMode)
        renderWheelSpeed(wheelSpeedPwm)
        renderAutoSpeedButtons(autoSpeedPwm)
    }

    override fun onConfigurationChanged(newConfig: Configuration) {
        super.onConfigurationChanged(newConfig)
        applyResponsiveSizing()
    }

    private fun bindViews() {
        rootContainer = findViewById(R.id.rootContainer)
        topHeaderBar = findViewById(R.id.topHeaderBar)
        tvBrandBadge = findViewById(R.id.tvBrandBadge)
        tvBrandTitle = findViewById(R.id.tvBrandTitle)
        tvTopStatus = findViewById(R.id.tvTopStatus)
        btnPowerOff = findViewById(R.id.btnPowerOff)
        btnThemeToggle = findViewById(R.id.btnThemeToggle)
        btnResetAll = findViewById(R.id.btnResetAll)
        btnStopAll = findViewById(R.id.btnStopAll)

        tabAllConsole = findViewById(R.id.tabAllConsole)
        tabFitCockpit = findViewById(R.id.tabFitCockpit)
        tabFullMap = findViewById(R.id.tabFullMap)
        mainScrollView = findViewById(R.id.mainScrollView)

        bannerConnectionAlert = findViewById(R.id.bannerConnectionAlert)
        tvConnectionBannerMsg = findViewById(R.id.tvConnectionBannerMsg)
        btnOpenWifi = findViewById(R.id.btnOpenWifi)

        cardAutonomous = findViewById(R.id.cardAutonomous)
        tvAutoHeaderLabel = findViewById(R.id.tvAutoHeaderLabel)
        tvAutoTitle = findViewById(R.id.tvAutoTitle)
        tvAutoReason = findViewById(R.id.tvAutoReason)
        tvAutoSpeedLabel = findViewById(R.id.tvAutoSpeedLabel)
        rowAutoSpeed = findViewById(R.id.rowAutoSpeed)
        btnToggleAutonomous = findViewById(R.id.btnToggleAutonomous)
        btnAutoSlow = findViewById(R.id.btnAutoSlow)
        btnAutoCruise = findViewById(R.id.btnAutoCruise)
        btnAutoFast = findViewById(R.id.btnAutoFast)

        cardCamera = findViewById(R.id.cardCamera)
        cameraStageRow = findViewById(R.id.cameraStageRow)
        cameraFrameWrap = findViewById(R.id.cameraFrameWrap)
        cameraServoDock = findViewById(R.id.cameraServoDock)
        cameraOverlayView = findViewById(R.id.cameraOverlayView)
        tvCameraWarning = findViewById(R.id.tvCameraWarning)
        rowCamTools = findViewById(R.id.rowCamTools)
        btnAiToggle = findViewById(R.id.btnAiToggle)
        btnAiDetect = findViewById(R.id.btnAiDetect)
        btnAiSafety = findViewById(R.id.btnAiSafety)
        btnMuteSound = findViewById(R.id.btnMuteSound)
        btnCamFitToggle = findViewById(R.id.btnCamFitToggle)
        btnFullCamera = findViewById(R.id.btnFullCamera)

        cardMap = findViewById(R.id.cardMap)
        tvMapHeader = findViewById(R.id.tvMapHeader)
        tvMapStatusChip = findViewById(R.id.tvMapStatusChip)
        mapFrameWrap = findViewById(R.id.mapFrameWrap)
        mapWebView = findViewById(R.id.mapWebView)
        mapOfflineOverlay = findViewById(R.id.mapOfflineOverlay)
        tvMapOverlayMsg = findViewById(R.id.tvMapOverlayMsg)
        btnMapFullscreen = findViewById(R.id.btnMapFullscreen)
        btnMapResetGyro = findViewById(R.id.btnMapResetGyro)
        btnMapLocateMe = findViewById(R.id.btnMapLocateMe)
        btnMapMoveRobot = findViewById(R.id.btnMapMoveRobot)
        btnMapPinHere = findViewById(R.id.btnMapPinHere)
        btnMapTapPin = findViewById(R.id.btnMapTapPin)
        btnMapCarRepaired = findViewById(R.id.btnMapCarRepaired)
        btnMapClearPins = findViewById(R.id.btnMapClearPins)

        cardWheels = findViewById(R.id.cardWheels)
        tvWheelsHeader = findViewById(R.id.tvWheelsHeader)
        tvWheelSpeed = findViewById(R.id.tvWheelSpeed)
        omniJoystick = findViewById(R.id.omniJoystick)
        btnRotateLeft = findViewById(R.id.btnRotateLeft)
        btnRotateRight = findViewById(R.id.btnRotateRight)
        rowWheelSpeedBar = findViewById(R.id.rowWheelSpeedBar)
        tvSpeedLow = findViewById(R.id.tvSpeedLow)
        tvSpeedMax = findViewById(R.id.tvSpeedMax)
        seekWheelSpeed = findViewById(R.id.seekWheelSpeed)

        cardActuators = findViewById(R.id.cardActuators)
        tvActHeader = findViewById(R.id.tvActHeader)
        tvServoReadout = findViewById(R.id.tvServoReadout)
        tvLblJack = findViewById(R.id.tvLblJack)
        tvLblHolder = findViewById(R.id.tvLblHolder)
        tvLblBoth = findViewById(R.id.tvLblBoth)
        tvLblCamera = findViewById(R.id.tvLblCamera)
        faderJack = findViewById(R.id.faderJack)
        faderHolder = findViewById(R.id.faderHolder)
        faderBoth = findViewById(R.id.faderBoth)
        faderServo = findViewById(R.id.faderServo)

        cardTelemetry = findViewById(R.id.cardTelemetry)
        tvTelHeader = findViewById(R.id.tvTelHeader)
        robotChassisView = findViewById(R.id.robotChassisView)
        tvTelemetryDetails = findViewById(R.id.tvTelemetryDetails)
        btnConfigureHost = findViewById(R.id.btnConfigureHost)
        btnOpenSpareWeb = findViewById(R.id.btnOpenSpareWeb)
        btnTelemetryPowerOff = findViewById(R.id.btnTelemetryPowerOff)

        faderServo.isSticky = true
        faderServo.accentStartHex = "#F472B6"
        faderServo.accentEndHex = "#8B5CF6"
        faderServo.setStickyFraction((servoAngleDeg - 6) / 124f)
        tvServoReadout.text = "${servoAngleDeg}°"
    }

    private fun setupRoundedFrameClips() {
        val radiusPx = dpToPx(14).toFloat()
        val provider = object : ViewOutlineProvider() {
            override fun getOutline(view: View, outline: Outline) {
                outline.setRoundRect(0, 0, view.width, view.height, radiusPx)
            }
        }
        cameraFrameWrap.outlineProvider = provider
        cameraFrameWrap.clipToOutline = true
        mapFrameWrap.outlineProvider = provider
        mapFrameWrap.clipToOutline = true
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupMapWebView() {
        mapWebView.settings.apply {
            javaScriptEnabled = true
            domStorageEnabled = true
            mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW
            setSupportZoom(false)
            builtInZoomControls = false
            displayZoomControls = false
            useWideViewPort = true
            loadWithOverviewMode = true
        }
        mapWebView.webViewClient = object : WebViewClient() {
            override fun onPageFinished(view: WebView?, url: String?) {
                if (!mapLoadFailed) {
                    mapOfflineOverlay.visibility = View.GONE
                    syncMapThemeToWebView()
                    mapWebView.postDelayed({
                        runMapJs("window.invalidateMapSize && window.invalidateMapSize()")
                    }, 180L)
                }
            }

            override fun onReceivedError(
                view: WebView?,
                request: WebResourceRequest?,
                error: WebResourceError?,
            ) {
                if (request?.isForMainFrame == true) {
                    mapLoadFailed = true
                    loadedMapBaseUrl = ""
                    mapOfflineOverlay.visibility = View.VISIBLE
                }
            }
        }
    }

    private fun syncMapHost(baseUrl: String?) {
        if (baseUrl.isNullOrBlank()) {
            if (loadedMapBaseUrl.isNotEmpty()) {
                loadedMapBaseUrl = ""
                mapOfflineOverlay.visibility = View.VISIBLE
            }
            return
        }
        if (baseUrl == loadedMapBaseUrl && !mapLoadFailed) return
        loadedMapBaseUrl = baseUrl
        mapLoadFailed = false
        val themeStr = if (isLightTheme) "light" else "dark"
        mapWebView.loadUrl("$baseUrl/mobile-map?theme=$themeStr")
    }

    private fun syncMapThemeToWebView() {
        val themeStr = if (isLightTheme) "light" else "dark"
        runMapJs("window.setAppTheme && window.setAppTheme('$themeStr')")
    }

    private fun runMapJs(script: String) {
        mapWebView.evaluateJavascript(script, null)
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setupControls() {
        findViewById<View>(R.id.btnConnectionHeader).setOnClickListener { showConnectionDialog() }
        btnConfigureHost.setOnClickListener { showConnectionDialog() }
        btnOpenWifi.setOnClickListener { openWifiSettings() }
        btnOpenSpareWeb.setOnClickListener { openSpareWebpage() }
        btnPowerOff.setOnClickListener { showShutdownConfirmationDialog() }
        btnTelemetryPowerOff.setOnClickListener { showShutdownConfirmationDialog() }

        btnThemeToggle.setOnClickListener {
            vibrateShort(15L)
            isLightTheme = !isLightTheme
            getSharedPreferences("liftmate_prefs", MODE_PRIVATE)
                .edit()
                .putBoolean("is_light_theme", isLightTheme)
                .apply()
            applyTheme(isLightTheme)
        }

        tabAllConsole.setOnClickListener { applyWorkspaceMode(WorkspaceMode.ALL_CONSOLE) }
        tabFitCockpit.setOnClickListener { applyWorkspaceMode(WorkspaceMode.FIT_COCKPIT) }
        tabFullMap.setOnClickListener { applyWorkspaceMode(WorkspaceMode.FULL_MAP) }

        btnResetAll.setOnClickListener {
            vibrateShort(25L)
            stopLocalMotionState()
            syncServoUi(6)
            postSimpleControl(ControlPayloadFormatter.buildSimpleModeJson("RESET_ALL"))
        }

        btnStopAll.setOnClickListener {
            triggerEmergencyStopAll()
        }

        btnToggleAutonomous.setOnClickListener { toggleAutonomousMode() }
        btnAutoSlow.setOnClickListener { selectAutoSpeed(70) }
        btnAutoCruise.setOnClickListener { selectAutoSpeed(90) }
        btnAutoFast.setOnClickListener { selectAutoSpeed(110) }

        btnAiToggle.setOnClickListener {
            val nextEnabled = !lastTelemetry.aiVision.enabled
            commandExecutor.execute {
                apiClient.setAiConfig(enabled = nextEnabled)
                mainHandler.post { pollOnceImmediate() }
            }
        }
        btnAiDetect.setOnClickListener {
            commandExecutor.execute {
                apiClient.setAiConfig(enabled = true, mode = "DETECTION")
                mainHandler.post { pollOnceImmediate() }
            }
        }
        btnAiSafety.setOnClickListener {
            commandExecutor.execute {
                apiClient.setAiConfig(enabled = true, mode = "SAFETY_ASSIST")
                mainHandler.post { pollOnceImmediate() }
            }
        }
        btnMuteSound.setOnClickListener {
            jackAlarm.muted = !jackAlarm.muted
            refreshCamToolPills()
        }
        btnCamFitToggle.setOnClickListener {
            camFillCrop = !camFillCrop
            cameraOverlayView.setFillCrop(camFillCrop)
            fullscreenCamView?.setFillCrop(camFillCrop)
            refreshCamToolPills()
        }
        btnFullCamera.setOnClickListener { showFullscreenCameraDialog() }

        // Map action buttons
        btnMapFullscreen.setOnClickListener {
            val next = if (workspaceMode == WorkspaceMode.FULL_MAP) {
                WorkspaceMode.ALL_CONSOLE
            } else {
                WorkspaceMode.FULL_MAP
            }
            applyWorkspaceMode(next)
        }
        btnMapResetGyro.setOnClickListener {
            vibrateShort(15L)
            runMapJs("window.liftmateMap && window.liftmateMap.resetGyro()")
            commandExecutor.execute { apiClient.postEmpty("/api/navigation/calibrate_imu") }
        }
        btnMapLocateMe.setOnClickListener {
            runMapJs("window.liftmateMap && window.liftmateMap.locateMe()")
        }
        btnMapMoveRobot.setOnClickListener {
            moveRobotModeActive = !moveRobotModeActive
            if (moveRobotModeActive) tapPinModeActive = false
            runMapJs("window.liftmateMap && window.liftmateMap.toggleMoveRobotMode()")
            refreshMapActionButtonStates()
        }
        btnMapPinHere.setOnClickListener {
            runMapJs("window.liftmateMap && window.liftmateMap.pinCarRepairHere()")
        }
        btnMapTapPin.setOnClickListener {
            tapPinModeActive = !tapPinModeActive
            if (tapPinModeActive) moveRobotModeActive = false
            runMapJs("window.liftmateMap && window.liftmateMap.toggleTapPinMode()")
            refreshMapActionButtonStates()
        }
        btnMapCarRepaired.setOnClickListener {
            runMapJs("window.liftmateMap && window.liftmateMap.markLatestCarRepaired()")
        }
        btnMapClearPins.setOnClickListener {
            runMapJs("window.liftmateMap && window.liftmateMap.clearRepairPins()")
        }

        // 360 Mecanum Joystick
        omniJoystick.onVectorChanged = { vy, vx, dragging ->
            joyVy = vy
            joyVx = vx
            joyDragging = dragging
            if (dragging) {
                val cmdVy = (vy * wheelSpeedPwm).roundToInt()
                val cmdVx = (vx * wheelSpeedPwm).roundToInt()
                val cmdOmega = rotateDir * wheelSpeedPwm
                dispatchWheel(cmdVy, cmdVx, cmdOmega, force = false)
            } else {
                val cmdOmega = rotateDir * wheelSpeedPwm
                dispatchWheel(0, 0, cmdOmega, force = true)
            }
        }

        btnRotateLeft.setOnTouchListener { v, ev -> handleRotateTouch(v, ev, -1) }
        btnRotateRight.setOnTouchListener { v, ev -> handleRotateTouch(v, ev, +1) }

        seekWheelSpeed.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                wheelSpeedPwm = 50 + progress
                renderWheelSpeed(wheelSpeedPwm)
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        // Actuators & Camera Servo faders
        faderJack.onValueChanged = { valNorm, active ->
            val pwm = if (active) (valNorm * 220f).roundToInt() else 0
            upperActPwm = pwm
            lowerActPwm = 0
            jackAlarm.setActive(pwm != 0)
            dispatchActuators(upperActPwm, lowerActPwm, force = !active)
        }
        faderHolder.onValueChanged = { valNorm, active ->
            val pwm = if (active) (valNorm * 220f).roundToInt() else 0
            upperActPwm = 0
            lowerActPwm = pwm
            jackAlarm.setActive(pwm != 0)
            dispatchActuators(upperActPwm, lowerActPwm, force = !active)
        }
        faderBoth.onValueChanged = { valNorm, active ->
            val pwm = if (active) (valNorm * 220f).roundToInt() else 0
            upperActPwm = pwm
            lowerActPwm = pwm
            jackAlarm.setActive(pwm != 0)
            dispatchActuators(upperActPwm, lowerActPwm, force = !active)
        }
        faderServo.onValueChanged = { fraction, active ->
            val angle = (6 + fraction * 124f).roundToInt().coerceIn(6, 130)
            onServoSliderMoved(angle, active, syncInline = false, syncFullscreen = true)
        }
    }

    private fun syncServoUi(angle: Int, syncInline: Boolean = true, syncFullscreen: Boolean = true) {
        val clamped = angle.coerceIn(6, 130)
        servoAngleDeg = clamped
        val frac = (clamped - 6) / 124f
        tvServoReadout.text = "${clamped}°"
        if (syncInline) {
            faderServo.setStickyFraction(frac)
        }
        fullscreenServoReadout?.text = "${clamped}°"
        if (syncFullscreen) {
            fullscreenServoFader?.setStickyFraction(frac)
        }
        robotChassisView.updateState(
            lastTelemetry.wheelPwms,
            lastTelemetry.aiVision.regions,
            servoAngleDeg,
            lastTelemetry.autonomous.enabled,
        )
    }

    private fun onServoSliderMoved(
        angle: Int,
        active: Boolean,
        syncInline: Boolean,
        syncFullscreen: Boolean,
    ) {
        syncServoUi(angle, syncInline = syncInline, syncFullscreen = syncFullscreen)
        dispatchServo(angle.coerceIn(6, 130), force = !active)
    }

    private fun applyWorkspaceMode(mode: WorkspaceMode) {
        workspaceMode = mode
        refreshWorkspaceTabPills()

        when (mode) {
            WorkspaceMode.ALL_CONSOLE -> {
                cardAutonomous.visibility = View.VISIBLE
                rowAutoSpeed.visibility = View.VISIBLE
                cardCamera.visibility = View.VISIBLE
                rowCamTools.visibility = View.VISIBLE
                cardMap.visibility = View.VISIBLE
                cardWheels.visibility = View.VISIBLE
                rowWheelSpeedBar.visibility = View.VISIBLE
                cardActuators.visibility = View.VISIBLE
                cardTelemetry.visibility = View.VISIBLE
                btnMapFullscreen.text = "Full Map"
            }
            WorkspaceMode.FIT_COCKPIT -> {
                cardAutonomous.visibility = View.VISIBLE
                rowAutoSpeed.visibility = View.GONE
                cardCamera.visibility = View.VISIBLE
                rowCamTools.visibility = View.VISIBLE
                cardMap.visibility = View.GONE
                cardWheels.visibility = View.VISIBLE
                rowWheelSpeedBar.visibility = View.GONE
                cardActuators.visibility = View.GONE
                cardTelemetry.visibility = View.GONE
                mainScrollView.scrollTo(0, 0)
            }
            WorkspaceMode.FULL_MAP -> {
                cardAutonomous.visibility = View.GONE
                cardCamera.visibility = View.GONE
                cardMap.visibility = View.VISIBLE
                cardWheels.visibility = View.GONE
                cardActuators.visibility = View.GONE
                cardTelemetry.visibility = View.GONE
                btnMapFullscreen.text = "Exit Full"
                mainScrollView.scrollTo(0, 0)
            }
        }
        applyResponsiveSizing()
        if (cardMap.visibility == View.VISIBLE) {
            mapWebView.postDelayed({
                runMapJs("window.invalidateMapSize && window.invalidateMapSize()")
            }, 150L)
        }
    }

    private fun applyResponsiveSizing() {
        val dm = resources.displayMetrics
        val screenH = dm.heightPixels
        val density = dm.density

        val camHeightPx: Int
        val mapHeightPx: Int
        val faderHeightPx: Int
        val rotateBtnHeightPx: Int

        when (workspaceMode) {
            WorkspaceMode.FIT_COCKPIT -> {
                // Fit camera + auto toggle + 360 joystick neatly in one viewport without scroll
                camHeightPx = (screenH * 0.25f).roundToInt().coerceIn(dpToPx(135), dpToPx(225))
                mapHeightPx = dpToPx(200)
                faderHeightPx = dpToPx(135)
                rotateBtnHeightPx = dpToPx(68)
            }
            WorkspaceMode.FULL_MAP -> {
                camHeightPx = dpToPx(170)
                mapHeightPx = (screenH * 0.56f).roundToInt().coerceIn(dpToPx(280), dpToPx(560))
                faderHeightPx = dpToPx(140)
                rotateBtnHeightPx = dpToPx(74)
            }
            WorkspaceMode.ALL_CONSOLE -> {
                camHeightPx = (screenH * 0.23f).roundToInt().coerceIn(dpToPx(150), dpToPx(230))
                mapHeightPx = (screenH * 0.27f).roundToInt().coerceIn(dpToPx(190), dpToPx(280))
                faderHeightPx = (screenH * 0.18f).roundToInt().coerceIn(dpToPx(125), dpToPx(165))
                rotateBtnHeightPx = if ((dm.widthPixels / density) < 360f) dpToPx(64) else dpToPx(74)
            }
        }

        cameraStageRow.layoutParams = cameraStageRow.layoutParams.apply { height = camHeightPx }
        mapFrameWrap.layoutParams = mapFrameWrap.layoutParams.apply { height = mapHeightPx }
        for (fader in listOf(faderJack, faderHolder, faderBoth)) {
            fader.layoutParams = fader.layoutParams.apply { height = faderHeightPx }
        }
        btnRotateLeft.layoutParams = btnRotateLeft.layoutParams.apply { height = rotateBtnHeightPx }
        btnRotateRight.layoutParams = btnRotateRight.layoutParams.apply { height = rotateBtnHeightPx }
    }

    private fun applyTheme(isLight: Boolean) {
        palette = if (isLight) ThemePalettes.LIGHT else ThemePalettes.DARK
        btnThemeToggle.text = if (isLight) "🌙" else "☀"

        window.statusBarColor = palette.bgTop
        window.navigationBarColor = palette.bgBottom
        WindowInsetsControllerCompat(window, window.decorView).apply {
            isAppearanceLightStatusBars = isLight
            isAppearanceLightNavigationBars = isLight
        }

        rootContainer.background = palette.screenBackground()
        topHeaderBar.setBackgroundColor(palette.headerBg)
        tvBrandBadge.background = palette.badgeDrawable()
        tvBrandBadge.setTextColor(if (isLight) Color.WHITE else Color.parseColor("#04101E"))
        tvBrandTitle.setTextColor(palette.textPrimary)

        val cards = listOf(bannerConnectionAlert, cardCamera, cardMap, cardWheels, cardActuators, cardTelemetry)
        for (c in cards) {
            c.background = palette.cardDrawable()
        }
        cameraServoDock.background = palette.cardDrawable(cornerRadiusPx = dpToPx(14).toFloat(), strokePx = 2)

        val secondaryLabels = listOf(
            tvAutoHeaderLabel, tvAutoSpeedLabel, tvMapHeader,
            tvWheelsHeader, tvActHeader, tvTelHeader, tvLblCamera,
        )
        for (tv in secondaryLabels) {
            tv.setTextColor(palette.textSecondary)
        }
        val mutedLabels = listOf(tvSpeedLow, tvSpeedMax)
        for (tv in mutedLabels) {
            tv.setTextColor(palette.textMuted)
        }
        val primaryLabels = listOf(tvLblJack, tvLblHolder, tvLblBoth, tvTelemetryDetails)
        for (tv in primaryLabels) {
            tv.setTextColor(palette.textPrimary)
        }
        tvServoReadout.setTextColor(Color.parseColor(if (isLight) "#DB2777" else "#EC4899"))
        tvWheelSpeed.setTextColor(palette.cyan)
        tvMapStatusChip.setTextColor(palette.cyan)

        val plainPills = listOf(
            btnThemeToggle, btnResetAll, btnOpenWifi,
            btnMapFullscreen, btnMapResetGyro, btnMapLocateMe, btnMapPinHere, btnMapClearPins,
            btnRotateLeft, btnRotateRight, btnConfigureHost, btnOpenSpareWeb, btnFullCamera,
        )
        for (btn in plainPills) {
            btn.background = palette.pillDrawable()
            btn.setTextColor(palette.textPrimary)
        }
        btnOpenWifi.setTextColor(palette.cyan)
        btnOpenSpareWeb.setTextColor(palette.cyan)
        for (pBtn in listOf(btnPowerOff, btnTelemetryPowerOff)) {
            pBtn.background = palette.dangerPillDrawable()
            pBtn.setTextColor(palette.red)
        }

        omniJoystick.setLightTheme(isLight)
        for (fader in listOf(faderJack, faderHolder, faderBoth, faderServo)) {
            fader.setLightTheme(isLight)
        }
        robotChassisView.setLightTheme(isLight)

        mapOfflineOverlay.setBackgroundColor(
            if (isLight) Color.parseColor("#EEF1F5F9") else Color.parseColor("#E6080E1E"),
        )
        tvMapOverlayMsg.setTextColor(palette.textSecondary)
        syncMapThemeToWebView()

        refreshWorkspaceTabPills()
        refreshCamToolPills()
        refreshMapActionButtonStates()
        renderAutoSpeedButtons(autoSpeedPwm)
        applyTelemetryUi(lastTelemetry)
    }

    private fun refreshWorkspaceTabPills() {
        val tabs = listOf(
            tabAllConsole to WorkspaceMode.ALL_CONSOLE,
            tabFitCockpit to WorkspaceMode.FIT_COCKPIT,
            tabFullMap to WorkspaceMode.FULL_MAP,
        )
        for ((btn, mode) in tabs) {
            val active = workspaceMode == mode
            btn.background = if (active) {
                palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
            } else {
                palette.pillDrawable()
            }
            btn.setTextColor(if (active) Color.WHITE else palette.textSecondary)
        }
    }

    private fun refreshCamToolPills() {
        btnMuteSound.background = palette.pillDrawable()
        btnMuteSound.text = if (jackAlarm.muted) "🔇 MUTED" else "🔊 SOUND ON"
        btnMuteSound.setTextColor(if (jackAlarm.muted) palette.textMuted else palette.cyan)

        btnCamFitToggle.background = if (camFillCrop) {
            palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
        } else {
            palette.pillDrawable()
        }
        btnCamFitToggle.text = if (camFillCrop) "Fill Crop" else "Fit Frame"
        btnCamFitToggle.setTextColor(if (camFillCrop) Color.WHITE else palette.textPrimary)
    }

    private fun refreshMapActionButtonStates() {
        btnMapMoveRobot.background = if (moveRobotModeActive) {
            palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
        } else {
            palette.pillDrawable()
        }
        btnMapMoveRobot.setTextColor(if (moveRobotModeActive) Color.WHITE else palette.textPrimary)

        btnMapTapPin.background = if (tapPinModeActive) {
            palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
        } else {
            palette.pillDrawable()
        }
        btnMapTapPin.setTextColor(if (tapPinModeActive) Color.WHITE else palette.textPrimary)

        btnMapCarRepaired.background = palette.gradientPillDrawable("#10B981", "#059669", "#80A7F3D0")
        btnMapCarRepaired.setTextColor(Color.WHITE)
    }

    private fun handleRotateTouch(v: View, ev: MotionEvent, dir: Int): Boolean {
        when (ev.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                v.isPressed = true
                rotateDir = dir
                val vy = (joyVy * wheelSpeedPwm).roundToInt()
                val vx = (joyVx * wheelSpeedPwm).roundToInt()
                dispatchWheel(vy, vx, dir * wheelSpeedPwm, force = true)
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                v.isPressed = false
                rotateDir = 0
                val vy = (joyVy * wheelSpeedPwm).roundToInt()
                val vx = (joyVx * wheelSpeedPwm).roundToInt()
                dispatchWheel(vy, vx, 0, force = true)
                return true
            }
        }
        return false
    }

    private fun triggerEmergencyStopAll() {
        vibrateShort(50L)
        stopLocalMotionState()
        postSimpleControl(ControlPayloadFormatter.buildSimpleModeJson("STOP_ALL"))
        Toast.makeText(this, "STOP ALL sent to robot", Toast.LENGTH_SHORT).show()
    }

    private fun showShutdownConfirmationDialog() {
        vibrateShort(35L)
        val dialogStyle = if (isLightTheme) {
            android.R.style.Theme_Material_Light_Dialog_Alert
        } else {
            android.R.style.Theme_Material_Dialog_Alert
        }
        AlertDialog.Builder(this, dialogStyle)
            .setTitle("Shutdown Robot & Raspberry Pi?")
            .setMessage(
                "This safely turns off the robot software and Raspberry Pi 3 OS without extra shutdown hardware:\n\n" +
                    "• Stops all Mecanum wheels, linear actuators, servo & Autonomous mode\n" +
                    "• Locks Arduino Mega 2560 in safe E-STOP & cuts USB hub 5V if supported\n" +
                    "• Cleanly powers off Raspberry Pi 3 OS to protect the SD card\n\n" +
                    "Wait until the green Pi ACT LED stops blinking before switching off the main battery.",
            )
            .setPositiveButton("Shutdown Now") { _, _ -> triggerSystemShutdown() }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun triggerSystemShutdown() {
        vibrateShort(65L)
        stopLocalMotionState()
        tvTopStatus.text = "Shutting down robot & Raspberry Pi OS…"
        tvTopStatus.setTextColor(palette.red)
        commandExecutor.execute {
            val resp = apiClient.shutdownSystem(cutUsbPower = true)
            mainHandler.post {
                if (resp == null) {
                    Toast.makeText(
                        this,
                        "Robot unreachable — connect Wi-Fi to SSID 'LiftMate'",
                        Toast.LENGTH_LONG,
                    ).show()
                } else {
                    val msg = resp.optString(
                        "message",
                        "Shutdown scheduled. Switch off battery after green Pi LED stops.",
                    )
                    bannerConnectionAlert.visibility = View.VISIBLE
                    tvConnectionBannerMsg.text = msg
                    tvConnectionBannerMsg.setTextColor(palette.red)
                    Toast.makeText(this, msg, Toast.LENGTH_LONG).show()
                }
            }
        }
    }

    private fun stopLocalMotionState() {
        joyVy = 0f
        joyVx = 0f
        joyDragging = false
        rotateDir = 0
        upperActPwm = 0
        lowerActPwm = 0
        jackAlarm.stop()
        omniJoystick.resetCenter()
    }

    private fun selectAutoSpeed(speed: Int) {
        autoSpeedPwm = speed
        renderAutoSpeedButtons(speed)
        if (lastTelemetry.autonomous.enabled) {
            commandExecutor.execute {
                apiClient.setAutonomousMode(true, autoSpeedPwm)
                mainHandler.post { pollOnceImmediate() }
            }
        }
    }

    private fun renderAutoSpeedButtons(speed: Int) {
        val mapping = listOf(btnAutoSlow to 70, btnAutoCruise to 90, btnAutoFast to 110)
        for ((btn, valSpeed) in mapping) {
            val active = speed == valSpeed
            btn.background = if (active) {
                palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
            } else {
                palette.pillDrawable()
            }
            btn.setTextColor(if (active) Color.WHITE else palette.textSecondary)
        }
    }

    private fun toggleAutonomousMode() {
        vibrateShort(30L)
        val currentlyAuto = lastTelemetry.autonomous.enabled
        val nextAuto = !currentlyAuto
        stopLocalMotionState()

        commandExecutor.execute {
            val resp = apiClient.setAutonomousMode(nextAuto, autoSpeedPwm)
            mainHandler.post {
                if (resp == null) {
                    Toast.makeText(this, "Robot unreachable — connect Wi-Fi to LiftMate", Toast.LENGTH_SHORT).show()
                } else if (!resp.optBoolean("ok", true)) {
                    val msg = resp.optString("message", "Could not activate Autonomous Mode")
                    Toast.makeText(this, msg, Toast.LENGTH_LONG).show()
                }
                pollOnceImmediate()
            }
        }
    }

    private fun dispatchWheel(vy: Int, vx: Int, omega: Int, force: Boolean) {
        val now = System.currentTimeMillis()
        if (!force && now - lastWheelTxMs.get() < 55L) return
        lastWheelTxMs.set(now)
        postSimpleControl(ControlPayloadFormatter.buildWheelJson(vy, vx, omega))
    }

    private fun dispatchActuators(upper: Int, lower: Int, force: Boolean) {
        val now = System.currentTimeMillis()
        if (!force && now - lastActTxMs.get() < 75L) return
        lastActTxMs.set(now)
        postSimpleControl(ControlPayloadFormatter.buildActuatorJson(upper, lower))
    }

    private fun dispatchServo(angle: Int, force: Boolean) {
        val now = System.currentTimeMillis()
        if (!force && now - lastServoTxMs.get() < 60L) return
        lastServoTxMs.set(now)
        postSimpleControl(ControlPayloadFormatter.buildServoJson(angle))
    }

    private fun postSimpleControl(jsonBody: String) {
        commandExecutor.execute {
            apiClient.sendSimpleControl(jsonBody)
        }
    }

    private fun renderWheelSpeed(pwm: Int) {
        val pct = (((pwm - 50) / 150f) * 80f + 20f).roundToInt()
        tvWheelSpeed.text = "Speed ${pct}% ($pwm PWM)"
    }

    private fun pollOnceImmediate() {
        commandExecutor.execute {
            val tel = apiClient.discoverAndPollStatus()
            val updated = if (tel != null) apiClient.enrichWithFullState(tel) else RobotTelemetry(connectedToRobot = false)
            mainHandler.post { applyTelemetryUi(updated) }
        }
    }

    override fun onResume() {
        super.onResume()
        wifiBinder.start()
        startPollingAndCamera()
        mainHandler.postDelayed(keepaliveRunnable, 130L)
    }

    override fun onPause() {
        super.onPause()
        mainHandler.removeCallbacks(keepaliveRunnable)
        if (joyDragging || rotateDir != 0 || upperActPwm != 0 || lowerActPwm != 0) {
            stopLocalMotionState()
            postSimpleControl(ControlPayloadFormatter.buildWheelJson(0, 0, 0))
            postSimpleControl(ControlPayloadFormatter.buildActuatorJson(0, 0))
        } else {
            jackAlarm.stop()
        }
        stopPollingAndCamera()
        wifiBinder.stop()
    }

    override fun onDestroy() {
        fullscreenDialog?.dismiss()
        mapWebView.destroy()
        super.onDestroy()
    }

    private fun startPollingAndCamera() {
        if (statusScheduler == null) {
            statusScheduler = Executors.newSingleThreadScheduledExecutor().also { exec ->
                exec.scheduleAtFixedRate({
                    try {
                        val basic = apiClient.discoverAndPollStatus()
                        if (basic == null) {
                            mainHandler.post { applyTelemetryUi(RobotTelemetry(connectedToRobot = false)) }
                        } else {
                            if (cameraStartAttemptedForUrl != basic.activeBaseUrl) {
                                cameraStartAttemptedForUrl = basic.activeBaseUrl
                                apiClient.startCamera("remote_pi")
                            }
                            val tick = pollTickCounter.incrementAndGet()
                            val rich = if (tick % 3 == 0) {
                                apiClient.enrichWithFullState(basic)
                            } else {
                                basic.copy(
                                    headingDeg = lastTelemetry.headingDeg,
                                    pitchDeg = lastTelemetry.pitchDeg,
                                    rollDeg = lastTelemetry.rollDeg,
                                    wheelPwms = lastTelemetry.wheelPwms,
                                    cameraStreaming = lastTelemetry.cameraStreaming,
                                )
                            }
                            mainHandler.post { applyTelemetryUi(rich) }
                        }
                    } catch (_: Exception) {
                    }
                }, 0L, 280L, TimeUnit.MILLISECONDS)
            }
        }

        if (cameraRunning.compareAndSet(false, true)) {
            cameraThread = Thread({
                while (cameraRunning.get()) {
                    val bmp = apiClient.fetchCameraFrame()
                    if (bmp != null) {
                        mainHandler.post {
                            cameraOverlayView.updateFrame(bmp)
                            fullscreenCamView?.updateFrame(bmp)
                        }
                        try {
                            Thread.sleep(45L)
                        } catch (_: InterruptedException) {
                            break
                        }
                    } else {
                        try {
                            Thread.sleep(220L)
                        } catch (_: InterruptedException) {
                            break
                        }
                    }
                }
            }, "LiftMateCamLoop").apply {
                isDaemon = true
                start()
            }
        }
    }

    private fun stopPollingAndCamera() {
        statusScheduler?.shutdownNow()
        statusScheduler = null
        cameraRunning.set(false)
        cameraThread?.interrupt()
        cameraThread = null
    }

    private fun applyTelemetryUi(tel: RobotTelemetry) {
        lastTelemetry = tel

        if (!tel.connectedToRobot) {
            tvTopStatus.text = "Searching 10.42.0.1…"
            tvTopStatus.setTextColor(palette.amber)
            bannerConnectionAlert.visibility = View.VISIBLE
            tvConnectionBannerMsg.text = "Searching Robot (10.42.0.1)… Connect Wi-Fi to SSID: LiftMate"
            tvConnectionBannerMsg.setTextColor(palette.amber)
            tvMapStatusChip.text = "Offline"
            syncMapHost(null)
        } else {
            val cleanHost = tel.activeBaseUrl.removePrefix("http://")
            tvMapStatusChip.text = "LIVE • ${tel.headingDeg.roundToInt()}°"
            syncMapHost(tel.activeBaseUrl)
            if (tel.serialConnected) {
                tvTopStatus.text = "LIVE $cleanHost • Mega OK (${tel.latencyMs}ms)"
                tvTopStatus.setTextColor(palette.emerald)
                bannerConnectionAlert.visibility = View.GONE
            } else {
                tvTopStatus.text = "Pi $cleanHost • Plug Mega USB"
                tvTopStatus.setTextColor(palette.red)
                bannerConnectionAlert.visibility = View.VISIBLE
                tvConnectionBannerMsg.text = "Connected to Pi, but Arduino Mega USB is disconnected!"
                tvConnectionBannerMsg.setTextColor(palette.red)
            }
        }

        val auto = tel.autonomous
        val ai = tel.aiVision
        val openPct = (ai.openSpaceRatio * 100f).roundToInt().coerceIn(0, 100)
        val wallTag = if (ai.wallDetected) " • WALL DETECTED" else ""
        if (auto.enabled) {
            cardAutonomous.background = palette.autoCardOnDrawable()
            tvAutoTitle.text = "AUTONOMOUS: ${auto.action}"
            tvAutoTitle.setTextColor(palette.emerald)
            tvAutoReason.text = "${auto.reason} (Open $openPct%$wallTag)"
            tvAutoReason.setTextColor(palette.textSecondary)
            btnToggleAutonomous.background = palette.gradientPillDrawable("#10B981", "#059669", "#80A7F3D0")
            btnToggleAutonomous.setTextColor(Color.WHITE)
            btnToggleAutonomous.text = "STOP → MANUAL"
            fullscreenAutoBtn?.apply {
                background = palette.gradientPillDrawable("#10B981", "#059669", "#80A7F3D0")
                text = "AUTO ON: ${auto.action}"
            }
        } else {
            cardAutonomous.background = palette.cardDrawable()
            tvAutoTitle.text = "MANUAL CONTROL"
            tvAutoTitle.setTextColor(palette.textPrimary)
            tvAutoReason.text = auto.reason.ifBlank { "Tap Activate Auto for smart wall & corridor navigation" }
            tvAutoReason.setTextColor(palette.textSecondary)
            btnToggleAutonomous.background = palette.pillDrawable()
            btnToggleAutonomous.setTextColor(palette.textPrimary)
            btnToggleAutonomous.text = "ACTIVATE AUTO"
            fullscreenAutoBtn?.apply {
                background = palette.pillDrawable()
                setTextColor(palette.textPrimary)
                text = "ACTIVATE AUTO"
            }
        }
        btnAiToggle.background = if (ai.enabled) {
            palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
        } else {
            palette.pillDrawable()
        }
        btnAiToggle.setTextColor(if (ai.enabled) Color.WHITE else palette.textSecondary)
        btnAiToggle.text = if (ai.enabled) "AI ON" else "AI OFF"

        val isSafetyMode = ai.enabled && ai.mode.equals("SAFETY_ASSIST", ignoreCase = true)
        btnAiDetect.background = if (ai.enabled && !isSafetyMode) {
            palette.gradientPillDrawable("#06B6D4", "#4F46E5", "#99BAE6FD")
        } else {
            palette.pillDrawable()
        }
        btnAiDetect.setTextColor(if (ai.enabled && !isSafetyMode) Color.WHITE else palette.textSecondary)

        btnAiSafety.background = if (isSafetyMode) {
            palette.gradientPillDrawable("#10B981", "#059669", "#80A7F3D0")
        } else {
            palette.pillDrawable()
        }
        btnAiSafety.setTextColor(if (isSafetyMode) Color.WHITE else palette.textSecondary)

        if (ai.warnings.isNotEmpty()) {
            tvCameraWarning.visibility = View.VISIBLE
            tvCameraWarning.text = ai.warnings.first()
        } else {
            tvCameraWarning.visibility = View.GONE
        }

        cameraOverlayView.updateTelemetry(ai, auto)
        fullscreenCamView?.updateTelemetry(ai, auto)
        robotChassisView.updateState(tel.wheelPwms, ai.regions, servoAngleDeg, auto.enabled)

        val pwmStr = tel.wheelPwms.joinToString("/")
        tvTelemetryDetails.text =
            "Port: ${tel.serialPort}\n" +
                "Safety: ${tel.safetyState}\n" +
                "Heading: ${tel.headingDeg.roundToInt()}° | Tilt: ${String.format("%.1f", tel.pitchDeg)}°/${String.format("%.1f", tel.rollDeg)}°\n" +
                "PWM [RL/RR/FL/FR]: $pwmStr"
    }

    private fun showConnectionDialog() {
        val currentCustom = apiClient.customHost
        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(48, 28, 48, 12)
        }
        val info = TextView(this).apply {
            text = "LiftMate auto-connects to 10.42.0.1 when Wi-Fi is on SSID 'LiftMate'.\n" +
                "Active: ${apiClient.activeBaseUrl ?: "Not connected yet"}\n\n" +
                "Optional Custom Robot URL / IP:"
            textSize = 13f
            setTextColor(palette.textSecondary)
        }
        val input = EditText(this).apply {
            hint = "e.g. 192.168.18.36:8000"
            setText(currentCustom)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setTextColor(palette.textPrimary)
            setHintTextColor(palette.textMuted)
        }
        layout.addView(info)
        layout.addView(input)

        val dialogStyle = if (isLightTheme) {
            android.R.style.Theme_Material_Light_Dialog_Alert
        } else {
            android.R.style.Theme_Material_Dialog_Alert
        }
        AlertDialog.Builder(this, dialogStyle)
            .setTitle("Robot Connection & Wi-Fi")
            .setView(layout)
            .setPositiveButton("Save & Reconnect") { _, _ ->
                val raw = input.text.toString().trim()
                apiClient.customHost = raw
                if (raw.isNotEmpty()) {
                    apiClient.setPreferredUrl(raw)
                }
                pollOnceImmediate()
            }
            .setNeutralButton("Open Wi-Fi") { _, _ -> openWifiSettings() }
            .setNegativeButton("Close", null)
            .show()
    }

    private fun openWifiSettings() {
        try {
            val intent = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                Intent(Settings.Panel.ACTION_WIFI)
            } else {
                Intent(Settings.ACTION_WIFI_SETTINGS)
            }
            startActivity(intent)
        } catch (_: Exception) {
            try {
                startActivity(Intent(Settings.ACTION_WIFI_SETTINGS))
            } catch (_: Exception) {
            }
        }
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun openSpareWebpage() {
        val target = apiClient.activeBaseUrl ?: "http://10.42.0.1"
        val dialog = Dialog(this, android.R.style.Theme_Black_NoTitleBar_Fullscreen)
        val container = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(palette.bgTop)
        }
        val topBar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
            setPadding(24, 18, 24, 14)
            setBackgroundColor(palette.headerBg)
        }
        val title = TextView(this).apply {
            text = "Spare Web UI ($target)"
            setTextColor(palette.textPrimary)
            textSize = 14f
            layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
        }
        val extBtn = Button(this).apply {
            text = "Browser"
            setTextColor(palette.cyan)
            background = palette.pillDrawable()
            setOnClickListener {
                try {
                    startActivity(Intent(Intent.ACTION_VIEW, Uri.parse(target)))
                } catch (_: Exception) {
                }
            }
        }
        val closeBtn = Button(this).apply {
            text = "Close"
            setTextColor(palette.textPrimary)
            background = palette.pillDrawable()
            setOnClickListener { dialog.dismiss() }
        }
        topBar.addView(title)
        topBar.addView(extBtn)
        topBar.addView(closeBtn)

        val webView = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            webViewClient = WebViewClient()
            loadUrl(target)
        }
        container.addView(topBar)
        container.addView(
            webView,
            LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f,
            ),
        )
        dialog.setContentView(container)
        dialog.setOnDismissListener { webView.destroy() }
        dialog.show()
    }

    private fun showFullscreenCameraDialog() {
        fullscreenDialog?.dismiss()
        val dialog = Dialog(this, android.R.style.Theme_Black_NoTitleBar_Fullscreen)
        val root = FrameLayout(this).apply {
            setBackgroundColor(Color.BLACK)
        }
        val camView = CameraOverlayView(this).apply {
            setFillCrop(camFillCrop)
        }
        root.addView(
            camView,
            FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT,
            ),
        )

        val controlsBar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
            setPadding(dpToPx(14), dpToPx(12), dpToPx(14), dpToPx(34))
        }
        ViewCompat.setOnApplyWindowInsetsListener(controlsBar) { v, insets ->
            val sysBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(
                dpToPx(14) + sysBars.left,
                dpToPx(12),
                dpToPx(14) + sysBars.right,
                dpToPx(18) + sysBars.bottom,
            )
            insets
        }
        val autoBtn = Button(this).apply {
            text = if (lastTelemetry.autonomous.enabled) "AUTO ON: ${lastTelemetry.autonomous.action}" else "ACTIVATE AUTO"
            setTextColor(Color.WHITE)
            textSize = 11.5f
            isSingleLine = true
            minWidth = 0
            minimumWidth = 0
            minHeight = 0
            minimumHeight = 0
            gravity = android.view.Gravity.CENTER
            setPadding(dpToPx(14), 0, dpToPx(14), 0)
            setBackgroundResource(if (lastTelemetry.autonomous.enabled) R.drawable.bg_pill_active_green else R.drawable.bg_pill_button)
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                dpToPx(40),
            )
            setOnClickListener { toggleAutonomousMode() }
        }
        val stopBtn = Button(this).apply {
            text = "STOP ALL"
            setTextColor(Color.WHITE)
            textSize = 11.5f
            isSingleLine = true
            minWidth = 0
            minimumWidth = 0
            minHeight = 0
            minimumHeight = 0
            gravity = android.view.Gravity.CENTER
            setPadding(dpToPx(14), 0, dpToPx(14), 0)
            setBackgroundResource(R.drawable.bg_stop_button)
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                dpToPx(40),
            ).apply { marginEnd = dpToPx(8) }
            setOnClickListener { triggerEmergencyStopAll() }
        }
        val exitBtn = Button(this).apply {
            text = "CLOSE"
            setTextColor(Color.WHITE)
            textSize = 11.5f
            isSingleLine = true
            minWidth = 0
            minimumWidth = 0
            minHeight = 0
            minimumHeight = 0
            gravity = android.view.Gravity.CENTER
            setPadding(dpToPx(14), 0, dpToPx(14), 0)
            setBackgroundResource(R.drawable.bg_pill_button)
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                dpToPx(40),
            )
            setOnClickListener { dialog.dismiss() }
        }
        val spacer = View(this).apply {
            layoutParams = LinearLayout.LayoutParams(0, 1, 1f)
        }
        controlsBar.addView(autoBtn)
        controlsBar.addView(spacer)
        controlsBar.addView(stopBtn)
        controlsBar.addView(exitBtn)

        val barParams = FrameLayout.LayoutParams(
            FrameLayout.LayoutParams.MATCH_PARENT,
            FrameLayout.LayoutParams.WRAP_CONTENT,
        ).apply {
            gravity = android.view.Gravity.BOTTOM
        }
        root.addView(controlsBar, barParams)

        // Docked Camera Tilt Servo slider right beside fullscreen camera feed
        val fsServoDock = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = android.view.Gravity.CENTER_HORIZONTAL
            background = android.graphics.drawable.GradientDrawable().apply {
                shape = android.graphics.drawable.GradientDrawable.RECTANGLE
                cornerRadius = dpToPx(16).toFloat()
                setColor(Color.parseColor("#BF0B1324"))
                setStroke(2, Color.parseColor("#66F472B6"))
            }
            setPadding(dpToPx(4), dpToPx(8), dpToPx(4), dpToPx(8))
        }
        val fsServoTitle = TextView(this).apply {
            text = "SERVO"
            setTextColor(Color.parseColor("#CBD5E1"))
            textSize = 9.5f
            setTypeface(null, android.graphics.Typeface.BOLD)
        }
        val fsServoVal = TextView(this).apply {
            text = "${servoAngleDeg}°"
            setTextColor(Color.parseColor("#F472B6"))
            textSize = 11.5f
            setTypeface(null, android.graphics.Typeface.BOLD)
        }
        val fsServo = VerticalFaderView(this).apply {
            isSticky = true
            accentStartHex = "#F472B6"
            accentEndHex = "#8B5CF6"
            setStickyFraction((servoAngleDeg - 6) / 124f)
            setLightTheme(false)
            onValueChanged = { frac, active ->
                val ang = (6 + frac * 124f).roundToInt().coerceIn(6, 130)
                onServoSliderMoved(ang, active, syncInline = true, syncFullscreen = false)
            }
        }
        fsServoDock.addView(fsServoTitle)
        fsServoDock.addView(fsServoVal)
        fsServoDock.addView(
            fsServo,
            LinearLayout.LayoutParams(dpToPx(50), 0, 1f).apply {
                topMargin = dpToPx(4)
            },
        )
        val dockParams = FrameLayout.LayoutParams(
            dpToPx(60),
            dpToPx(240),
        ).apply {
            gravity = android.view.Gravity.END or android.view.Gravity.CENTER_VERTICAL
            marginEnd = dpToPx(12)
        }
        root.addView(fsServoDock, dockParams)

        dialog.setContentView(root)
        dialog.setOnDismissListener {
            fullscreenCamView = null
            fullscreenAutoBtn = null
            fullscreenServoFader = null
            fullscreenServoReadout = null
            fullscreenDialog = null
        }
        fullscreenCamView = camView
        fullscreenAutoBtn = autoBtn
        fullscreenServoFader = fsServo
        fullscreenServoReadout = fsServoVal
        fullscreenDialog = dialog
        camView.updateTelemetry(lastTelemetry.aiVision, lastTelemetry.autonomous)
        dialog.show()
    }

    private fun dpToPx(dp: Int): Int =
        (dp * resources.displayMetrics.density).roundToInt()

    @Suppress("DEPRECATION")
    private fun vibrateShort(ms: Long) {
        try {
            val vib = getSystemService(VIBRATOR_SERVICE) as? Vibrator ?: return
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                vib.vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
            } else {
                vib.vibrate(ms)
            }
        } catch (_: Exception) {
        }
    }
}
