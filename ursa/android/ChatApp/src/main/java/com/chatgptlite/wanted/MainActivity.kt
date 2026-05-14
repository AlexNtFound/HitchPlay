package com.chatgptlite.wanted

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.system.Os
import android.util.Log
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.viewModels
import androidx.compose.animation.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.BatteryFull
import androidx.compose.material.icons.filled.ControlCamera
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Menu
import androidx.compose.material.icons.filled.Mic
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.chatgptlite.wanted.data.whisper.asr.IRecorderListener
import com.chatgptlite.wanted.data.whisper.asr.IWhisperListener
import com.chatgptlite.wanted.data.whisper.asr.Recorder
import com.chatgptlite.wanted.data.whisper.asr.Whisper
import com.chatgptlite.wanted.data.whisper.utils.WaveUtil
import com.chatgptlite.wanted.helpers.sendMessage
import com.chatgptlite.wanted.services.getFilePath
import com.chatgptlite.wanted.ui.NavRoute
// AdvanceViewModel no longer needed — advance is now an overlay panel
import com.chatgptlite.wanted.ui.settings.controller.VideoCamSettingsViewModel
import com.chatgptlite.wanted.ui.settings.controller.VideoStreamingSetting
import com.chatgptlite.wanted.ui.settings.network.NetworkConfigScreen
import com.chatgptlite.wanted.ui.settings.occupancy.Occupancy
import com.chatgptlite.wanted.ui.settings.occupancy.OccupancyViewModel
import com.chatgptlite.wanted.ui.settings.prompt.PromptSettingPage
import com.chatgptlite.wanted.ui.settings.rover.RoverSettingsViewModel
import com.chatgptlite.wanted.ui.settings.rover.SettingsScreen
import com.chatgptlite.wanted.ui.settings.rover.TelemetryScreen
import com.chatgptlite.wanted.ui.settings.rover.TelemetryViewModel
import com.chatgptlite.wanted.ui.settings.terminal.TerminalScreen
import com.chatgptlite.wanted.ui.settings.terminal.TerminalViewModel
import com.chatgptlite.wanted.ui.theme.ChatGPTLiteTheme
import com.quicinc.chatapp.ChatMessage
import com.quicinc.chatapp.GenieWrapper
import com.quicinc.chatapp.R
import com.quicinc.chatapp.StringCallback
import com.quicinc.chatapp.provisioning.ModelManifest
import com.quicinc.chatapp.provisioning.ProvisioningActivity
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.map
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStream
import java.nio.file.Path
import java.nio.file.Paths

@AndroidEntryPoint
class MainActivity : ComponentActivity() {
    companion object {
        init {
            System.loadLibrary("chatapp")
        }
        private const val REQUEST_RECORD_AUDIO = 100
    }

    private val mainViewModel: MainViewModel by viewModels()
    private val TAG = "MainActivity"
    private val micVisibleState = mutableStateOf(false)
    private lateinit var voiceWakeupManager: VoiceWakeupManager

    var text = mutableStateOf("")
    var genieResponse = mutableStateOf("")
    private val isForegroundRecording = mutableStateOf(false)
    lateinit var genieWrapper: GenieWrapper

    /** Launches [ProvisioningActivity] and recreates this activity on success. */
    private val provisioningLauncher: ActivityResultLauncher<Intent> =
        registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
            if (result.resultCode == RESULT_OK) {
                recreate()
            } else {
                Toast.makeText(
                    this,
                    "Model setup was cancelled. Cannot start chat.",
                    Toast.LENGTH_LONG
                ).show()
                finish()
            }
        }


    private fun startRecorder() {
        var modelPath = getFilePath(this, "whisper-tiny-en.tflite")
        var vocabPath = getFilePath(this, "filters_vocab_en.bin")
        Log.d(TAG, "Starting recorder from MainActivity.")
        updateForegroundRecordingSignal(true)
        isForegroundRecording.value = true
        micVisibleState.value = true

        val whisper = Whisper(this).apply {
            loadModel(modelPath, vocabPath, false)
            setListener(object : IWhisperListener {
                override fun onUpdateReceived(message: String?) {
                    Log.d("foreground", "onUpdateReceived: $message")
                }

                override fun onResultReceived(result: String?) {
                    Log.d("foreground", "onResultReceived: $result")
                    text.value = result ?: ""

                    result?.let {
                        genieResponse.value = ""

                        getGenieResponse(it) { responseToken ->
                            runOnUiThread {
                                genieResponse.value += responseToken
                                Log.d("MainActivity", "Genie response so far: $genieResponse")
                            }
                        }

                        val addr = "10.0.0.1"
                        val port = "8000"
                    }
                }

            })
        }

        val waveFilePath = getFilePath(this, WaveUtil.RECORDING_FILE)
        val record = Recorder(this).apply {
            setListener(object : IRecorderListener {
                override fun onUpdateReceived(message: String) {
                    Log.d("foreground", "onUpdateReceived: $message")
                    if (message.contains("done")) {
                        Log.d("foreground", "start translation")
                        whisper.setFilePath(waveFilePath)
                        whisper.setAction(Whisper.ACTION_TRANSCRIBE)
                        whisper.start()
                    }
                    Log.d(
                        "foreground",
                        "${message.contains("done")} $message ${Whisper.MSG_PROCESSING_DONE}"
                    )
                }

                override fun onDataReceived(samples: FloatArray?) {
                    Log.d("foreground", "onDataReceived: $samples")
                }

            })
        }
        record.setFilePath(waveFilePath)
        record.start()


        CoroutineScope(Dispatchers.IO).launch {
            while (true) {
                delay(500)
                if (!record.isInProgress()) {
                    Log.d("foreground", "Recorder stopped. Restarting background recorder...")
                    isForegroundRecording.value = true
                    updateForegroundRecordingSignal(false)
                }
            }
        }
    }

    private fun sendGenieCommandToRover(command: String, addr: String, port: String) {
        val TAG = "sendGenieCommandToRover"
        val textToSend = command.trim()

        if (textToSend.isBlank()) {
            Log.e(TAG, "Empty command received, not sending.")
            return
        }

        Log.d(TAG, "Sending dynamic command to rover: $textToSend")

        CoroutineScope(Dispatchers.IO).launch {
            try {
                mainViewModel.setRoverState("Executing")
                val response = sendMessage(addr, port, textToSend)
                if (response.isSuccessful) {
                    Log.d(TAG, "Command executed successfully: $textToSend")
                } else {
                    Log.e(TAG, "Failed to execute command: ${response.errorBody()?.string()}")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error sending command: ${e.message}", e)
            }
        }
    }

    private fun updateForegroundRecordingSignal(isRecording: Boolean) {
        val intent = Intent("com.chatgptlite.wanted.ACTION_FOREGROUND_RECORDING").apply {
            putExtra("isForegroundRecording", isRecording)
        }
        sendBroadcast(intent)
    }

    @Throws(IOException::class, NullPointerException::class)
    fun copyAssetsDir(inputAssetRelPath: String?, outputPath: String?) {
        val outputAssetPath = File(Paths.get(outputPath, inputAssetRelPath).toString())

        val subAssetList = this.assets.list(inputAssetRelPath!!)
        if (subAssetList!!.size == 0) {
            if (!outputAssetPath.exists()) {
                copyFile(inputAssetRelPath, outputAssetPath)
            }
            return
        }

        if (!outputAssetPath.exists()) {
            outputAssetPath.mkdirs()
        }
        for (subAssetName in subAssetList) {
            val input_sub_asset_path = Paths.get(inputAssetRelPath, subAssetName).toString()
            copyAssetsDir(input_sub_asset_path, outputPath)
        }
    }

    @Throws(IOException::class)
    fun copyFile(inputFilePath: String?, outputAssetFile: File?) {
        val `in` = this.assets.open(inputFilePath!!)
        val out: OutputStream = FileOutputStream(outputAssetFile)

        val buffer = ByteArray(1024 * 1024)
        var read: Int
        while ((`in`.read(buffer).also { read = it }) != -1) {
            out.write(buffer, 0, read)
        }
        Log.i("chatbackend", "copied from" + inputFilePath)
    }

    /**
     * Cheap launch-time check: every bin listed in the model manifest exists
     * and has the expected size. Hash verification is intentionally skipped here —
     * verifying ~5 GB on every cold start would block the UI thread for seconds.
     * The provisioning flow does a full SHA-256 verify after download.
     */
    private fun areAllBinsValid(modelDir: File, modelName: String): Boolean {
        return try {
            val manifest = ModelManifest.load(this, modelName)
            manifest.files.all { f ->
                val file = File(modelDir, f.name)
                file.exists() && file.length() == f.sizeBytes
            }
        } catch (t: Throwable) {
            Log.w(TAG, "Failed to load manifest for '$modelName'; will re-provision", t)
            false
        }
    }

    fun getGenieResponse(prompt: String, onResponse: (String) -> Unit) {
        genieWrapper.getResponseForPrompt(prompt, object : StringCallback {
            override fun onNewString(str: String?) {
                str?.let {
                    Log.d("GenieResponse", "Received token: $it")
                    onResponse(it)
                }
            }
        })
    }


    @OptIn(ExperimentalMaterial3Api::class)
    @SuppressLint("UnusedMaterial3ScaffoldPaddingParameter")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Log.d(TAG, "Device ABI: ${Build.SUPPORTED_ABIS.joinToString()}")

        WindowCompat.setDecorFitsSystemWindows(window, true)

        lateinit var htpExtConfigPath: Path
        try {
            val supportedSocModel = HashMap<String, String>()
            supportedSocModel.putIfAbsent("SM8750", "qualcomm-snapdragon-8-elite.json")
            supportedSocModel.putIfAbsent("SM8650", "qualcomm-snapdragon-8-gen3.json")
            supportedSocModel.putIfAbsent("QCS8550", "qualcomm-snapdragon-8-gen2.json")

            val socModel = Build.SOC_MODEL
            if (!supportedSocModel.containsKey(socModel)) {
                val errorMsg =
                    "Unsupported device. Please ensure you have one of the following device to run the ChatApp: $supportedSocModel"
                Log.e("ChatApp", errorMsg)
                Toast.makeText(this, errorMsg, Toast.LENGTH_LONG).show()
                finish()
            }

            // Use externalFilesDir (NOT externalCacheDir): DownloadManager on Android 11+
            // can only write into externalFilesDir or public dirs. Keeping all model
            // artifacts (bins + tokenizer + genie-config + htp_config) under one root.
            val externalDir = getExternalFilesDir(null)!!.absolutePath
            try {
                copyAssetsDir("models", externalDir.toString())
                copyAssetsDir("htp_config", externalDir.toString())
            } catch (e: IOException) {
                val errorMsg = "Error during copying model asset to external storage: $e"
                Log.e("ChatApp", errorMsg)
                Toast.makeText(this, errorMsg, Toast.LENGTH_SHORT).show()
                finish()
            }
            htpExtConfigPath = Paths.get(
                externalDir, "htp_config",
                supportedSocModel[socModel]
            )
        } catch (e: java.lang.Exception) {
            val errorMsg = "Unexpected error occurred while running ChatApp:$e"
            Log.e("ChatApp", errorMsg)
            Toast.makeText(this, errorMsg, Toast.LENGTH_LONG).show()
            finish()
        }


        val messages = ArrayList<ChatMessage>(1000)

        val cWelcomeMessage = "Hi! How can I help you?"
        val cConversationActivityKeyHtpConfig = htpExtConfigPath.toString()
        val cConversationActivityKeyModelName = "qwen2_5_7b_instruct"

        // ----- Model bin presence gate -----
        // If any required .bin is missing or corrupt, hand off to ProvisioningActivity.
        // It downloads from GitHub Releases over Wi-Fi, verifies SHA-256, then finishes;
        // we recreate() on success and re-enter this block with all bins valid.
        val externalFilesBase = getExternalFilesDir(null)!!.absolutePath
        val modelDirFile = File(Paths.get(externalFilesBase, "models", cConversationActivityKeyModelName).toString())
        if (!areAllBinsValid(modelDirFile, cConversationActivityKeyModelName)) {
            val intent = Intent(this, ProvisioningActivity::class.java).apply {
                putExtra(ProvisioningActivity.EXTRA_MODEL_NAME, cConversationActivityKeyModelName)
            }
            provisioningLauncher.launch(intent)
            return // Stop onCreate; we'll recreate() once provisioning returns OK.
        }

        try {
            val nativeLibPath = applicationContext.applicationInfo.nativeLibraryDir
            Os.setenv("ADSP_LIBRARY_PATH", nativeLibPath, true)
            Os.setenv("LD_LIBRARY_PATH", nativeLibPath, true)

            val htpExtensionsDir = cConversationActivityKeyHtpConfig
            val modelName = cConversationActivityKeyModelName
            val modelDir = modelDirFile.absolutePath

            genieWrapper = GenieWrapper(modelDir, htpExtensionsDir)
            Log.i("Chatbackend", "$modelName Loaded.")
        } catch (e: java.lang.Exception) {
            Log.e("ChatApp", "Error during conversation with Chatbot: $e")
            Toast.makeText(this, "Unexpected error observed. Exiting app.", Toast.LENGTH_SHORT)
                .show()
            finish()
        }

        setContentView(
            ComposeView(this).apply {
                consumeWindowInsets = false
                setContent {
                    val navController = rememberNavController()
                    val currentBackStack by navController.currentBackStackEntryFlow.collectAsState(initial = null)
                    val currentRoute = currentBackStack?.destination?.route ?: NavRoute.VIDEO_STREAM

                    // Activity-scoped ViewModels — survive page switches
                    val controllerViewModel: VideoCamSettingsViewModel = viewModel()
                    val statusViewModel: RoverSettingsViewModel = viewModel()

                    // Initialize all feeds at startup — Controller and Status both pre-connect
                    LaunchedEffect(Unit) {
                        controllerViewModel.linkStatusViewModel(statusViewModel)
                        controllerViewModel.initFeedsOnce()
                        statusViewModel.initFeedsOnce(mainViewModel)
                    }

                    // Main pages are Controller and Status
                    val mainPages = listOf(NavRoute.VIDEO_STREAM, NavRoute.ROVER_SETTINGS)
                    val isMainPage = currentRoute in mainPages
                    val isSubPage = !isMainPage

                    val currentTitle = when (currentRoute) {
                        NavRoute.ROVER_SETTINGS -> "Status"
                        NavRoute.VIDEO_STREAM -> "Controller"
                        NavRoute.TERMINAL -> "Terminal"
                        NavRoute.PROMPT_SETTINGS -> "Prompt Settings"
                        NavRoute.TELEMETRY -> "Telemetry"
                        NavRoute.OCCUPANCY -> "Occupancy Map"
                        NavRoute.NETWORK_CONFIG -> "Network Config"
                        else -> ""
                    }

                    // Advance panel state
                    var advancePanelOpen by remember { mutableStateOf(false) }

                    // Occupancy map popup state (shared across pages)
                    var occPopupVisible by remember { mutableStateOf(false) }

                    val scope = rememberCoroutineScope()
                    val focusManager = LocalFocusManager.current

                    val darkTheme = remember(key1 = "darkTheme") {
                        mutableStateOf(true)
                    }

                    ChatGPTLiteTheme(darkTheme.value) {
                        Box(modifier = Modifier.fillMaxSize()) {
                            val animationFrames = listOf(
                                R.drawable.mic_img_1,
                                R.drawable.mic_img_2,
                                R.drawable.mic_img_3,
                                R.drawable.mic_img_4,
                                R.drawable.mic_img_5,
                                R.drawable.mic_img_6,
                                R.drawable.mic_img_7,
                                R.drawable.mic_img_8,
                                R.drawable.mic_img_9,
                                R.drawable.mic_img_10,
                                R.drawable.mic_img_11,
                                R.drawable.mic_img_12
                            )

                            Scaffold(
                                topBar = {
                                    CenterAlignedTopAppBar(
                                        title = {
                                            Text(
                                                text = currentTitle,
                                                fontWeight = FontWeight.Bold,
                                                textAlign = TextAlign.Center,
                                                color = MaterialTheme.colorScheme.primary
                                            )
                                        },
                                        navigationIcon = {
                                            if (isSubPage) {
                                                IconButton(onClick = { navController.navigateUp() }) {
                                                    Icon(
                                                        imageVector = Icons.Default.ArrowBack,
                                                        contentDescription = "Back",
                                                        tint = MaterialTheme.colorScheme.primary
                                                    )
                                                }
                                            } else {
                                                // Battery indicator from real data
                                                val batteryPct by statusViewModel.battery.collectAsState()
                                                BatteryIndicator(batteryPct)
                                            }
                                        },
                                        actions = {
                                            if (isMainPage) {
                                                // Hamburger menu to toggle advance overlay
                                                IconButton(onClick = {
                                                    advancePanelOpen = !advancePanelOpen
                                                }) {
                                                    Icon(
                                                        imageVector = Icons.Default.Menu,
                                                        contentDescription = "Advance Menu",
                                                        tint = MaterialTheme.colorScheme.primary
                                                    )
                                                }
                                            }
                                        },
                                        modifier = Modifier
                                            .height(80.dp)
                                            .padding(top = 6.dp),
                                        colors = TopAppBarDefaults.topAppBarColors(
                                            containerColor = MaterialTheme.colorScheme.background
                                        )
                                    )
                                },
                                bottomBar = {
                                    BottomNavigationBar(
                                        navController = navController,
                                        currentRoute = currentRoute,
                                        onMicClick = {
                                            Log.d("MicButton", "Mic clicked!")
                                            startRecorder()
                                            micVisibleState.value = true
                                        }
                                    )
                                }
                            ) { innerPadding ->
                                Surface(
                                    color = MaterialTheme.colorScheme.background,
                                    modifier = Modifier.padding(innerPadding)
                                ) {
                                    NavHost(
                                        navController = navController,
                                        startDestination = NavRoute.VIDEO_STREAM
                                    ) {
                                        composable(NavRoute.VIDEO_STREAM) {
                                            VideoStreamingSetting(
                                                controllerViewModel,
                                                onBackPressed = { navController.navigateUp() },
                                                onOccMapClick = { occPopupVisible = true }
                                            )
                                        }
                                        composable(NavRoute.ROVER_SETTINGS) {
                                            SettingsScreen(
                                                mainViewModel = mainViewModel,
                                                viewModel = statusViewModel,
                                                onBackPressed = { navController.navigateUp() },
                                                onOccMapClick = { occPopupVisible = true }
                                            )
                                        }
                                        composable(NavRoute.TERMINAL) {
                                            TerminalScreen(
                                                viewModel = viewModel<TerminalViewModel>(),
                                                onBackPressed = { navController.navigateUp() }
                                            )
                                        }
                                        composable(NavRoute.PROMPT_SETTINGS) {
                                            PromptSettingPage(
                                                onBackPressed = { navController.navigateUp() }
                                            )
                                        }
                                        composable(NavRoute.TELEMETRY) {
                                            TelemetryScreen(
                                                viewModel = viewModel(),
                                                onBackPressed = { navController.navigateUp() }
                                            )
                                        }
                                        composable(NavRoute.OCCUPANCY) {
                                            Occupancy(
                                                viewModel = viewModel<OccupancyViewModel>(),
                                                onBackPressed = { navController.navigateUp() }
                                            )
                                        }
                                        composable(NavRoute.NETWORK_CONFIG) {
                                            NetworkConfigScreen(
                                                onBackPressed = { navController.navigateUp() }
                                            )
                                        }
                                    }
                                }
                            }

                            // ===== Advance Overlay Panel =====
                            if (advancePanelOpen) {
                                // Scrim / backdrop
                                Box(
                                    modifier = Modifier
                                        .fillMaxSize()
                                        .background(Color.Black.copy(alpha = 0.4f))
                                        .clickable { advancePanelOpen = false }
                                        .zIndex(14f)
                                )
                                // Panel
                                AdvanceOverlayPanel(
                                    onDismiss = { advancePanelOpen = false },
                                    onItemClick = { route ->
                                        advancePanelOpen = false
                                        navController.navigate(route) {
                                            launchSingleTop = true
                                        }
                                    },
                                    modifier = Modifier
                                        .align(Alignment.TopEnd)
                                        .padding(top = 56.dp, end = 12.dp)
                                        .zIndex(15f)
                                )
                            }

                            // ===== Occupancy Map Popup =====
                            if (occPopupVisible) {
                                OccupancyPopup(
                                    bitmap = statusViewModel.occupancyBitmap.value
                                        ?: controllerViewModel.occupancyBitmap.value,
                                    onDismiss = { occPopupVisible = false }
                                )
                            }

                            // ===== MicPopup =====
                            if (micVisibleState.value) {
                                MicPopup(
                                    text = text,
                                    genieResponse = genieResponse,
                                    animationFrames = animationFrames,
                                    onConfirm = {
                                        val addr = "10.0.0.1"
                                        val port = "8000"
                                        sendGenieCommandToRover(genieResponse.value, addr, port)
                                        text.value = ""
                                        genieResponse.value = ""
                                        micVisibleState.value = false
                                        isForegroundRecording.value = false
                                        updateForegroundRecordingSignal(false)
                                        voiceWakeupManager.resumeWakeWordDetection()
                                    },
                                    onCancel = {
                                        text.value = ""
                                        genieResponse.value = ""
                                        micVisibleState.value = false
                                        isForegroundRecording.value = false
                                        updateForegroundRecordingSignal(false)
                                        voiceWakeupManager.resumeWakeWordDetection()
                                    }
                                )
                            }
                        }
                    }
                }
            }
        )

        voiceWakeupManager = VoiceWakeupManager(this) {
            Log.d(TAG, "Wake word callback triggered, starting recorder...")
            startRecorder()
        }
        checkAndRequestPermissions()
    }

    private fun checkAndRequestPermissions() {
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.RECORD_AUDIO
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.RECORD_AUDIO),
                REQUEST_RECORD_AUDIO
            )
        } else {
            startVoiceWakeup()
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (requestCode == REQUEST_RECORD_AUDIO) {
            if (grantResults.isNotEmpty() &&
                grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startVoiceWakeup()
            } else {
                Toast.makeText(
                    this,
                    "Microphone permission required for voice wakeup",
                    Toast.LENGTH_LONG
                ).show()
            }
        }
    }

    private fun startVoiceWakeup() {
        voiceWakeupManager.startWakeWordDetection()
    }

    override fun onDestroy() {
        super.onDestroy()
        // Guard: if onCreate early-returned (e.g. waiting for ProvisioningActivity),
        // voiceWakeupManager was never assigned and accessing it would throw.
        if (::voiceWakeupManager.isInitialized) {
            voiceWakeupManager.stop()
        }
    }
}


// ==================== Bottom Navigation (2 tabs + mic FAB) ====================

@Composable
fun BottomNavigationBar(
    navController: NavHostController,
    currentRoute: String,
    onMicClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(80.dp)
    ) {
        NavigationBar(
            containerColor = MaterialTheme.colorScheme.background,
            tonalElevation = 0.dp,
            modifier = Modifier.align(Alignment.BottomCenter)
        ) {
            // Controller tab
            val controllerSelected = currentRoute == NavRoute.VIDEO_STREAM
            NavigationBarItem(
                selected = controllerSelected,
                onClick = {
                    if (!controllerSelected) {
                        navController.navigate(NavRoute.VIDEO_STREAM) {
                            launchSingleTop = true
                            restoreState = true
                        }
                    }
                },
                icon = {
                    Icon(
                        imageVector = Icons.Default.ControlCamera,
                        contentDescription = "Controller",
                        tint = if (controllerSelected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                },
                label = {
                    Text(
                        text = "Controller",
                        color = if (controllerSelected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                },
                colors = NavigationBarItemDefaults.colors(
                    indicatorColor = MaterialTheme.colorScheme.background
                )
            )

            // Spacer for mic button
            Spacer(modifier = Modifier.weight(1f))

            // Status tab
            val statusSelected = currentRoute == NavRoute.ROVER_SETTINGS
            NavigationBarItem(
                selected = statusSelected,
                onClick = {
                    if (!statusSelected) {
                        navController.navigate(NavRoute.ROVER_SETTINGS) {
                            launchSingleTop = true
                            restoreState = true
                        }
                    }
                },
                icon = {
                    Icon(
                        imageVector = Icons.Default.Home,
                        contentDescription = "Status",
                        tint = if (statusSelected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                },
                label = {
                    Text(
                        text = "Status",
                        color = if (statusSelected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                },
                colors = NavigationBarItemDefaults.colors(
                    indicatorColor = MaterialTheme.colorScheme.background
                )
            )
        }

        // Floating mic button in center
        Box(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .offset(y = (-24).dp)
        ) {
            MicButton(onMicClick = onMicClick)
        }
    }
}


// ==================== Battery Indicator ====================

@Composable
fun BatteryIndicator(batteryPct: Int? = null) {
    val color = when {
        batteryPct == null -> Color(0xFF888888)     // No data yet
        batteryPct >= 50 -> MaterialTheme.colorScheme.primary
        batteryPct >= 25 -> Color(0xFFFFEB3B)
        batteryPct > 0 -> Color(0xFFF44336)
        else -> Color(0xFF888888)                   // 0% — critical
    }
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier.padding(start = 8.dp)
    ) {
        Icon(
            imageVector = Icons.Default.BatteryFull,
            contentDescription = "Battery",
            tint = color,
            modifier = Modifier.size(20.dp)
        )
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = if (batteryPct != null) "$batteryPct%" else "--",
            color = color,
            fontSize = 11.sp,
            fontWeight = FontWeight.SemiBold
        )
    }
}


// ==================== Advance Overlay Panel ====================

data class AdvanceItem(
    val title: String,
    val description: String,
    val route: String
)

@Composable
fun AdvanceOverlayPanel(
    onDismiss: () -> Unit,
    onItemClick: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    val items = listOf(
        AdvanceItem("Terminal", "Send command directly to Rover", NavRoute.TERMINAL),
        AdvanceItem("Prompt Settings", "Configure system prompt for AI", NavRoute.PROMPT_SETTINGS)
    )

    Card(
        modifier = modifier.width(280.dp),
        shape = RoundedCornerShape(12.dp),
        colors = CardDefaults.cardColors(
            containerColor = Color(0xFF1A1C1B)
        ),
        border = BorderStroke(1.dp, MaterialTheme.colorScheme.primary.copy(alpha = 0.25f)),
        elevation = CardDefaults.cardElevation(defaultElevation = 8.dp)
    ) {
        Column {
            // Header
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 16.dp, vertical = 14.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Advance",
                    color = MaterialTheme.colorScheme.primary,
                    fontWeight = FontWeight.Bold,
                    fontSize = 15.sp
                )
                Text(
                    text = "\u00D7",
                    color = Color(0xFF888888),
                    fontSize = 22.sp,
                    modifier = Modifier.clickable { onDismiss() }
                )
            }

            Divider(color = Color.White.copy(alpha = 0.08f), thickness = 1.dp)

            // Items
            Column(
                modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                items.forEach { item ->
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .border(
                                1.dp,
                                MaterialTheme.colorScheme.primary,
                                RoundedCornerShape(8.dp)
                            )
                            .clickable { onItemClick(item.route) }
                            .padding(12.dp)
                    ) {
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Column(modifier = Modifier.weight(1f)) {
                                Text(
                                    text = item.title,
                                    color = MaterialTheme.colorScheme.primary,
                                    fontWeight = FontWeight.Bold,
                                    fontSize = 13.sp
                                )
                                Text(
                                    text = item.description,
                                    color = Color(0xFFAAAAAA),
                                    fontSize = 11.sp
                                )
                            }
                            Text(
                                text = "\u2192",
                                color = MaterialTheme.colorScheme.primary,
                                fontSize = 20.sp
                            )
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(8.dp))
        }
    }
}


// ==================== Occupancy Map Popup ====================

@Composable
fun OccupancyPopup(bitmap: android.graphics.Bitmap?, onDismiss: () -> Unit) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.85f))
            .clickable { onDismiss() }
            .zIndex(20f),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Box(
                modifier = Modifier
                    .fillMaxWidth(0.9f)
                    .aspectRatio(1f)
                    .border(
                        2.dp,
                        MaterialTheme.colorScheme.primary.copy(alpha = 0.4f),
                        RoundedCornerShape(12.dp)
                    )
                    .clip(RoundedCornerShape(12.dp))
                    .background(Color.Black),
                contentAlignment = Alignment.Center
            ) {
                if (bitmap != null) {
                    Image(
                        bitmap = bitmap.asImageBitmap(),
                        contentDescription = "Occupancy Map",
                        modifier = Modifier.fillMaxSize()
                    )
                } else {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text(text = "\uD83D\uDDFA", fontSize = 48.sp)
                        Spacer(modifier = Modifier.height(8.dp))
                        Text(
                            text = "Loading occupancy map...",
                            color = Color(0xFF888888),
                            fontSize = 14.sp
                        )
                    }
                }
            }

            Spacer(modifier = Modifier.height(12.dp))
            Text(
                text = "Tap anywhere to close",
                color = Color(0xFF666666),
                fontSize = 11.sp
            )
        }
    }
}


// ==================== Mic Button ====================

@Composable
fun MicButton(
    onMicClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .size(72.dp)
            .background(
                color = MaterialTheme.colorScheme.secondary,
                shape = RoundedCornerShape(50)
            )
            .border(
                width = 2.dp,
                color = MaterialTheme.colorScheme.primary,
                shape = RoundedCornerShape(50)
            )
            .clickable {
                onMicClick()
            },
        contentAlignment = Alignment.Center
    ) {
        Icon(
            imageVector = Icons.Filled.Mic,
            contentDescription = "Voice Input",
            tint = MaterialTheme.colorScheme.primary
        )
    }
}


// ==================== Mic Popup ====================

@Composable
fun MicPopup(
    text: MutableState<String>,
    genieResponse: MutableState<String>,
    animationFrames: List<Int>,
    onConfirm: () -> Unit,
    onCancel: () -> Unit
) {
    val currentFrameIndex = remember { mutableStateOf(0) }
    val showConfirmButtons = remember { mutableStateOf(false) }

    LaunchedEffect(Unit) {
        while (true) {
            delay(200)
            currentFrameIndex.value = (currentFrameIndex.value + 1) % animationFrames.size
        }
    }

    LaunchedEffect(genieResponse.value) {
        if (genieResponse.value.contains("sorry", ignoreCase = true)) {
            delay(3000)
            text.value = ""
            genieResponse.value = ""
            onCancel()
        }
    }

    LaunchedEffect(genieResponse.value) {
        if (genieResponse.value.isBlank() || genieResponse.value.contains("sorry", ignoreCase = true)) return@LaunchedEffect

        val lastSnapshot = genieResponse.value
        delay(500)
        if (genieResponse.value == lastSnapshot) {
            showConfirmButtons.value = true
        }
    }

    LaunchedEffect(text.value) {
        if (genieResponse.value.contains("sorry", ignoreCase = true)){
            delay(3000)
            text.value = ""
            genieResponse.value = ""
            onCancel()
        }
    }

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .fillMaxHeight(1f)
            .background(
                brush = Brush.verticalGradient(
                    colors = listOf(
                        MaterialTheme.colorScheme.background.copy(alpha = 0f),
                        MaterialTheme.colorScheme.background.copy(alpha = 0.7f),
                        MaterialTheme.colorScheme.background.copy(alpha = 1f)
                    )
                )
            ),
        contentAlignment = Alignment.Center
    ) {
        Box(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 20.dp)
        ) {
            ListeningAnimation(animationFrames, currentFrameIndex.value)
        }
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .align(Alignment.BottomCenter)
                .padding(bottom = 190.dp),
            verticalArrangement = Arrangement.Bottom
        ) {
            if (text.value.isNotEmpty()) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(horizontal = 16.dp)
                ) {
                    MessageBubble(
                        message = text.value,
                        textColor = Color.White,
                        alignment = Alignment.CenterEnd
                    )
                }
                Spacer(modifier = Modifier.height(16.dp))
            }

            if (genieResponse.value.isNotBlank()) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(horizontal = 16.dp)
                ) {
                    MessageBubble(
                        message = if (genieResponse.value.contains("Sorry", ignoreCase = true)) {
                            genieResponse.value
                        } else {
                            "Can you confirm the command:\n${genieResponse.value}"
                        },
                        textColor = MaterialTheme.colorScheme.primary,
                        alignment = Alignment.CenterStart
                    )
                }
                Spacer(modifier = Modifier.height(16.dp))

                if (!genieResponse.value.contains("Sorry", ignoreCase = true)) {
                    if (showConfirmButtons.value) {
                        ConfirmCancelButtons(
                            onConfirm = onConfirm,
                            onCancel = onCancel
                        )
                    }
                }
            }
        }
    }
}

@Composable
fun MessageBubble(
    message: String,
    textColor: Color,
    alignment: Alignment
) {
    Box(
        modifier = Modifier.fillMaxWidth()
    ) {
        Box(
            modifier = Modifier
                .widthIn(max = 280.dp)
                .align(alignment)
                .background(
                    color = Color.Black,
                    shape = RoundedCornerShape(24.dp)
                )
                .border(
                    1.dp,
                    if (textColor == Color.White) Color.White else MaterialTheme.colorScheme.primary,
                    RoundedCornerShape(24.dp)
                )
                .padding(12.dp)
        ) {
            Text(
                text = message,
                style = MaterialTheme.typography.bodyMedium,
                color = textColor,
                textAlign = TextAlign.Start
            )
        }
    }
}


@Composable
private fun ConfirmCancelButtons(
    onConfirm: () -> Unit,
    onCancel: () -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.Center
    ) {
        ConfirmButton(text = "Cancel", onClick = onCancel)
        Spacer(modifier = Modifier.width(32.dp))
        ConfirmButton(text = "Confirm", onClick = onConfirm)
    }
}

@Composable
private fun ConfirmButton(text: String, onClick: () -> Unit) {
    Button(
        onClick = onClick,
        colors = ButtonDefaults.buttonColors(containerColor = Color.Black),
        border = BorderStroke(0.5.dp, MaterialTheme.colorScheme.primary),
        shape = RoundedCornerShape(24.dp),
        modifier = Modifier.width(120.dp),
        contentPadding = PaddingValues(0.dp)
    ) {
        Text(
            text,
            color = MaterialTheme.colorScheme.primary,
            fontSize = 16.sp,
            modifier = Modifier.padding(vertical = 8.dp)
        )
    }
}

@Composable
private fun ListeningAnimation(
    animationFrames: List<Int>,
    frameIndex: Int
) {
    Image(
        painter = painterResource(id = animationFrames[frameIndex]),
        contentDescription = "Listening Animation",
        modifier = Modifier
            .size(150.dp)
            .background(Color.Transparent)
    )
}
