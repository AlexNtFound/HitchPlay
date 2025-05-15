package com.chatgptlite.wanted.services

import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.IBinder
import android.util.Log
import com.chatgptlite.wanted.data.whisper.asr.BackgroundRecorder
import com.chatgptlite.wanted.data.whisper.asr.Whisper
import com.chatgptlite.wanted.data.whisper.asr.IRecorderListener
import com.chatgptlite.wanted.data.whisper.asr.IWhisperListener
import com.chatgptlite.wanted.permission.PermissionCheck
import com.chatgptlite.wanted.data.whisper.utils.WaveUtil
import kotlinx.coroutines.*
import androidx.compose.runtime.mutableStateOf
//import com.chatgptlite.wanted.ui.conversations.components.TAG
import java.io.File

import android.Manifest
import android.content.BroadcastReceiver
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.net.Uri
import android.provider.Settings
import androidx.core.content.ContextCompat
import com.chatgptlite.wanted.helpers.PermissionRequestActivity
import java.util.concurrent.atomic.AtomicBoolean


val TAG = "AudioService"

fun getFilePath(context: Context, assetName: String): String {
    val outfile = File(context.filesDir, assetName);
    if (!outfile.exists()) {
        Log.d(TAG, "File not found - " + outfile.absolutePath)
    }

    Log.d(TAG, "Returned asset path: " + outfile.absolutePath)
    return outfile.absolutePath
}



class AudioService : Service() {
    private lateinit var backgroundRecorder: BackgroundRecorder
    private lateinit var backgroundWhisper: Whisper
    private val isRecorderRunning = mutableStateOf(true)
    private val isPermissionGranted = AtomicBoolean(false)
    private val coroutineScope = CoroutineScope(Dispatchers.IO)

    private val keywordDetectedReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action == "com.chatgptlite.wanted.ACTION_KEYWORD_DETECTED") {
                val isKeywordDetected = intent.getBooleanExtra("isKeywordDetected", false)
                if (isKeywordDetected) {
                    pauseBackgroundRecorder()
                } else {
                    restartBackgroundRecorder()
                }
            }
        }
    }



    override fun onCreate() {
        super.onCreate()
        val context = this
        var modelPath = getFilePath(context, "whisper-tiny-en.tflite")
        var vocabPath = getFilePath(context, "filters_vocab_en.bin")

        val filter = IntentFilter("com.chatgptlite.wanted.ACTION_FOREGROUND_RECORDING")
        registerReceiver(foregroundRecordingReceiver, filter)


        // Initialize Background Recorder
        backgroundRecorder = BackgroundRecorder(context).apply {
            setListener(object : IRecorderListener {
                override fun onUpdateReceived(message: String) {
                    Log.d(TAG, "Recorder Update: $message")
                    if (message.contains("done")) {
                        Log.d(TAG, "Background recording completed.")
                        backgroundWhisper.setFilePath(getFilePath(context, WaveUtil.RECORDING_FILE))
                        backgroundWhisper.setAction(Whisper.ACTION_TRANSCRIBE)
                        backgroundWhisper.start()
                    }
                    Log.d(TAG, "${message.contains("done")} $message ${Whisper.MSG_PROCESSING_DONE}")
                }

                override fun onDataReceived(samples: FloatArray?) {
                    Log.d(TAG, "onDataReceived: $samples")
                }
            })
        }

        // Initialize Background Whisper
        backgroundWhisper = Whisper(context).apply {
            loadModel(
                modelPath, vocabPath, false
            )
            setListener(object : IWhisperListener {
                private val keywordVariations = listOf(
                    "ursa", "ursula", "ursor", "mersa", "hersa", "versa", "ozal", "alza", "arza", "rover"
                )

                // Fuzzy matching threshold (0-1)
                private val matchThreshold = 0.7

                override fun onUpdateReceived(message: String?) {
                    Log.d(TAG, "onUpdateReceived: $message")
                }

                override fun onResultReceived(result: String?) {
                    Log.d(TAG, "onResultReceived: $result")
                    // text = TextFieldValue(result ?: "")

                    // Advanced keyword detection with multiple strategies
                    result?.let { transcribedText ->
                        val detectedKeyword = findBestKeywordMatch(transcribedText)

                        if (detectedKeyword != null) {
                            Log.d(TAG, "Keyword detected: $detectedKeyword")
                            sendKeywordDetectedBroadcast(detectedKeyword)
                        }
                    }
                }

                /**
                 * Find the best keyword match using multiple strategies
                 * @param text Transcribed text to search
                 * @return Matched keyword or null if no good match found
                 */
                private fun findBestKeywordMatch(text: String): String? {
                    // Convert to lowercase for case-insensitive matching
                    val lowercaseText = text.lowercase()

                    // 1. Exact match (including near neighbors)
                    keywordVariations.forEach { keyword ->
                        if (lowercaseText.contains(keyword, ignoreCase = true)) {
                            return keyword
                        }
                    }

                    // 2. Fuzzy matching using Levenshtein distance
                    keywordVariations.forEach { keyword ->
                        if (calculateSimilarity(lowercaseText, keyword) >= matchThreshold) {
                            return keyword
                        }
                    }

                    return null
                }

                /**
                 * Calculate similarity between two strings using Levenshtein distance
                 * @param s1 First string
                 * @param s2 Second string
                 * @return Similarity score (0-1)
                 */
                private fun calculateSimilarity(s1: String, s2: String): Double {
                    // Levenshtein distance implementation
                    val m = s1.length
                    val n = s2.length
                    val dp = Array(m + 1) { IntArray(n + 1) }

                    // Initialize first row and column
                    for (i in 0..m) dp[i][0] = i
                    for (j in 0..n) dp[0][j] = j

                    // Fill the matrix
                    for (i in 1..m) {
                        for (j in 1..n) {
                            val cost = if (s1[i - 1] == s2[j - 1]) 0 else 1
                            dp[i][j] = minOf(
                                dp[i - 1][j] + 1,      // Deletion
                                dp[i][j - 1] + 1,      // Insertion
                                dp[i - 1][j - 1] + cost // Substitution
                            )
                        }
                    }

                    // Calculate similarity score
                    val maxLength = maxOf(m, n)
                    val distance = dp[m][n]
                    return 1.0 - (distance.toDouble() / maxLength)
                }
            })
        }


    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // Check and request permissions before starting recorder
        coroutineScope.launch {
            ensureAudioRecordingPermission()
            if (isPermissionGranted.get()) {
                startBackgroundRecorder()
            }
        }
        return START_STICKY
    }

    private suspend fun ensureAudioRecordingPermission() {
        while (!isPermissionGranted.get()) {
            if (ContextCompat.checkSelfPermission(
                    this,
                    Manifest.permission.RECORD_AUDIO
                ) == PackageManager.PERMISSION_GRANTED
            ) {
                isPermissionGranted.set(true)
                Log.d(TAG, "Audio recording permission granted.")
            } else {
                Log.e(TAG, "Audio recording permission not granted. Prompting user...")
                promptPermission()
                delay(5000) // Wait before re-checking permission
            }
        }
    }

    private fun promptPermission() {
        val intent = Intent(this, PermissionRequestActivity::class.java).apply {
            addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
        }
        startActivity(intent)
    }


    private fun startBackgroundRecorder() {
        val waveFilePath = getFilePath(this, WaveUtil.RECORDING_FILE)
        coroutineScope.launch {
            while (isRecorderRunning.value) {
                if (!backgroundRecorder.isInProgress()) {
                    Log.d(TAG, "Background recorder starting...")
                    backgroundRecorder.setFilePath(waveFilePath)
                    backgroundRecorder.start()
                }
                delay(500)
            }
        }
    }


    private fun pauseBackgroundRecorder() {
        isRecorderRunning.value = false
        backgroundRecorder.stop()
        Log.d(TAG, "Background recorder paused.")
    }

    private fun restartBackgroundRecorder() {
        if (!isRecorderRunning.value) {
            isRecorderRunning.value = true
            startBackgroundRecorder()
            Log.d(TAG, "Background recorder restarted.")
        }
    }

    private val foregroundRecordingReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action == "com.chatgptlite.wanted.ACTION_FOREGROUND_RECORDING") {
                val isForegroundRecording = intent.getBooleanExtra("isForegroundRecording", false)
                if (!isForegroundRecording) {
                    Log.d(TAG, "Foreground recording stopped, restarting BackgroundRecorder.")
                    restartBackgroundRecorder()
                } else {
                    Log.d(TAG, "Foreground recording active, pausing BackgroundRecorder.")
                    pauseBackgroundRecorder()
                }
            }
        }
    }

    private fun sendKeywordDetectedBroadcast(detectedKeyword: String) {
        val intent = Intent("com.chatgptlite.wanted.ACTION_KEYWORD_DETECTED").apply {
            putExtra("keyword", detectedKeyword)
        }
        sendBroadcast(intent)
    }



    companion object {
        fun start(context: Context) {
            Log.d(TAG, "Starting AudioService...")
            val intent = Intent(context, AudioService::class.java)
            context.startService(intent)
//            ContextCompat.startForegroundService(context, intent)
        }

//        fun stop(context: Context) {
//            Log.d(TAG, "Stopping AudioService...")
//            val intent = Intent(context, AudioService::class.java)
//            context.stopService(intent)
//        }
    }

    override fun onDestroy() {
        super.onDestroy()
        coroutineScope.cancel()
        backgroundRecorder.stop()
    }

    override fun onBind(intent: Intent?): IBinder? {
        return null
    }
}