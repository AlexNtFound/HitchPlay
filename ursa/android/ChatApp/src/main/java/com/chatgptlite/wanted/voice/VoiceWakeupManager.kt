package com.chatgptlite.wanted

import ai.picovoice.porcupine.*
import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.util.Log
import android.widget.Toast


class VoiceWakeupManager(
    private val context: Context,
    private val onWakeWordDetected: () -> Unit  // Callback when wake word is detected
) {

    private var porcupineManager: PorcupineManager? = null
    private var speechRecognizer: SpeechRecognizer? = null
    private var isListening = false

    companion object {
        private const val TAG = "VoiceWakeup"
    }

    fun startWakeWordDetection() {
        if (isListening) return

        val accessKey = "YOUR ACCESS KEY HERE"; // AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)
        val keywordPath = "Hey-Ursa_en_android_v3_0_0.ppn"; // path relative to 'assets' folder

        try {
            // Use Porcupine to listen for wake word
            val porcupineManager = PorcupineManager.Builder()
                .setAccessKey(accessKey)
                .setKeywordPath(keywordPath)
//                .setKeyword(Porcupine.BuiltInKeyword.PORCUPINE)
                .setSensitivity(0.5f)
                .build(context) { // detection event logic/callback
                    handleWakeWordDetected()
                }

            porcupineManager?.start()
            isListening = true
            Log.d(TAG, "Voice wake word detection started")
            Toast.makeText(context, "Voice wakeup enabled, say 'Hey Ursa'", Toast.LENGTH_SHORT).show()

        } catch (e: PorcupineException) {
            Log.e(TAG, "Porcupine initialization failed: ${e.message}")
            // Fallback solution - continuous recognition
//            startContinuousRecognition()
        }
    }

    private fun handleWakeWordDetected() {
        Log.d(TAG, "Wake word detected! Triggering recorder...")
        Toast.makeText(context, "Wake word detected!", Toast.LENGTH_SHORT).show()

        // Stop wake word detection temporarily
        porcupineManager?.stop()

        // Call the callback to trigger MainActivity's startRecorder()
        onWakeWordDetected()
    }

    fun resumeWakeWordDetection() {
        try {
            porcupineManager?.start()
            Log.d(TAG, "Wake word detection resumed")
        } catch (e: Exception) {
            Log.e(TAG, "Resume failed: ${e.message}")
            // Try to restart continuous recognition if Porcupine fails
            if (speechRecognizer != null) {
                restartContinuousRecognition()
            }
        }
    }

    private fun startContinuousRecognition() {
        Log.d(TAG, "Using continuous speech recognition mode for wake word detection")
        Toast.makeText(context, "Using continuous listening mode", Toast.LENGTH_SHORT).show()

        speechRecognizer = SpeechRecognizer.createSpeechRecognizer(context)
        isListening = true

        val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
            putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
            putExtra(RecognizerIntent.EXTRA_LANGUAGE, "en-US")
            putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true)
        }

        speechRecognizer?.setRecognitionListener(object : RecognitionListener {
            override fun onResults(results: Bundle?) {
                val text = results?.getStringArrayList(
                    SpeechRecognizer.RESULTS_RECOGNITION
                )?.firstOrNull() ?: ""

                Log.d(TAG, "Continuous recognition heard: $text")

                if (text.contains("hey google", ignoreCase = true)) {
                    handleWakeWordDetected()
                } else {
                    // Continue listening for wake word
                    android.os.Handler(android.os.Looper.getMainLooper()).postDelayed({
                        speechRecognizer?.startListening(intent)
                    }, 500)
                }
            }

            override fun onError(error: Int) {
                val errorMsg = when(error) {
                    SpeechRecognizer.ERROR_NO_MATCH -> "No speech detected"
                    SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> "Speech timeout"
                    SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> "Missing permissions"
                    SpeechRecognizer.ERROR_AUDIO -> "Audio error"
                    SpeechRecognizer.ERROR_LANGUAGE_NOT_SUPPORTED -> "Language not supported"
                    SpeechRecognizer.ERROR_CLIENT -> "Client"
                    SpeechRecognizer.ERROR_CANNOT_CHECK_SUPPORT -> "Cannot check support"
                    SpeechRecognizer.ERROR_CANNOT_LISTEN_TO_DOWNLOAD_EVENTS -> "Cannot listen to download events"
                    SpeechRecognizer.ERROR_LANGUAGE_UNAVAILABLE -> "Language unavailable"
                    SpeechRecognizer.ERROR_NETWORK -> "Network error"
                    SpeechRecognizer.ERROR_NETWORK_TIMEOUT -> "Network timeout"
                    else -> "Error: $error"
                }
                // Only log errors that aren't timeout/no-match
                if (error != SpeechRecognizer.ERROR_NO_MATCH &&
                    error != SpeechRecognizer.ERROR_SPEECH_TIMEOUT) {
                    Log.w(TAG, "Recognition error: $errorMsg")
                }
                // Restart listening with a delay to prevent spam
                android.os.Handler(android.os.Looper.getMainLooper()).postDelayed({
                    if (isListening) {
                        speechRecognizer?.startListening(intent)
                    }
                }, 1000)
            }

            override fun onReadyForSpeech(params: Bundle?) {
                // Don't log this anymore to reduce spam
            }

            override fun onBeginningOfSpeech() {}
            override fun onRmsChanged(rmsdB: Float) {}
            override fun onBufferReceived(buffer: ByteArray?) {}
            override fun onEndOfSpeech() {}
            override fun onPartialResults(partialResults: Bundle?) {
                val text = partialResults?.getStringArrayList(
                    SpeechRecognizer.RESULTS_RECOGNITION
                )?.firstOrNull() ?: ""

                if (text.contains("hey google", ignoreCase = true)) {
                    Log.d(TAG, "Wake word detected in partial results!")
                    speechRecognizer?.cancel() // Stop current recognition
                    handleWakeWordDetected()
                }
            }
            override fun onEvent(eventType: Int, params: Bundle?) {}
        })

        speechRecognizer?.startListening(intent)
    }

    private fun restartContinuousRecognition() {
        speechRecognizer?.destroy()
        speechRecognizer = null
        startContinuousRecognition()
    }

    fun stop() {
        isListening = false
        porcupineManager?.stop()
        porcupineManager?.delete()
        speechRecognizer?.destroy()
        speechRecognizer = null
        Log.d(TAG, "Voice wakeup stopped")
    }
}