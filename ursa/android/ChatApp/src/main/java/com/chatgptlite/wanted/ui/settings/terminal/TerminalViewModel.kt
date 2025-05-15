package com.chatgptlite.wanted.ui.settings.terminal

import android.app.Application
import android.content.Context
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.chatgptlite.wanted.ui.settings.rover.RoverConfig
import com.chatgptlite.wanted.helpers.sendMessage
import dagger.hilt.android.internal.Contexts.getApplication
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import okhttp3.OkHttpClient
import okhttp3.WebSocket
import java.net.URLEncoder
import java.nio.charset.StandardCharsets

class TerminalViewModel(application: Application) : AndroidViewModel(application) {
    // Placeholder for any future logic or state for the Advance screen
    private val DEFAULT_IPADDRESS = "10.0.0.120"
    private val DEFAULT_PORT = "8000"
    private val CONCAT = "ros2 topic echo "

    private val webSockets = mutableListOf<WebSocket>()
    private val client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.120"
    private val WEBSOCKET_PORT = "9090"

    private val _pingResult = MutableStateFlow<String?>(null)
    val pingResult: StateFlow<String?> = _pingResult.asStateFlow()

    private val _messageResult = MutableStateFlow<String?>(null)
    val messageResult: StateFlow<String?> = _messageResult.asStateFlow()

    fun stopWebSocket(topic: String) {
        val iterator = webSockets.iterator()
        while (iterator.hasNext()) {
            val webSocket = iterator.next()
            if (webSocket.request().url.toString().contains(topic)) {
                webSocket.close(1000, "Closed connection for $topic")
                iterator.remove() // Remove from list after closing
            }
        }
    }

    fun closeAllConnections() {
        webSockets.forEach { it.close(1000, "Closed all connections") }
        webSockets.clear()
        client.dispatcher.executorService.shutdown()
    }

    override fun onCleared() {
        super.onCleared()
        closeAllConnections() // Ensure WebSocket is closed when ViewModel is cleared
        client.dispatcher.executorService.shutdown()
    }

    fun sendPing(ipAddress: String) {
        viewModelScope.launch {
            try {
                Log.d("Terminal", "launching sendPing function")
                val (ip, port) = ipAddress.split(":")
                val response = com.chatgptlite.wanted.helpers.sendPing(ip, port)
                if (response.isSuccessful) {
                    Log.d("Terminal", "sendPing successful")
                    _pingResult.value = "Ping successful"
                } else {
                    Log.d("Terminal", "sendPing has error")
                    _pingResult.value = "Error sending ping: ${response.message()}"
                }
            } catch (e: Exception) {
                Log.d("Terminal", "sendPing has error [${e.message}]")
                _pingResult.value = "Error sending ping: [${e.message}]"
            }
        }
    }

    fun sendMessage(textToSend: String, ipAddress: String? = null, onSuccess: (() -> Unit)? = null, onFail: ((error: String) -> Unit)? = null) {
        val newIpAddress = if (ipAddress == null) {
            val sharedPreferences = getApplication<Application>().getSharedPreferences("RoverSettings", Context.MODE_PRIVATE)
            sharedPreferences.getString("ipAddress", "") ?: return
        } else ipAddress
        viewModelScope.launch {
            try {
                Log.d("Terminal", "launching sendMessage function")
                val (ip, port) = newIpAddress.split(":")
                // url encode the textToSend
                val encodedText = URLEncoder.encode(textToSend, StandardCharsets.UTF_8.toString())
                val response = com.chatgptlite.wanted.helpers.sendMessage(ip, port, encodedText)

                if (response.isSuccessful) {
                    Log.d("Terminal", "sendMessage successful")
                    val statusResponse = response.body()
                    _messageResult.value = "Message sent successfully. Server status: ${statusResponse?.status}. Request: ${statusResponse?.request}"
                    onSuccess?.invoke()
                } else {
                    Log.d("Terminal", "sendMessage failed")
                    val msg = "Error sending message: ${response.message()}"
                    _messageResult.value = msg
                    onFail?.invoke(msg)
                }
            } catch (e: Exception) {
                val msg = "Error sending message: ${e.message}"
                Log.d("Terminal", "sendMessage failed, error [${msg}]")
                _messageResult.value = msg
                e.message?.let { onFail?.invoke(msg) }
            }
        }
    }

    fun saveConfig(ipAddress: String, port: String, textToSend: String) {
        val sharedPreferences = getApplication<Application>().getSharedPreferences("RoverSettings", Context.MODE_PRIVATE)
        with(sharedPreferences.edit()) {
            putString("ipAddress", ipAddress)
            putString("port", port)
            putString("textToSend", textToSend)
            apply()
        }
        Log.i("RoverSettingsViewModel", "Config saved: $ipAddress:$port -> $textToSend")
    }

    fun loadConfig(): RoverConfig? {
        val sharedPreferences = getApplication<Application>().getSharedPreferences("RoverSettings", Context.MODE_PRIVATE)
        val ipAddress = sharedPreferences.getString("ipAddress", null)
        val port = sharedPreferences.getString("port", null)
        val textToSend = sharedPreferences.getString("textToSend", null)

        return if (ipAddress != null && port != null && textToSend != null) {
            RoverConfig(ipAddress, port, textToSend)
        } else {
            null
        }
    }


}

data class AdvanceConfig(
    val ipAddress: String,
    val port: String,
    val textToSend: String
)