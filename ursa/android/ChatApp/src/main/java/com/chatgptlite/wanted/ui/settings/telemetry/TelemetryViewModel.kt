package com.chatgptlite.wanted.ui.settings.rover

import android.app.Application
import android.content.Context
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.chatgptlite.wanted.helpers.sendMessage
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.net.URLEncoder
import java.nio.charset.StandardCharsets
import okhttp3.OkHttpClient
import okhttp3.WebSocket
import okhttp3.Request

import kotlinx.coroutines.*
import com.chatgptlite.wanted.helpers.RoverWebSocketListener
import org.json.JSONException
import org.json.JSONObject

class TelemetryViewModel(application: Application) : AndroidViewModel(application) {

    // MutableState for x, y, z coordinates, velocity, and heading
    var xCoordinate = MutableStateFlow<String?>("0")
    var yCoordinate = MutableStateFlow<String?>("0")
    var zCoordinate = MutableStateFlow<String?>("0")
    var velocity = MutableStateFlow<String?>("-1000")
    var heading = MutableStateFlow<String?>("0")
    var battery = MutableStateFlow<String?>("0")
    private val DEFAULT_IPADDRESS = "10.0.0.1"
    private val DEFAULT_PORT = "8000"
    private val CONCAT = "ros2 topic echo "

    private var velocityJob: Job? = null

    private val webSockets = mutableListOf<WebSocket>()
    private val client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.1"
    private val WEBSOCKET_PORT = "9090"

    private val _pingResult = MutableStateFlow<String?>(null)
    val pingResult: StateFlow<String?> = _pingResult.asStateFlow()

    private val _messageResult = MutableStateFlow<String?>(null)
    val messageResult: StateFlow<String?> = _messageResult.asStateFlow()

    fun startVelocityWebSocket() {
        val topic = "/cmd_vel"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener = RoverWebSocketListener(topic) { message ->
            // This lambda will be called with each message received from /cmd_vel
            Log.d("VelocityData", "Received velocity data: $message")
            try {
                val jsonObject = JSONObject(message)
                val msgObject = jsonObject.getJSONObject("msg")
                val linearX = msgObject.getJSONObject("linear").getDouble("x")
                val angularZ = msgObject.getJSONObject("angular").getDouble("z")

                Log.d("VelocityData", "Linear x: $linearX, Angular z: $angularZ")
                velocity.value = if (linearX != null && angularZ != null) {
                    "linear: x: $linearX angular z: $angularZ"
                } else {
                    "Error parsing velocity data"
                }
                // You can then pass these values to your callback, update UI, or handle as needed

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket) // Add to list
        client.dispatcher.executorService.shutdown()
    }

    fun startCoordinatesWebSocket() {
        val topic = "/pose"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener = RoverWebSocketListener(topic) { message ->
            // This lambda will be called with each message received from /cmd_vel
            Log.d("CoordinatesData", "Received coordinates data: $message")
            // Handle the message here, e.g., parse it, update UI, etc.
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket) // Add to list
        client.dispatcher.executorService.shutdown()
    }

    fun startHeadingWebSocket() {
        val topic = "/pose"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener = RoverWebSocketListener(topic) { message ->
            // This lambda will be called with each message received from /cmd_vel
            Log.d("HeadingData", "Received heading data: $message")
            // Handle the message here, e.g., parse it, update UI, etc.
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket) // Add to list
        client.dispatcher.executorService.shutdown()
    }

    fun startBatteryWebSocket() {
        val topic = "/firmware/battery_averaged"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener = RoverWebSocketListener(topic) { message ->
            // This lambda will be called with each message received from /cmd_vel
            Log.d("BatteryData", "Received battery data: $message")
            // Handle the message here, e.g., parse it, update UI, etc.
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket) // Add to list
        client.dispatcher.executorService.shutdown()
    }


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
                val (ip, port) = ipAddress.split(":")
                val response = com.chatgptlite.wanted.helpers.sendPing(ip, port)
                if (response.isSuccessful) {
                    _pingResult.value = "Ping successful"
                } else {
                    _pingResult.value = "Error sending ping: ${response.message()}"
                }
            } catch (e: Exception) {
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
                val (ip, port) = newIpAddress.split(":")
                // url encode the textToSend
                val encodedText = URLEncoder.encode(textToSend, StandardCharsets.UTF_8.toString())

                val response = sendMessage(ip, port, encodedText)

                if (response.isSuccessful) {
                    val statusResponse = response.body()
                    _messageResult.value = "Message sent successfully. Server status: ${statusResponse?.status}. Request: ${statusResponse?.request}"
                    onSuccess?.invoke()
                } else {
                    val msg = "Error sending message: ${response.message()}"
                    _messageResult.value = msg
                    onFail?.invoke(msg)
                }
            } catch (e: Exception) {
                val msg = "Error sending message: ${e.message}"
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

    fun clearResults() {
        _pingResult.value = null
        _messageResult.value = null
    }

    fun getCoordinates() {
        TODO("Not yet implemented")
    }

    fun getVelocity(commandToSent: String) {
        viewModelScope.launch {
            try {
                val newCommandToSent = CONCAT + commandToSent
                val encodedText = URLEncoder.encode(newCommandToSent, StandardCharsets.UTF_8.toString())
                Log.d("Velocityresponse", newCommandToSent)

                // send the /cmd_vel command to retrieve velocity
                val response = sendMessage(DEFAULT_IPADDRESS, DEFAULT_PORT, encodedText)

                if (response.isSuccessful) {
                    val statusResponse = response.body()
                    _messageResult.value = "Velocity: ${statusResponse?.status}"
                    velocity.value = statusResponse?.status // update velocity state
                    Log.d("Velocityresponse", statusResponse?.toString() ?: "statusResponse is null")
                } else {
                    _messageResult.value = "Error retrieving velocity: ${response.message()}"
                }
            } catch (e: Exception) {
                _messageResult.value = "Error: ${e.message}"
            }
        }
    }


    fun stopVelocityUpdates() {
        velocityJob?.cancel() // Cancel the coroutine
        velocityJob = null
    }

    fun startVelocityUpdates(commandToSend: String) {
        stopVelocityUpdates()
        velocityJob = viewModelScope.launch {
            while (isActive) {  // Keeps looping until the coroutine is canceled
                try {
                    val newCommandToSend = CONCAT + commandToSend // Replace "CONCAT" as needed
                    val encodedText = URLEncoder.encode(newCommandToSend, StandardCharsets.UTF_8.toString())
                    Log.d("VelocityUpdate", newCommandToSend)

                    // Call sendMessage to retrieve velocity and update state
                    val response = sendMessage(DEFAULT_IPADDRESS, DEFAULT_PORT, encodedText)

                    // response: {status=success/failure, request=cmd, output=json}

                    if (response.isSuccessful) {
                        val statusResponse = response.body()
                        val requestText = statusResponse?.request ?: ""

                        // Parse the string in requestText to extract "linear x" and "angular z"
                        val linearX = Regex("x:\\s*([-?\\d.]+)").find(requestText)?.groupValues?.get(1)
                        val angularZ = Regex("z:\\s*([-?\\d.]+)").findAll(requestText).lastOrNull()?.groupValues?.get(1)

                        // Construct the parsed result or show an error if parsing fails
                        velocity.value = if (linearX != null && angularZ != null) {
                            "linear: x: $linearX angular z: $angularZ"
                        } else {
                            "Error parsing velocity data"
                        }

                        _messageResult.value = "Velocity: ${velocity.value}"

//                        _messageResult.value = "Velocity: ${statusResponse?.request}"
//                        velocity.value = statusResponse?.request // Update velocity state
                        Log.d("VelocityUpdate", statusResponse?.toString() ?: "statusResponse is null")
                    } else {
                        _messageResult.value = "Error retrieving velocity: ${response.message()}"
                    }
                } catch (e: Exception) {
                    _messageResult.value = "Error: ${e.message}"
                }

                // Wait for 1 second before sending the next update
                delay(50L)
            }
        }
    }

    fun getHeading(commandToSent: String) {
        viewModelScope.launch {
            try {

                val encodedText = URLEncoder.encode(commandToSent, StandardCharsets.UTF_8.toString())

                // send the /cmd_vel command to retrieve velocity
                val response = sendMessage(DEFAULT_IPADDRESS, DEFAULT_PORT, encodedText)

                if (response.isSuccessful) {
                    val statusResponse = response.body()
                    _messageResult.value = "Heading: ${statusResponse?.status}"
                    heading.value = statusResponse?.status // update state
                } else {
                    _messageResult.value = "Error retrieving velocity: ${response.message()}"
                }
            } catch (e: Exception) {
                _messageResult.value = "Error: ${e.message}"
            }
        }
    }

    fun getBattery(commandToSent: String) {
        viewModelScope.launch {
            try {

                val encodedText = URLEncoder.encode(commandToSent, StandardCharsets.UTF_8.toString())
                // send the /cmd_vel command to retrieve velocity
                val response = sendMessage(DEFAULT_IPADDRESS, DEFAULT_PORT, encodedText)

                if (response.isSuccessful) {
                    val statusResponse = response.body()
                    _messageResult.value = "Battery: ${statusResponse?.status}"
                    battery.value = statusResponse?.status // update state
                } else {
                    _messageResult.value = "Error retrieving velocity: ${response.message()}"
                }
            } catch (e: Exception) {
                _messageResult.value = "Error: ${e.message}"
            }
        }
    }
}

data class TelemetryConfig(
    val ipAddress: String,
    val port: String,
    val textToSend: String
)