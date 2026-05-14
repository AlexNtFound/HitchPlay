package com.chatgptlite.wanted.ui.settings.rover

import android.app.Application
import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.Log
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import okhttp3.OkHttpClient
import okhttp3.WebSocket
import okhttp3.Request

import kotlinx.coroutines.*
import com.chatgptlite.wanted.helpers.RoverWebSocketListener
import com.chatgptlite.wanted.ui.settings.controller.MjpegReader
//import com.google.firebase.crashlytics.buildtools.reloc.com.google.common.util.concurrent.AtomicDouble
import org.json.JSONArray
import org.json.JSONException
import org.json.JSONObject
import java.io.ByteArrayInputStream

import com.chatgptlite.wanted.MainViewModel
import kotlinx.coroutines.flow.collectLatest

class RoverSettingsViewModel(application: Application) : AndroidViewModel(application) {

    // MutableState for x, y, z coordinates, velocity, and heading
    var xCoordinate = MutableStateFlow<String?>("0")
    var yCoordinate = MutableStateFlow<String?>("0")
    var zCoordinate = MutableStateFlow<String?>("0")
    var linear_velocity = MutableStateFlow<String?>("0")
    var angular_velocity = MutableStateFlow<String?>("0")
    var heading = MutableStateFlow<String?>("0")
    var battery = MutableStateFlow<Int?>(null)  // null = no data yet
    private val DEFAULT_IPADDRESS = "10.0.0.1"
    private val DEFAULT_PORT = "8000"
    private val CONCAT = "ros2 topic echo "

    // map values
    val currentFrame = mutableStateOf<Bitmap?>(null)
    var occupancyBitmap = mutableStateOf<Bitmap?>(null)

    private var velocityJob: Job? = null

    private val webSockets = mutableListOf<WebSocket>()
    private val client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.1"
    private val WEBSOCKET_PORT = "9090"

    private val _pingResult = MutableStateFlow<String?>(null)
    val pingResult: StateFlow<String?> = _pingResult.asStateFlow()

    private val _messageResult = MutableStateFlow<String?>(null)
    val messageResult: StateFlow<String?> = _messageResult.asStateFlow()

    private val _roverState = MutableStateFlow("Disconnected")
    val roverState: StateFlow<String> = _roverState
    private var _connectionReady = false

    // ---- Per-connection guards (prevent duplicates while allowing retry) ----
    private var _coordsStarted = false
    private var _velocityStarted = false
    private var _batteryStarted = false
    private var _occupancyStarted = false
    private var _feedStarted = false
    private var _syncStarted = false

    fun initFeedsOnce(mainViewModel: MainViewModel) {
        syncWithMainViewModel(mainViewModel)
        // Pose data is now piped from ControllerViewModel — no duplicate /pose WebSocket
        startVelocityWebSocket()
        startBatteryWebSocket()
        startOccupancyWebSocket()
        if (!_feedStarted) {
            _feedStarted = true
            receiveFeed("10.0.0.1", "8080", "/stream?topic=/camera/image_raw&type=ros_compressed")
        }
    }

    /** Called by ControllerViewModel when it receives /pose data — avoids duplicate WebSocket */
    fun updatePoseFromController(x: Double, y: Double, orientationZ: Double) {
        xCoordinate.value = String.format("%.2f", x)
        yCoordinate.value = String.format("%.2f", y)
        heading.value = String.format("%.2f", orientationZ)
    }

    // ---- Per-connection guards (prevent duplicates while allowing retry) ----
    private var _coordsStarted = false
    private var _velocityStarted = false
    private var _batteryStarted = false
    private var _occupancyStarted = false
    private var _feedStarted = false
    private var _syncStarted = false

    fun initFeedsOnce(mainViewModel: MainViewModel) {
        syncWithMainViewModel(mainViewModel)
        startCoordinatesWebSocket()
        startVelocityWebSocket()
        startBatteryWebSocket()
        startOccupancyWebSocket()
        if (!_feedStarted) {
            _feedStarted = true
            receiveFeed("10.0.0.1", "8080", "/stream?topic=/camera/image_raw&type=ros_compressed")
        }
    }

    // Function to sync with MainViewModel
    fun syncWithMainViewModel(mainViewModel: MainViewModel) {
        if (_syncStarted) return
        _syncStarted = true
        viewModelScope.launch {
            mainViewModel.roverStateFlow.collectLatest { newState ->
                // Operational states from LLM/commands override connection state
                _roverState.value = newState
            }
        }
    }

    /** Called by ControllerViewModel when all WebSocket topics connect */
    fun updateConnectionState(allConnected: Boolean) {
        _connectionReady = allConnected
        // Only update state if no operational state is active
        val current = _roverState.value
        if (allConnected && (current == "Disconnected" || current == "Connecting")) {
            _roverState.value = "Idle"
        } else if (!allConnected && current == "Idle") {
            _roverState.value = "Connecting"
        }
    }

    /** Called by ControllerViewModel when /nav_status updates — reflects real rover activity */
    fun updateNavState(status: String) {
        if (!_connectionReady) return  // Don't override connection states
        _roverState.value = when (status) {
            "navigating" -> "Navigating"
            "replaying" -> "Replaying"
            "line_following" -> "Line Following"
            "arrived", "idle" -> "Idle"
            "cancelled", "replay_stopped", "line_follow_stopped" -> "Idle"
            "failed", "timeout", "blocked" -> "Error"
            else -> status.replaceFirstChar { it.uppercase() }  // Fallback: capitalize raw status
        }
    }

    //Video Feed
    fun receiveFeed(ipAddress: String, port: String, route: String) {
        viewModelScope.launch(Dispatchers.IO) {
            val request = Request.Builder()
                .url("http://$ipAddress:$port$route")
                .build()
            Log.d("VideoFeed", "receiveFeed request built")
            try {
                client.newCall(request).execute().use { response ->
                    if (!response.isSuccessful) throw Exception("Unexpected code $response")
                    Log.d("VideoFeed", "parsing video data")
                    val input = response.body?.byteStream() ?: return@use
                    val reader = MjpegReader(input)

                    while (true) {
                        val frameBytes = reader.readFrame() ?: break
                        val bitmap = BitmapFactory.decodeStream(ByteArrayInputStream(frameBytes))
                        currentFrame.value = bitmap
                    }
                }
            } catch (e: Exception) {
                e.printStackTrace()
                Log.e("VideoFeed", "Exception in Video Feed $e")
            }
        }
    }


    //Occupancy Map
    fun startOccupancyWebSocket() {
        if (_occupancyStarted) return
        _occupancyStarted = true
        val topic = "/map"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        Log.d("Occupancy", "Start websocket")
        val listener = RoverWebSocketListener(topic) { message ->
            Log.d("Occupancy", "Received data: $message")
            try {
                val jsonObject = JSONObject(message)
                val dataArray = jsonObject.getJSONObject("msg").getJSONArray("data")
                val width = jsonObject.getJSONObject("msg").getJSONObject("info").getInt("width")
                val height = jsonObject.getJSONObject("msg").getJSONObject("info").getInt("height")
                // Update occupancy bitmap
                updateOccupancyBitmap(dataArray, width, height)

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
        }

        client.newWebSocket(request, listener)

    }

    private fun updateOccupancyBitmap(dataArray: JSONArray, width: Int, height: Int) {
        // Create a new Bitmap
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        val paint = Paint()

        // Iterate through the data and draw on the Bitmap
        for (row in 0 until height) {
            for (col in 0 until width) {
                val index = row * width + col
                val value = dataArray.getInt(index)

                // Map the value to a color
                val color = when {
                    value == -1 -> Color.GRAY // Undefined
//                    value in 0..100 -> {
//                        // Map the value (0-100) to grayscale (0-255)
//                        val intensity = 255 - (value * 255 / 100)
//                        Color.rgb(intensity, intensity, intensity)
//                    }
                    value == 0 -> Color.WHITE
                    else -> Color.BLACK // Fallback for any unexpected value
                }

                paint.color = color
                canvas.drawPoint(col.toFloat(), row.toFloat(), paint)
            }
        }

        // Update the mutable state
        occupancyBitmap.value = bitmap
    }

    val defaultOccupancyMap: List<List<Int>> = listOf(
        listOf(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
        listOf(-1, 0, 0, 0, 0, 0, 0, 0, 0, -1),
        listOf(-1, 0, 10, 20, 30, 40, 50, 60, 0, -1),
        listOf(-1, 0, 20, 40, 60, 80, 100, 60, 0, -1),
        listOf(-1, 0, 30, 60, 90, 100, 90, 60, 0, -1),
        listOf(-1, 0, 40, 80, 100, 100, 100, 80, 0, -1),
        listOf(-1, 0, 30, 60, 90, 100, 90, 60, 0, -1),
        listOf(-1, 0, 20, 40, 60, 80, 100, 40, 0, -1),
        listOf(-1, 0, 0, 0, 0, 0, 0, 0, 0, -1),
        listOf(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
    )

    fun startVelocityWebSocket() {
        if (_velocityStarted) return
        _velocityStarted = true
        val topic1 = "/cmd_vel"
        val topic2 = "/cmd_vel_nav"

        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener1 = RoverWebSocketListener(topic1) { message ->
            processVelocityMessage(message)
        }

        val listener2 = RoverWebSocketListener(topic2) { message ->
            processVelocityMessage(message)
        }

        val webSocket_1 = client.newWebSocket(request, listener1)
        val webSocket_2 = client.newWebSocket(request, listener2)
        webSockets.add(webSocket_1) // Add to list
        webSockets.add(webSocket_2)
//        client.dispatcher.executorService.shutdown()
    }

    private fun processVelocityMessage(message: String) {
        try {
            val jsonObject = JSONObject(message)
            val msgObject = jsonObject.getJSONObject("msg")
            val linearX = msgObject.getJSONObject("linear").getDouble("x")
            val angularZ = msgObject.getJSONObject("angular").getDouble("z")

            // Always pass through the actual value — including 0.0 when stopped
            linear_velocity.value = String.format("%.5f", linearX)
            angular_velocity.value = String.format("%.5f", angularZ)

        } catch (e: JSONException) {
            Log.e("WebSocket", "JSON parsing error: ${e.message}")
        }
    }

    // Coordinates and Heading
    fun startCoordinatesWebSocket() {
        if (_coordsStarted) return
        _coordsStarted = true
        val topic = "/pose"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        Log.d("CoordinatesData", "startCoordinatesWebSocket")

        val listener = RoverWebSocketListener(topic) { message ->
            Log.d("CoordinatesData", "Received coordinates data: $message")

            try {
                Log.d("CoordinatesData", "In try condition")
//                val jsonObject = JSONObject(message)
//                val msgObject = jsonObject.getJSONObject("pose").getJSONObject("pose")
//                val local_x = msgObject.getJSONObject("position").getDouble("x")
//                val local_y = msgObject.getJSONObject("position").getDouble("y")
//                val local_z = msgObject.getJSONObject("orientation").getDouble("z")

                val jsonObject = JSONObject(message)
                val poseObject = jsonObject.getJSONObject("msg").getJSONObject("pose").getJSONObject("pose")
                val local_x = poseObject.getJSONObject("position").getDouble("x")
                val local_y = poseObject.getJSONObject("position").getDouble("y")
                val local_z = poseObject.getJSONObject("orientation").getDouble("z")


                Log.d("Coordinates", "x: $local_x, y: $local_y, z: $local_z")
                xCoordinate.value = if (local_x != null) {String.format("%.2f", local_x)} else { "Error Paring x data"}
                yCoordinate.value = if (local_y != null) {String.format("%.2f", local_y)} else { "Error Paring y data"}
                heading.value = if (local_z != null) {String.format("%.2f", local_z)} else { "Error Paring z data"}

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
            // Handle the message here, e.g., parse it, update UI, etc.

        }

        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket)
    }

    fun startBatteryWebSocket() {
        if (_batteryStarted) return
        _batteryStarted = true
        val topic = "/firmware/battery_averaged"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val listener = RoverWebSocketListener(topic) { message ->
            Log.d("BatteryData", "Received battery data: $message")
            try {
                val jsonObject = JSONObject(message)
                val msgObject = jsonObject.getJSONObject("msg")
                val voltage = msgObject.getDouble("data")

                Log.d("BatteryData", "Parsed battery data: $voltage")

                // Convert voltage to percentage
                val batteryPercentage = convertVoltageToPercentage(voltage)

                battery.value = batteryPercentage

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
        }

        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket)
    }

    private fun convertVoltageToPercentage(voltage: Double): Int {
        // Leo Rover 12V battery: 12.6V = full, ~10.0V = dead
        return when {
            voltage >= 12.6 -> 100
            voltage >= 11.8 -> 75
            voltage >= 11.4 -> 50
            voltage >= 10.9 -> 25
            voltage >= 10.2 -> 10
            else -> 5  // Critically low but still reporting
        }
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
    }

    override fun onCleared() {
        super.onCleared()
        closeAllConnections()
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

class MjpegReader(private val input: java.io.InputStream) {
    private val buffer = ByteArray(1024)

    fun readFrame(): ByteArray? {
        val baos = java.io.ByteArrayOutputStream()
        var lastBytes = ByteArray(2)
        var isJpegStart = false

        while (true) {
            val bytesRead = input.read(buffer)
            if (bytesRead == -1) return null

            for (i in 0 until bytesRead) {
                baos.write(buffer[i].toInt())
                System.arraycopy(lastBytes, 1, lastBytes, 0, 1)
                lastBytes[1] = buffer[i]

                if (!isJpegStart && lastBytes[0] == 0xFF.toByte() && lastBytes[1] == 0xD8.toByte()) {
                    isJpegStart = true
                    baos.reset()
                    baos.write(0xFF)
                    baos.write(0xD8)
                } else if (isJpegStart && lastBytes[0] == 0xFF.toByte() && lastBytes[1] == 0xD9.toByte()) {
                    return baos.toByteArray()
                }
            }
        }
    }
}

data class RoverConfig(
    val ipAddress: String,
    val port: String,
    val textToSend: String
)