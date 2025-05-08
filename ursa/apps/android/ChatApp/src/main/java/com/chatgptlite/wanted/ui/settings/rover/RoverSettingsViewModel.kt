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
    var linear_velocity = MutableStateFlow<String?>("-1000")
    var angular_velocity = MutableStateFlow<String?>("-1000")
    var heading = MutableStateFlow<String?>("0")
    var battery = MutableStateFlow<Int>(0)
    private val DEFAULT_IPADDRESS = "10.0.0.120"
    private val DEFAULT_PORT = "8000"
    private val CONCAT = "ros2 topic echo "

    // map values
    val currentFrame = mutableStateOf<Bitmap?>(null)
    var occupancyBitmap = mutableStateOf<Bitmap?>(null)

    private var velocityJob: Job? = null

    private val webSockets = mutableListOf<WebSocket>()
    private val client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.120"
    private val WEBSOCKET_PORT = "9090"

    private val _pingResult = MutableStateFlow<String?>(null)
    val pingResult: StateFlow<String?> = _pingResult.asStateFlow()

    private val _messageResult = MutableStateFlow<String?>(null)
    val messageResult: StateFlow<String?> = _messageResult.asStateFlow()

    private val _roverState = MutableStateFlow("Executing")
    val roverState: StateFlow<String> = _roverState

    // Function to sync with MainViewModel
    fun syncWithMainViewModel(mainViewModel: MainViewModel) {
        viewModelScope.launch {
            mainViewModel.roverStateFlow.collectLatest { newState ->
                _roverState.value = newState // Update when MainViewModel emits new state
            }
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
//        val topic = "/local_costmap/costmap"
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

        val client = OkHttpClient()
        client.newWebSocket(request, listener)
        client.dispatcher.executorService.shutdown()

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
        val topic1 = "/cmd_vel"
        val topic2 = "/cmd_vel_nav"

        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        val client = OkHttpClient()
        var tempLinearVel = com.google.common.util.concurrent.AtomicDouble(0.0)
        var tempAngularVel = com.google.common.util.concurrent.AtomicDouble(0.0)

        val listener1 = RoverWebSocketListener(topic1) { message ->
            processVelocityMessage(message, tempLinearVel, tempAngularVel)
        }

        val listener2 = RoverWebSocketListener(topic2) { message ->
            processVelocityMessage(message, tempLinearVel, tempAngularVel)
        }

        val webSocket_1 = client.newWebSocket(request, listener1)
        val webSocket_2 = client.newWebSocket(request, listener2)
        webSockets.add(webSocket_1) // Add to list
        webSockets.add(webSocket_2)
//        client.dispatcher.executorService.shutdown()
    }

    private fun processVelocityMessage(
        message: String,
        tempLinearVel: com.google.common.util.concurrent.AtomicDouble,
        tempAngularVel: com.google.common.util.concurrent.AtomicDouble
    ) {
        try {
            val jsonObject = JSONObject(message)
            val msgObject = jsonObject.getJSONObject("msg")
            val linearX = msgObject.getJSONObject("linear").getDouble("x")
            val angularZ = msgObject.getJSONObject("angular").getDouble("z")

            synchronized(tempLinearVel) {
                tempLinearVel.set(
                    if (linearX != 0.0) linearX else tempLinearVel.get()
                )
            }
            synchronized(tempAngularVel) {
                tempAngularVel.set(
                    if (angularZ != 0.0) angularZ else tempAngularVel.get()
                )
            }

            // Update UI or pass data to other components
            linear_velocity.value = String.format("%.5f", tempLinearVel.get())
            angular_velocity.value = String.format("%.5f", tempAngularVel.get())

        } catch (e: JSONException) {
            Log.e("WebSocket", "JSON parsing error: ${e.message}")
        }
    }

    fun startVelocityWebSocket_() {
        val topic_1 = "/cmd_vel"
        val topic_2 = "/cmd_vel_nav"

        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        var temp_linear_vel = 0.0
        var temp_angular_vel = 0.0

        var listener_1 = RoverWebSocketListener(topic_1) { message ->
            // This lambda will be called with each message received from /cmd_vel
            Log.d("VelocityData", "Received velocity data: $message")
            try {
                val jsonObject = JSONObject(message)
                val msgObject = jsonObject.getJSONObject("msg")
                val linearX = msgObject.getJSONObject("linear").getDouble("x")
                val angularZ = msgObject.getJSONObject("angular").getDouble("z")

                Log.d("VelocityData", "Linear x: $linearX, Angular z: $angularZ")
                temp_linear_vel = if (linearX != null) {
                    linearX
                } else {
                    -2000.0
                }
                temp_angular_vel = if (angularZ != null) {
                    angularZ
                } else {
                    -2000.0
                }


                // You can then pass these values to your callback, update UI, or handle as needed

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
        }


        val listener_2 = RoverWebSocketListener(topic_2) { message ->
                // This lambda will be called with each message received from /cmd_vel
                Log.d("VelocityData", "Received velocity data: $message")
                try {
                    val jsonObject = JSONObject(message)
                    val msgObject = jsonObject.getJSONObject("msg")
                    val linearX = msgObject.getJSONObject("linear").getDouble("x")
                    val angularZ = msgObject.getJSONObject("angular").getDouble("z")

                    Log.d("VelocityData", "Linear x: $linearX, Angular z: $angularZ")

                    temp_linear_vel = when {
                        linearX != 0.0 -> linearX
                        temp_linear_vel != 0.0 -> temp_linear_vel
                        else -> 0.0
                    }

                    temp_angular_vel = when {
                        angularZ != 0.0 -> angularZ
                        temp_angular_vel != 0.0 -> temp_angular_vel
                        else -> 0.0
                    }

                    // You can then pass these values to your callback, update UI, or handle as needed

                } catch (e: JSONException) {
                    Log.e("WebSocket", "JSON parsing error: ${e.message}")
                }
        }

        linear_velocity.value = if (temp_linear_vel != null ) {
                    String.format("%.5f", temp_linear_vel)
        } else {
            "Error parsing velocity data"
        }

        angular_velocity.value = if (temp_angular_vel != null ) {
            String.format("%.5f", temp_angular_vel)
        } else {
            "Error parsing velocity data"
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener_1)
        webSockets.add(webSocket) // Add to list
        client.dispatcher.executorService.shutdown()

        val client_2 = OkHttpClient()
        val webSocket_2 = client_2.newWebSocket(request, listener_2)
        webSockets.add(webSocket_2) // Add to list
        client.dispatcher.executorService.shutdown()
    }

    // Coordinates and Heading
    fun startCoordinatesWebSocket() {
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
            Log.d("BatteryData", "Received battery data: $message")
            try {
                val jsonObject = JSONObject(message)
                val msgObject = jsonObject.getJSONObject("msg")
                val voltage = msgObject.getDouble("data")

                Log.d("BatteryData", "Parsed battery data: $voltage")

                // Convert voltage to percentage
                val batteryPercentage = convertVoltageToPercentage(voltage)

                battery.value = if (batteryPercentage != null) {
                    batteryPercentage
                } else {
                    0
                }

            } catch (e: JSONException) {
                Log.e("WebSocket", "JSON parsing error: ${e.message}")
            }
        }

        val client = OkHttpClient()
        val webSocket = client.newWebSocket(request, listener)
        webSockets.add(webSocket)
        client.dispatcher.executorService.shutdown()
    }

    private fun convertVoltageToPercentage(voltage: Double): Int? {
        return when {
            voltage >= 12.6 -> 100
            voltage in 11.8..12.6 -> 75
            voltage in 11.4..11.8 -> 50
            voltage in 10.9..11.4 -> 25
            voltage < 10.9 -> null // Represents low battery
            else -> null
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
        client.dispatcher.executorService.shutdown()
    }

    override fun onCleared() {
        super.onCleared()
        closeAllConnections() // Ensure WebSocket is closed when ViewModel is cleared
        client.dispatcher.executorService.shutdown()
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