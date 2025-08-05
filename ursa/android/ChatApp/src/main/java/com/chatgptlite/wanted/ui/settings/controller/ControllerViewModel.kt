package com.chatgptlite.wanted.ui.settings.controller

import android.app.Application
import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.AndroidViewModel
import android.util.Log
import androidx.lifecycle.viewModelScope
import com.chatgptlite.wanted.helpers.RoverWebSocketListener
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.WebSocket
import org.json.JSONArray
import org.json.JSONException
import org.json.JSONObject
import java.io.ByteArrayInputStream

class VideoCamSettingsViewModel(application: Application) : AndroidViewModel(application) {
    private val client = OkHttpClient()
    val currentFrame = mutableStateOf<Bitmap?>(null)
    var occupancyBitmap = mutableStateOf<Bitmap?>(null)

    private var vel_webSocket: WebSocket? = null // Variable to remember the WebSocket
    private var base_webSocket: WebSocket? = null // Variable to remember the WebSocket
    private val control_client = OkHttpClient()
    private val base_client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.1"
    private val WEBSOCKET_PORT = "9090"

    // Function to create the WebSocket
    fun createWebSocket() {
        var request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        var listener = RoverWebSocketListener("/cmd_vel") { message ->
            Log.d("WebSocket", "Message received: $message")
            // Handle incoming messages if needed
        }

        vel_webSocket = control_client.newWebSocket(request, listener)

        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()

        listener = RoverWebSocketListener("/goal_pose") { message ->
            Log.d("WebSocket", "Message received: $message")
            // Handle incoming messages if needed
        }

        base_webSocket = base_client.newWebSocket(request, listener)
    }

    fun returnToBase() {

        if (base_webSocket == null) {
            Log.e("Base", "WebSocket not initialized. Call createWebSocket first.")
            return
        }

        val returnToBaseMessage = """
        {
            "op": "publish",
            "topic": "/goal_pose",
            "msg": {
                "header": {
                    "stamp": {
                        "sec": 0
                    },
                    "frame_id": "map"
                },
                "pose": {
                    "position": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0
                    },
                    "orientation": {
                        "w": 1.0
                    }
                }
            }
        }
    """.trimIndent()

        // Send the command
        val sendSuccess = base_webSocket?.send(returnToBaseMessage)
        if (sendSuccess == true) {
            Log.d("Base", "Return to base command sent successfully: $returnToBaseMessage")
        } else {
            Log.e("Base", "Failed to send return to base command.")
        }
    }


    fun controlRover(x: Double, z_angle: Double) {
        if (vel_webSocket == null) {
            Log.e("RoverControl", "WebSocket not initialized. Call createWebSocket first.")
            return
        }

        val controlMessage = """
        {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {
                    "x": $x,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": $z_angle
                }
            }
        }
    """.trimIndent()

        val sendSuccess = vel_webSocket?.send(controlMessage)
        if (sendSuccess == true) {
            Log.d("RoverControl", "Command sent successfully.")
        } else {
            Log.e("RoverControl", "Failed to send command.")
        }
    }

    fun receiveFeed(ipAddress: String, port: String, route: String) {
        viewModelScope.launch(Dispatchers.IO) {
            val request = Request.Builder()
                .url("http://$ipAddress:$port$route")
                .build()

            try {
                client.newCall(request).execute().use { response ->
                    if (!response.isSuccessful) throw Exception("Unexpected code $response")

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
                // Handle error
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

    fun saveConfig(ipAddress: String, port: String, route: String) {
        val sharedPreferences = getApplication<Application>().getSharedPreferences("CameraSetting", Context.MODE_PRIVATE)
        with(sharedPreferences.edit()) {
            putString("ipAddress", ipAddress)
            putString("port", port)
            putString("route", route)
            apply()
        }
        Log.i("CameraSettingsViewModel", "Config saved: $ipAddress:$port -> $route")
    }

    fun loadConfig(): CameraConfig? {
        val sharedPreferences = getApplication<Application>().getSharedPreferences("CameraSetting", Context.MODE_PRIVATE)
        val ipAddress = sharedPreferences.getString("ipAddress", null)
        val port = sharedPreferences.getString("port", null)
        val route = sharedPreferences.getString("route", null)

        return if (ipAddress != null && port != null && route != null) {
            CameraConfig(ipAddress, port, route)
        } else {
            null
        }
    }

//
//    fun closeAllConnections() {
//        vel_webSocket.close(1000, "Closed all connections")
//        base_webSocket.close(1000, "Closed all connections")
//        client.dispatcher.executorService.shutdown()
//    }

    override fun onCleared() {
        super.onCleared()
//
//        control_client.dispatcher.executorService.shutdown()
//        base_client.dispatcher.executorService.shutdown()
//        client.dispatcher.executorService.shutdown()
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

data class CameraConfig(
    val ipAddress: String,
    val port: String,
    val route: String
)