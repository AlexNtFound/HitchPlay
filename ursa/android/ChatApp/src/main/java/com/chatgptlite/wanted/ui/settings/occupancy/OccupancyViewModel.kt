package com.chatgptlite.wanted.ui.settings.occupancy

import android.app.Application
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import com.chatgptlite.wanted.helpers.RoverWebSocketListener
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONException
import org.json.JSONObject
import org.json.JSONArray
import kotlinx.coroutines.flow.update
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import androidx.compose.runtime.mutableStateOf

class OccupancyViewModel(application: Application) : AndroidViewModel(application) {

    private val WEBSOCKET_IPADDRESS = "10.0.0.120"
    private val WEBSOCKET_PORT = "9090"
    var occupancyBitmap = mutableStateOf<Bitmap?>(null)

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
                    value == -1 -> Color.WHITE // Undefined
                    value in 0..100 -> {
                        // Map the value (0-100) to grayscale (0-255)
                        val intensity = 255 - (value * 255 / 100)
                        Color.rgb(intensity, intensity, intensity)
                    }
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

    fun testOccupancyBitmap() {
        val width = defaultOccupancyMap.first().size
        val height = defaultOccupancyMap.size

        // Flatten the occupancy map into a single list
        val flattenedData = defaultOccupancyMap.flatten()

        // Convert to a mock JSONArray
        val jsonArray = JSONArray(flattenedData)

        // Update the bitmap with the mock data
        updateOccupancyBitmap(jsonArray, width, height)
    }
}

