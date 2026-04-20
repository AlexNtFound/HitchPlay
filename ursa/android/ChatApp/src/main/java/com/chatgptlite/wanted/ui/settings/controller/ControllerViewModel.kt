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
import androidx.compose.runtime.mutableFloatStateOf
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

    private var vel_webSocket: WebSocket? = null
    private var base_webSocket: WebSocket? = null
    private var estop_webSocket: WebSocket? = null
    private var cancel_webSocket: WebSocket? = null
    private var waypoint_webSocket: WebSocket? = null
    private var linefollow_webSocket: WebSocket? = null
    private var navstatus_webSocket: WebSocket? = null

    private val control_client = OkHttpClient()
    private val base_client = OkHttpClient()
    private val WEBSOCKET_IPADDRESS = "10.0.0.1"
    private val WEBSOCKET_PORT = "9090"

    // ---- UI State ----
    val eStopActive = mutableStateOf(false)
    val isRecording = mutableStateOf(false)
    val isReplaying = mutableStateOf(false)
    val isLineFollowing = mutableStateOf(false)
    val navStatus = mutableStateOf("idle")
    val navStatusDetail = mutableStateOf("")
    val navMode = mutableStateOf("slam")

    // Speed limits (adjustable at runtime)
    val maxLinearSpeed = mutableFloatStateOf(1.0f)
    val maxAngularSpeed = mutableFloatStateOf(1.5f)

    // Home position
    private var homeX = 0.0
    private var homeY = 0.0
    private var homeYaw = 0.0
    val homeIsCustom = mutableStateOf(false)

    // Current robot pose (from TF: map -> base_link)
    val currentRobotX = mutableStateOf(0.0)
    val currentRobotY = mutableStateOf(0.0)
    val currentRobotYaw = mutableStateOf(0.0)
    private var tf_webSocket: WebSocket? = null

    // Waypoint list (recorded poses as JSON-serializable data)
    private val waypoints = mutableListOf<WaypointEntry>()
    val waypointCount = mutableStateOf(0)
    private var recordLastTime = 0L

    // ---- Init Guard (prevent duplicate connections on page switch) ----
    private var feedsInitialized = false

    fun initFeedsOnce() {
        if (feedsInitialized) return
        feedsInitialized = true
        val config = loadConfig()
        val ip = config?.ipAddress ?: "10.0.0.1"
        val port = config?.port ?: "8080"
        val route = config?.route ?: "/stream?topic=/camera/image_raw&type=ros_compressed"
        receiveFeed(ip, port, route)
        createWebSocket()
        startOccupancyWebSocket()
    }

    // ---- WebSocket Setup ----

    fun createWebSocket() {
        // /cmd_vel — rover velocity
        var request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        var listener = RoverWebSocketListener("/cmd_vel") { message ->
            Log.d("WebSocket", "cmd_vel received: $message")
        }
        vel_webSocket = control_client.newWebSocket(request, listener)

        // /goal_pose — navigation goals
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        listener = RoverWebSocketListener("/goal_pose") { message ->
            Log.d("WebSocket", "goal_pose received: $message")
        }
        base_webSocket = base_client.newWebSocket(request, listener)

        // /e_stop — emergency stop
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        listener = RoverWebSocketListener("/e_stop") { message ->
            Log.d("WebSocket", "e_stop received: $message")
        }
        estop_webSocket = OkHttpClient().newWebSocket(request, listener)

        // /cancel_nav — cancel navigation
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        listener = RoverWebSocketListener("/cancel_nav") { message ->
            Log.d("WebSocket", "cancel_nav received: $message")
        }
        cancel_webSocket = OkHttpClient().newWebSocket(request, listener)

        // /waypoint_replay — send waypoint list for autonomous replay
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        listener = RoverWebSocketListener("/waypoint_replay") { message ->
            Log.d("WebSocket", "waypoint_replay received: $message")
        }
        waypoint_webSocket = OkHttpClient().newWebSocket(request, listener)

        // /line_follow_cmd — line following commands
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        listener = RoverWebSocketListener("/line_follow_cmd") { message ->
            Log.d("WebSocket", "line_follow_cmd received: $message")
        }
        linefollow_webSocket = OkHttpClient().newWebSocket(request, listener)

        // /nav_status — subscribe for navigation status updates
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        val navStatusListener = RoverWebSocketListener("/nav_status") { message ->
            handleNavStatus(message)
        }
        navstatus_webSocket = OkHttpClient().newWebSocket(request, navStatusListener)

        // /tf — subscribe for robot pose (map -> base_link)
        request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        val tfListener = RoverWebSocketListener("/tf") { message ->
            handleTfMessage(message)
        }
        tf_webSocket = OkHttpClient().newWebSocket(request, tfListener)
    }

    // ---- TF Pose Tracking ----

    private fun handleTfMessage(rawMessage: String) {
        try {
            val json = JSONObject(rawMessage)
            val msg = json.optJSONObject("msg") ?: return
            val transforms = msg.optJSONArray("transforms") ?: return

            for (i in 0 until transforms.length()) {
                val tf = transforms.getJSONObject(i)
                val header = tf.optJSONObject("header") ?: continue
                val childFrame = tf.optString("child_frame_id", "")
                val parentFrame = header.optString("frame_id", "")

                // We want map -> base_link (same as Python script's TF lookup)
                if (parentFrame == "map" && childFrame == "base_link") {
                    val transform = tf.getJSONObject("transform")
                    val translation = transform.getJSONObject("translation")
                    val rotation = transform.getJSONObject("rotation")

                    currentRobotX.value = translation.getDouble("x")
                    currentRobotY.value = translation.getDouble("y")

                    // Convert quaternion to yaw: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                    val qx = rotation.getDouble("x")
                    val qy = rotation.getDouble("y")
                    val qz = rotation.getDouble("z")
                    val qw = rotation.getDouble("w")
                    currentRobotYaw.value = Math.atan2(
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)
                    )
                }
            }
        } catch (e: Exception) {
            // TF messages are high frequency, don't spam logs
        }
    }

    // ---- Navigation Status ----

    private fun handleNavStatus(rawMessage: String) {
        try {
            val json = JSONObject(rawMessage)
            val data = json.optJSONObject("msg")?.optString("data") ?: return
            val parts = data.split(": ", limit = 2)
            val newStatus = parts.getOrElse(0) { data }
            val detail = parts.getOrElse(1) { "" }

            navStatus.value = newStatus
            navStatusDetail.value = detail

            when (newStatus) {
                "replaying" -> {
                    isReplaying.value = true
                }
                "line_following" -> {
                    isLineFollowing.value = true
                }
                "idle", "arrived", "cancelled", "failed",
                "timeout", "blocked", "replay_stopped", "line_follow_stopped" -> {
                    if (newStatus == "replay_stopped" || newStatus == "cancelled") {
                        isReplaying.value = false
                    }
                    if (newStatus == "line_follow_stopped" || newStatus == "cancelled") {
                        isLineFollowing.value = false
                    }
                }
            }
            Log.d("NavStatus", "Status: $newStatus, Detail: $detail")
        } catch (e: Exception) {
            Log.e("NavStatus", "Parse error: ${e.message}")
        }
    }

    // ---- E-Stop ----

    fun toggleEStop() {
        eStopActive.value = !eStopActive.value
        publishEStop(eStopActive.value)
        if (eStopActive.value) {
            // Send zero velocity immediately
            controlRover(0.0, 0.0)
        }
    }

    private fun publishEStop(active: Boolean) {
        val msg = """
        {
            "op": "publish",
            "topic": "/e_stop",
            "msg": { "data": $active }
        }
        """.trimIndent()
        val success = estop_webSocket?.send(msg)
        Log.d("EStop", "E-Stop ${if (active) "ACTIVATED" else "released"}, sent=$success")
    }

    // ---- Rover Movement ----

    fun controlRover(x: Double, z_angle: Double) {
        if (eStopActive.value) {
            Log.w("RoverControl", "E-Stop active, ignoring command")
            return
        }
        if (vel_webSocket == null) {
            Log.e("RoverControl", "WebSocket not initialized. Call createWebSocket first.")
            return
        }

        val controlMessage = """
        {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": { "x": $x, "y": 0.0, "z": 0.0 },
                "angular": { "x": 0.0, "y": 0.0, "z": $z_angle }
            }
        }
        """.trimIndent()

        val sendSuccess = vel_webSocket?.send(controlMessage)
        if (sendSuccess == true) {
            Log.d("RoverControl", "Command sent: linear=$x, angular=$z_angle")
        } else {
            Log.e("RoverControl", "Failed to send command.")
        }
    }

    // ---- Navigation: Home ----

    /** Set home to specific coordinates */
    fun setHome(x: Double, y: Double, yaw: Double) {
        homeX = x
        homeY = y
        homeYaw = yaw
        homeIsCustom.value = true
        Log.i("Nav", "Home set to ($x, $y, ${Math.toDegrees(yaw)}deg)")
    }

    /** Set home to current robot position (from TF) */
    fun setHomeFromCurrentPose() {
        homeX = currentRobotX.value
        homeY = currentRobotY.value
        homeYaw = currentRobotYaw.value
        homeIsCustom.value = true
        Log.i("Nav", "Home set to current pose (${homeX}, ${homeY}, ${Math.toDegrees(homeYaw)}deg)")
    }

    fun resetHome() {
        homeX = 0.0
        homeY = 0.0
        homeYaw = 0.0
        homeIsCustom.value = false
        Log.i("Nav", "Home reset to map origin (0, 0, 0°)")
    }

    fun navigateHome() {
        publishGoalPose(homeX, homeY, homeYaw)
    }

    fun returnToBase() {
        publishGoalPose(0.0, 0.0, 0.0)
    }

    private fun publishGoalPose(x: Double, y: Double, yaw: Double) {
        if (base_webSocket == null) {
            Log.e("Nav", "WebSocket not initialized.")
            return
        }
        // Convert yaw to quaternion (rotation around Z axis)
        val qz = Math.sin(yaw / 2.0)
        val qw = Math.cos(yaw / 2.0)

        val msg = """
        {
            "op": "publish",
            "topic": "/goal_pose",
            "msg": {
                "header": { "stamp": { "sec": 0 }, "frame_id": "map" },
                "pose": {
                    "position": { "x": $x, "y": $y, "z": 0.0 },
                    "orientation": { "x": 0.0, "y": 0.0, "z": $qz, "w": $qw }
                }
            }
        }
        """.trimIndent()

        val success = base_webSocket?.send(msg)
        Log.d("Nav", "Goal pose sent to ($x, $y, ${Math.toDegrees(yaw)}°), success=$success")
    }

    // ---- Cancel Navigation / Replay / Line Follow ----

    fun cancelAll() {
        // Publish cancel_nav
        val cancelMsg = """
        {
            "op": "publish",
            "topic": "/cancel_nav",
            "msg": {}
        }
        """.trimIndent()
        cancel_webSocket?.send(cancelMsg)

        // Also send zero velocity
        controlRover(0.0, 0.0)

        // Stop local replay/line-follow state
        if (isReplaying.value) stopReplay()
        if (isLineFollowing.value) stopLineFollow()

        navStatus.value = "idle"
        Log.i("Nav", "All navigation cancelled")
    }

    // ---- Waypoint Recording ----

    fun startRecording() {
        waypoints.clear()
        waypointCount.value = 0
        isRecording.value = true
        recordLastTime = System.currentTimeMillis()
        Log.i("Waypoint", "Recording started")
    }

    fun stopRecording() {
        isRecording.value = false
        waypointCount.value = waypoints.size
        Log.i("Waypoint", "Recording stopped, ${waypoints.size} waypoints saved")
        saveWaypoints()
    }

    fun toggleRecording() {
        if (isRecording.value) stopRecording() else startRecording()
    }

    /** Call from joystick listener at ~10Hz to record current position */
    fun recordWaypointTick(x: Double, y: Double, yaw: Double) {
        if (!isRecording.value) return
        val now = System.currentTimeMillis()
        if (now - recordLastTime < 100) return // 10Hz throttle
        recordLastTime = now
        waypoints.add(WaypointEntry("nav", x, y, yaw))
        waypointCount.value = waypoints.size
    }

    private fun saveWaypoints() {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                val prefs = getApplication<Application>()
                    .getSharedPreferences("WaypointData", Context.MODE_PRIVATE)
                val jsonArray = JSONArray()
                for (wp in waypoints) {
                    val obj = JSONObject()
                    obj.put("type", wp.type)
                    obj.put("x", wp.x)
                    obj.put("y", wp.y)
                    obj.put("yaw", wp.yaw)
                    jsonArray.put(obj)
                }
                prefs.edit().putString("waypoints", jsonArray.toString()).apply()
                Log.i("Waypoint", "Saved ${waypoints.size} waypoints to prefs")
            } catch (e: Exception) {
                Log.e("Waypoint", "Save failed: ${e.message}")
            }
        }
    }

    private fun loadWaypoints() {
        try {
            val prefs = getApplication<Application>()
                .getSharedPreferences("WaypointData", Context.MODE_PRIVATE)
            val json = prefs.getString("waypoints", null) ?: return
            val array = JSONArray(json)
            waypoints.clear()
            for (i in 0 until array.length()) {
                val obj = array.getJSONObject(i)
                waypoints.add(
                    WaypointEntry(
                        obj.getString("type"),
                        obj.getDouble("x"),
                        obj.getDouble("y"),
                        obj.getDouble("yaw")
                    )
                )
            }
            waypointCount.value = waypoints.size
            Log.i("Waypoint", "Loaded ${waypoints.size} waypoints from prefs")
        } catch (e: Exception) {
            Log.e("Waypoint", "Load failed: ${e.message}")
        }
    }

    // ---- Waypoint Replay ----

    fun toggleReplay() {
        if (isReplaying.value) stopReplay() else startReplay()
    }

    private fun startReplay() {
        if (waypoints.isEmpty()) {
            loadWaypoints()
        }
        if (waypoints.isEmpty()) {
            Log.w("Replay", "No waypoints to replay")
            return
        }

        val jsonArray = JSONArray()
        for (wp in waypoints) {
            val obj = JSONObject()
            obj.put("type", wp.type)
            obj.put("x", wp.x)
            obj.put("y", wp.y)
            obj.put("yaw", wp.yaw)
            jsonArray.put(obj)
        }
        val payload = JSONObject()
        payload.put("action", "start")
        payload.put("waypoints", jsonArray)

        val msg = """
        {
            "op": "publish",
            "topic": "/waypoint_replay",
            "msg": { "data": ${JSONObject.quote(payload.toString())} }
        }
        """.trimIndent()

        waypoint_webSocket?.send(msg)
        isReplaying.value = true
        Log.i("Replay", "Started replay with ${waypoints.size} waypoints")
    }

    private fun stopReplay() {
        val payload = JSONObject()
        payload.put("action", "stop")

        val msg = """
        {
            "op": "publish",
            "topic": "/waypoint_replay",
            "msg": { "data": ${JSONObject.quote(payload.toString())} }
        }
        """.trimIndent()

        waypoint_webSocket?.send(msg)
        isReplaying.value = false
        Log.i("Replay", "Stopped replay")
    }

    // ---- Line Following ----

    fun toggleLineFollow() {
        if (isLineFollowing.value) stopLineFollow() else startLineFollow()
    }

    private fun startLineFollow() {
        val payload = JSONObject()
        payload.put("action", "start")

        val msg = """
        {
            "op": "publish",
            "topic": "/line_follow_cmd",
            "msg": { "data": ${JSONObject.quote(payload.toString())} }
        }
        """.trimIndent()

        linefollow_webSocket?.send(msg)
        isLineFollowing.value = true
        Log.i("LineFollow", "Started line following")
    }

    private fun stopLineFollow() {
        val payload = JSONObject()
        payload.put("action", "stop")

        val msg = """
        {
            "op": "publish",
            "topic": "/line_follow_cmd",
            "msg": { "data": ${JSONObject.quote(payload.toString())} }
        }
        """.trimIndent()

        linefollow_webSocket?.send(msg)
        isLineFollowing.value = false
        Log.i("LineFollow", "Stopped line following")
    }

    // ---- Speed Tuning ----

    fun adjustLinearSpeed(delta: Float) {
        val new = (maxLinearSpeed.floatValue + delta).coerceIn(0.1f, 2.0f)
        maxLinearSpeed.floatValue = Math.round(new * 10f) / 10f
    }

    fun adjustAngularSpeed(delta: Float) {
        val new = (maxAngularSpeed.floatValue + delta).coerceIn(0.1f, 3.0f)
        maxAngularSpeed.floatValue = Math.round(new * 10f) / 10f
    }

    // ---- Video Feed ----

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
            }
        }
    }

    // ---- Occupancy Map ----

    fun startOccupancyWebSocket() {
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
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        val paint = Paint()

        for (row in 0 until height) {
            for (col in 0 until width) {
                val index = row * width + col
                val value = dataArray.getInt(index)
                val color = when {
                    value == -1 -> Color.GRAY
                    value == 0 -> Color.WHITE
                    else -> Color.BLACK
                }
                paint.color = color
                canvas.drawPoint(col.toFloat(), row.toFloat(), paint)
            }
        }
        occupancyBitmap.value = bitmap
    }

    // ---- Config Persistence ----

    fun saveConfig(ipAddress: String, port: String, route: String) {
        val sharedPreferences = getApplication<Application>()
            .getSharedPreferences("CameraSetting", Context.MODE_PRIVATE)
        with(sharedPreferences.edit()) {
            putString("ipAddress", ipAddress)
            putString("port", port)
            putString("route", route)
            apply()
        }
        Log.i("CameraSettingsViewModel", "Config saved: $ipAddress:$port -> $route")
    }

    fun loadConfig(): CameraConfig? {
        val sharedPreferences = getApplication<Application>()
            .getSharedPreferences("CameraSetting", Context.MODE_PRIVATE)
        val ipAddress = sharedPreferences.getString("ipAddress", null)
        val port = sharedPreferences.getString("port", null)
        val route = sharedPreferences.getString("route", null)

        return if (ipAddress != null && port != null && route != null) {
            CameraConfig(ipAddress, port, route)
        } else {
            null
        }
    }

    override fun onCleared() {
        super.onCleared()
        // Cleanup websockets if needed
    }
}

// ---- Data Classes ----

data class WaypointEntry(
    val type: String,  // "nav" or "slope"
    val x: Double,
    val y: Double,
    val yaw: Double
)

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
