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
import com.chatgptlite.wanted.ui.settings.rover.RoverSettingsViewModel

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

    // Removed separate control_client / base_client — use shared `client` for all WebSockets
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

    // ---- Feedback Toast ----
    val toastMessage = mutableStateOf<String?>(null)
    val poseReceived = mutableStateOf(false)  // true once any /pose data arrives

    // ---- Connection Status Tracking ----
    // Each topic: false = connecting, true = connected
    val connectionStatus = mutableMapOf(
        "/cmd_vel" to mutableStateOf(false),
        "/e_stop" to mutableStateOf(false),
        "/pose" to mutableStateOf(false),
        "/nav_status" to mutableStateOf(false),
        "/goal_pose" to mutableStateOf(false),
        "/cancel_nav" to mutableStateOf(false),
        "/waypoint_replay" to mutableStateOf(false),
        "/line_follow_cmd" to mutableStateOf(false),
        "/map" to mutableStateOf(false)
    )
    val allConnected = mutableStateOf(false)

    private fun onTopicConnected(topic: String) {
        connectionStatus[topic]?.value = true
        Log.i("WS-Connect", "$topic connected")

        val total = connectionStatus.size
        val connected = connectionStatus.values.count { it.value }

        // Push partial connection progress to Status ViewModel
        if (connected == 1) {
            // First topic connected — rover is reachable
            statusViewModel?.updateConnectionState(false)
        }

        if (connected == total && !allConnected.value) {
            allConnected.value = true
            toastMessage.value = "All systems ready — $total/$total topics connected"
            Log.i("WS-Connect", "ALL $total topics connected!")
            // Notify Status ViewModel that all connections are ready
            statusViewModel?.updateConnectionState(true)
        }
    }

    private fun onTopicFailed(topic: String, error: String) {
        Log.e("WS-Connect", "$topic failed: $error")
    }

    // Speed limits (adjustable at runtime)
    val maxLinearSpeed = mutableFloatStateOf(1.0f)
    val maxAngularSpeed = mutableFloatStateOf(1.5f)

    // Home position
    private var homeX = 0.0
    private var homeY = 0.0
    private var homeYaw = 0.0
    val homeIsCustom = mutableStateOf(false)

    // Current robot pose (from /pose topic — shared with Status page)
    val currentRobotX = mutableStateOf(0.0)
    val currentRobotY = mutableStateOf(0.0)
    val currentRobotYaw = mutableStateOf(0.0)
    private var pose_webSocket: WebSocket? = null

    // Reference to Status ViewModel — set once from MainActivity to pipe pose data
    private var statusViewModel: RoverSettingsViewModel? = null

    fun linkStatusViewModel(vm: RoverSettingsViewModel) {
        statusViewModel = vm
    }

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
        loadWaypoints() // Restore saved waypoints so replay works immediately
        loadHome() // Restore saved home position
    }

    // ---- WebSocket Setup ----

    fun createWebSocket() {
        // Priority order: safety + driving first, then data, then autonomous modes
        val wsUrl = "ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT"

        // P1: /cmd_vel — joystick driving, must work immediately
        vel_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/cmd_vel",
                onMessageReceived = { Log.d("WebSocket", "cmd_vel received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P2: /e_stop — safety critical, must be ready before driving
        estop_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/e_stop",
                onMessageReceived = { Log.d("WebSocket", "e_stop received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P3: /pose — robot position for Set Home + Recording
        pose_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/pose",
                onMessageReceived = ::handlePoseMessage,
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P4: /nav_status — status strip feedback
        navstatus_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/nav_status",
                onMessageReceived = ::handleNavStatus,
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P5: /goal_pose — navigation goals (Go Home)
        base_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/goal_pose",
                onMessageReceived = { Log.d("WebSocket", "goal_pose received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P6: /cancel_nav — cancel navigation
        cancel_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/cancel_nav",
                onMessageReceived = { Log.d("WebSocket", "cancel_nav received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P7: /waypoint_replay — replay recorded paths
        waypoint_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/waypoint_replay",
                onMessageReceived = { Log.d("WebSocket", "waypoint_replay received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )

        // P8: /line_follow_cmd — line following
        linefollow_webSocket = client.newWebSocket(
            Request.Builder().url(wsUrl).build(),
            RoverWebSocketListener("/line_follow_cmd",
                onMessageReceived = { Log.d("WebSocket", "line_follow_cmd received") },
                onConnected = ::onTopicConnected,
                onFailed = ::onTopicFailed
            )
        )
    }

    // ---- Pose Tracking (from /pose topic) ----

    private fun handlePoseMessage(rawMessage: String) {
        try {
            val json = JSONObject(rawMessage)
            val poseObj = json.getJSONObject("msg")
                .getJSONObject("pose")
                .getJSONObject("pose")

            val x = poseObj.getJSONObject("position").getDouble("x")
            val y = poseObj.getJSONObject("position").getDouble("y")

            val orientation = poseObj.getJSONObject("orientation")
            val qx = orientation.getDouble("x")
            val qy = orientation.getDouble("y")
            val qz = orientation.getDouble("z")
            val qw = orientation.getDouble("w")

            // Convert quaternion to yaw
            val yaw = Math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )

            currentRobotX.value = x
            currentRobotY.value = y
            currentRobotYaw.value = yaw

            // Pipe pose data to Status page — eliminates duplicate /pose WebSocket
            statusViewModel?.updatePoseFromController(x, y, qz)

            if (!poseReceived.value) {
                poseReceived.value = true
                Log.i("Pose", "First pose received! x=${"%.2f".format(x)}, y=${"%.2f".format(y)}")
            }

            // Auto-record waypoint if recording is active
            if (isRecording.value) {
                recordWaypointTick(x, y, yaw)
            }

            Log.d("Pose", "x=${"%.2f".format(x)}, y=${"%.2f".format(y)}, yaw=${"%.1f".format(Math.toDegrees(yaw))}°")
        } catch (e: Exception) {
            Log.e("Pose", "Parse error: ${e.message}")
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

            // Pipe to Status page so it reflects real rover activity
            statusViewModel?.updateNavState(newStatus)

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
        waypoints.clear()
        waypointCount.value = 0
        isRecording.value = true
        recordLastTime = System.currentTimeMillis()
        toastMessage.value = "Recording started — drive the rover"
        Log.i("Waypoint", "Recording started")
    }

    fun stopRecording() {
        isRecording.value = false
        waypointCount.value = waypoints.size
        Log.i("Waypoint", "Recording stopped, ${waypoints.size} waypoints saved")
        saveWaypoints()
        toastMessage.value = "Recording saved: ${waypoints.size} waypoints"
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
                // Save as list-of-lists: [["nav", x, y, yaw], ...] — matches Python format
                val jsonArray = JSONArray()
                for (wp in waypoints) {
                    val item = JSONArray()
                    item.put(wp.type)
                    item.put(wp.x)
                    item.put(wp.y)
                    item.put(wp.yaw)
                    jsonArray.put(item)
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
                val item = array.get(i)
                if (item is JSONArray) {
                    // New format: ["nav", x, y, yaw]
                    waypoints.add(
                        WaypointEntry(
                            item.getString(0),
                            item.getDouble(1),
                            item.getDouble(2),
                            item.getDouble(3)
                        )
                    )
                } else if (item is JSONObject) {
                    // Legacy format: {"type":"nav","x":...,"y":...,"yaw":...}
                    waypoints.add(
                        WaypointEntry(
                            item.getString("type"),
                            item.getDouble("x"),
                            item.getDouble("y"),
                            item.getDouble("yaw")
                        )
                    )
                }
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
            toastMessage.value = "No waypoints to replay — record a path first"
            Log.w("Replay", "No waypoints to replay")
            return
        }

        // Find nearest nav waypoint to current position (matches Python script)
        val startIdx = findNearestNavIndex()

        // Build waypoints as list-of-lists: [["nav", x, y, yaw], ...]
        // This matches the Python script format that Pi5 expects
        val wpArray = JSONArray()
        for (wp in waypoints) {
            val item = JSONArray()
            item.put(wp.type)
            item.put(wp.x)
            item.put(wp.y)
            item.put(wp.yaw)
            wpArray.put(item)
        }
        val payload = JSONObject()
        payload.put("cmd", "start")           // Pi5 expects "cmd", NOT "action"
        payload.put("waypoints", wpArray)
        payload.put("start_index", startIdx)

        // TODO(replay): wire up the actual rosbridge publish + send for startReplay.
        // The previous body of this function was a paste from controlRover() — it
        // referenced undefined symbols (controlMessage, x, z_angle) and contained an
        // unbalanced """ that broke the file's lexer. That paste has been removed.
        // To finish: wrap `payload` in a rosbridge publish envelope (op, topic, msg)
        // and send via the appropriate websocket (likely a /replay/start topic).
        Log.w(
            "Replay",
            "startReplay() not fully implemented — payload built but not sent. Payload: $payload"
        )
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

    /** Set home to current robot position (from /pose) */
    fun setHomeFromCurrentPose() {
        val x = currentRobotX.value
        val y = currentRobotY.value
        val yaw = currentRobotYaw.value
        if (!poseReceived.value) {
            toastMessage.value = "No pose data yet — /pose topic not connected"
            Log.w("Nav", "Set Home failed: no /pose data received yet")
            return
        }
        homeX = x
        homeY = y
        homeYaw = yaw
        homeIsCustom.value = true
        // Persist home to SharedPreferences
        saveHome()
        toastMessage.value = "Home set: (${"%.2f".format(x)}, ${"%.2f".format(y)}, ${"%.1f".format(Math.toDegrees(yaw))}°)"
        Log.i("Nav", "Home set to current pose ($homeX, $homeY, ${Math.toDegrees(homeYaw)}deg)")
    }

    fun resetHome() {
        homeX = 0.0
        homeY = 0.0
        homeYaw = 0.0
        homeIsCustom.value = false
        val prefs = getApplication<Application>()
            .getSharedPreferences("HomePosition", Context.MODE_PRIVATE)
        prefs.edit().clear().apply()
        toastMessage.value = "Home reset to origin (0, 0)"
        Log.i("Nav", "Home reset to map origin (0, 0, 0°)")
    }

    private fun saveHome() {
        val prefs = getApplication<Application>()
            .getSharedPreferences("HomePosition", Context.MODE_PRIVATE)
        prefs.edit()
            .putFloat("homeX", homeX.toFloat())
            .putFloat("homeY", homeY.toFloat())
            .putFloat("homeYaw", homeYaw.toFloat())
            .putBoolean("homeIsCustom", true)
            .apply()
        Log.i("Nav", "Home saved to prefs")
    }

    private fun loadHome() {
        val prefs = getApplication<Application>()
            .getSharedPreferences("HomePosition", Context.MODE_PRIVATE)
        if (prefs.getBoolean("homeIsCustom", false)) {
            homeX = prefs.getFloat("homeX", 0f).toDouble()
            homeY = prefs.getFloat("homeY", 0f).toDouble()
            homeYaw = prefs.getFloat("homeYaw", 0f).toDouble()
            homeIsCustom.value = true
            Log.i("Nav", "Loaded home from prefs: ($homeX, $homeY)")
        }
    }

    fun navigateHome() {
        if (!homeIsCustom.value && homeX == 0.0 && homeY == 0.0) {
            toastMessage.value = "Navigating to origin (0, 0) — no custom home set"
        } else {
            toastMessage.value = "Navigating home (${"%.2f".format(homeX)}, ${"%.2f".format(homeY)})"
        }
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
        // Publish cancel_nav (std_msgs/Empty via rosbridge)
        val cancelMsg = """
        {
            "op": "publish",
            "topic": "/cancel_nav",
            "msg": {}
        }
        """.trimIndent()
        cancel_webSocket?.send(cancelMsg)

        // Also send zero velocity immediately
        controlRover(0.0, 0.0)

        // Stop replay on Pi5 (matches Python: json.dumps({"cmd": "stop"}))
        if (isReplaying.value) stopReplay()

        // Stop line following on Pi5
        if (isLineFollowing.value) stopLineFollow()

        navStatus.value = "idle"
        toastMessage.value = "All navigation cancelled"
        Log.i("Nav", "All navigation cancelled")
    }

    // ---- Waypoint Recording ----

    fun startRecording() {
        if (!poseReceived.value) {
            toastMessage.value = "Cannot record — no pose data from /pose topic"
            Log.w("Waypoint", "Record failed: no /pose data received yet")
            return
        }
        waypoints.clear()
        waypointCount.value = 0
        isRecording.value = true
        recordLastTime = System.currentTimeMillis()
        toastMessage.value = "Recording started — drive the rover"
        Log.i("Waypoint", "Recording started")
    }

    fun stopRecording() {
        isRecording.value = false
        waypointCount.value = waypoints.size
        Log.i("Waypoint", "Recording stopped, ${waypoints.size} waypoints saved")
        saveWaypoints()
        toastMessage.value = "Recording saved: ${waypoints.size} waypoints"
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
                // Save as list-of-lists: [["nav", x, y, yaw], ...] — matches Python format
                val jsonArray = JSONArray()
                for (wp in waypoints) {
                    val item = JSONArray()
                    item.put(wp.type)
                    item.put(wp.x)
                    item.put(wp.y)
                    item.put(wp.yaw)
                    jsonArray.put(item)
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
                val item = array.get(i)
                if (item is JSONArray) {
                    // New format: ["nav", x, y, yaw]
                    waypoints.add(
                        WaypointEntry(
                            item.getString(0),
                            item.getDouble(1),
                            item.getDouble(2),
                            item.getDouble(3)
                        )
                    )
                } else if (item is JSONObject) {
                    // Legacy format: {"type":"nav","x":...,"y":...,"yaw":...}
                    waypoints.add(
                        WaypointEntry(
                            item.getString("type"),
                            item.getDouble("x"),
                            item.getDouble("y"),
                            item.getDouble("yaw")
                        )
                    )
                }
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
            toastMessage.value = "No waypoints to replay — record a path first"
            Log.w("Replay", "No waypoints to replay")
            return
        }

        // Find nearest nav waypoint to current position (matches Python script)
        val startIdx = findNearestNavIndex()

        // Build waypoints as list-of-lists: [["nav", x, y, yaw], ...]
        // This matches the Python script format that Pi5 expects
        val wpArray = JSONArray()
        for (wp in waypoints) {
            val item = JSONArray()
            item.put(wp.type)
            item.put(wp.x)
            item.put(wp.y)
            item.put(wp.yaw)
            wpArray.put(item)
        }
        val payload = JSONObject()
        payload.put("cmd", "start")           // Pi5 expects "cmd", NOT "action"
        payload.put("waypoints", wpArray)
        payload.put("start_index", startIdx)

        val msg = """
        {
            "op": "publish",
            "topic": "/waypoint_replay",
            "msg": { "data": ${JSONObject.quote(payload.toString())} }
        }
        """.trimIndent()

        waypoint_webSocket?.send(msg)
        isReplaying.value = true
        toastMessage.value = "Replay started from WP ${startIdx + 1}/${waypoints.size}"
        Log.i("Replay", "Started replay from WP ${startIdx + 1}/${waypoints.size}")
    }

    private fun stopReplay() {
        val payload = JSONObject()
        payload.put("cmd", "stop")            // Pi5 expects "cmd", NOT "action"

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

    /** Find the nearest nav waypoint to current robot position (matches Python script) */
    private fun findNearestNavIndex(): Int {
        val rx = currentRobotX.value
        val ry = currentRobotY.value
        if (!poseReceived.value) return 0

        var bestIdx = 0
        var bestDist = Double.MAX_VALUE
        for (i in waypoints.indices) {
            val wp = waypoints[i]
            if (wp.type == "nav") {
                val dx = wp.x - rx
                val dy = wp.y - ry
                val dist = Math.sqrt(dx * dx + dy * dy)
                if (dist < bestDist) {
                    bestDist = dist
                    bestIdx = i
                }
            }
        }
        return bestIdx
    }

    // ---- Line Following ----

    fun toggleLineFollow() {
        if (isLineFollowing.value) stopLineFollow() else startLineFollow()
    }

    private fun startLineFollow() {
        val payload = JSONObject()
        payload.put("cmd", "start")           // Pi5 expects "cmd", NOT "action"

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
        payload.put("cmd", "stop")            // Pi5 expects "cmd", NOT "action"

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

    // P9: /map — occupancy map (lowest priority, visual only)
    fun startOccupancyWebSocket() {
        val topic = "/map"
        val request = Request.Builder()
            .url("ws://$WEBSOCKET_IPADDRESS:$WEBSOCKET_PORT")
            .build()
        Log.d("Occupancy", "Start websocket")
        val listener = RoverWebSocketListener(topic,
            onMessageReceived = { message ->
                try {
                    val jsonObject = JSONObject(message)
                    val dataArray = jsonObject.getJSONObject("msg").getJSONArray("data")
                    val width = jsonObject.getJSONObject("msg").getJSONObject("info").getInt("width")
                    val height = jsonObject.getJSONObject("msg").getJSONObject("info").getInt("height")
                    updateOccupancyBitmap(dataArray, width, height)
                } catch (e: JSONException) {
                    Log.e("WebSocket", "JSON parsing error: ${e.message}")
                }
            },
            onConnected = ::onTopicConnected,
            onFailed = ::onTopicFailed
        )

        client.newWebSocket(request, listener)
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
