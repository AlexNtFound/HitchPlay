package com.chatgptlite.wanted.ui.settings.controller

import android.graphics.Bitmap
import android.util.Log
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.requiredWidthIn
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.wrapContentHeight
import androidx.compose.foundation.layout.wrapContentWidth
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Divider
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import io.github.controlwear.virtual.joystick.android.JoystickView
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import kotlinx.coroutines.delay


@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun VideoStreamingSetting(
    viewModel: VideoCamSettingsViewModel,
    onBackPressed: () -> Unit,
    onOccMapClick: () -> Unit = {}
) {
    var ipAddress by remember { mutableStateOf("10.0.0.1") }
    var port by remember { mutableStateOf("8080") }
    var route by remember { mutableStateOf("/stream?topic=/camera/image_raw&type=ros_compressed") }

    // Config loaded for local display only; feeds/websockets initialized once in MainActivity
    LaunchedEffect(Unit) {
        val config = viewModel.loadConfig()
        config?.let {
            ipAddress = it.ipAddress
            port = it.port
            route = it.route
        }
    }

    val maxLinSpeed = viewModel.maxLinearSpeed.floatValue
    val maxAngSpeed = viewModel.maxAngularSpeed.floatValue

    // ---- Toast / Snackbar feedback ----
    val snackbarHostState = remember { SnackbarHostState() }
    val toast = viewModel.toastMessage.value
    LaunchedEffect(toast) {
        if (toast != null) {
            snackbarHostState.showSnackbar(toast)
            viewModel.toastMessage.value = null
        }
    }

    Scaffold(
        snackbarHost = { SnackbarHost(snackbarHostState) }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 16.dp, vertical = 4.dp)
        ) {
            // 1. Video Feed
            VideoFeedDisplay(viewModel.currentFrame.value, viewModel.occupancyBitmap.value, onOccMapClick)

            Spacer(modifier = Modifier.height(4.dp))

            // 2. Status Strip (now includes pose + home info)
            StatusStrip(viewModel)

            Spacer(modifier = Modifier.height(4.dp))

            // 3. E-Stop
            EStopButton(viewModel)

            Spacer(modifier = Modifier.height(4.dp))

            // 4. Navigation Row
            SectionLabel("Navigation")
            Spacer(modifier = Modifier.height(2.dp))
            NavigationRow(viewModel)

            Spacer(modifier = Modifier.height(4.dp))

            // 5. Record and Autonomy Row
            SectionLabel("Record & Autonomy")
            Spacer(modifier = Modifier.height(2.dp))
            AutonomyRow(viewModel)

            Spacer(modifier = Modifier.height(4.dp))

            // 6. Speed Tuning (above joysticks so mic FAB doesn't block it)
            SpeedTuningRow(viewModel)

            Spacer(modifier = Modifier.height(4.dp))

            // 7. Joysticks — fills remaining space
            JoystickRow(
                viewModel, maxLinSpeed, maxAngSpeed,
                modifier = Modifier.weight(1f)
            )
        }
    }
}


// ==================== Status Strip ====================

@Composable
fun StatusStrip(viewModel: VideoCamSettingsViewModel) {
    val status = viewModel.navStatus.value
    val mode = viewModel.navMode.value
    val poseOk = viewModel.poseReceived.value
    val robotX = viewModel.currentRobotX.value
    val robotY = viewModel.currentRobotY.value
    val homeSet = viewModel.homeIsCustom.value
    val wpCount = viewModel.waypointCount.value
    val ready = viewModel.allConnected.value

    // Count connected topics
    val total = viewModel.connectionStatus.size
    val connected = viewModel.connectionStatus.values.count { it.value }

    val statusDotColor = when (status) {
        "idle", "arrived" -> Color(0xFF4CAF50)
        "navigating", "rotating", "driving" -> Color(0xFFFFEB3B)
        "blocked", "failed", "timeout" -> Color(0xFFF44336)
        "replaying", "line_following" -> Color(0xFF2196F3)
        else -> Color.White
    }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(
                color = Color(0xFF2A2E2E),
                shape = RoundedCornerShape(8.dp)
            )
            .padding(horizontal = 10.dp, vertical = 4.dp)
    ) {
        // Row 1: Connection progress or Nav status
        if (!ready) {
            // Still connecting — show progress
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(
                    modifier = Modifier
                        .size(8.dp)
                        .background(Color(0xFFFFEB3B), shape = RoundedCornerShape(4.dp))
                )
                Spacer(modifier = Modifier.width(4.dp))
                Text(
                    text = "Connecting $connected/$total",
                    color = Color(0xFFFFEB3B),
                    fontSize = 10.sp
                )
                Spacer(modifier = Modifier.weight(1f))
                // Show which topics are connected as dots
                viewModel.connectionStatus.forEach { (topic, state) ->
                    val shortName = topic.removePrefix("/").take(3)
                    val dotColor = if (state.value) Color(0xFF4CAF50) else Color(0xFF555555)
                    Box(
                        modifier = Modifier
                            .size(6.dp)
                            .background(dotColor, shape = RoundedCornerShape(3.dp))
                    )
                    Spacer(modifier = Modifier.width(2.dp))
                }
            }
        } else {
            // All connected — show normal nav status
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Box(
                    modifier = Modifier
                        .size(8.dp)
                        .background(statusDotColor, shape = RoundedCornerShape(4.dp))
                )
                Spacer(modifier = Modifier.width(4.dp))
                Text(
                    text = status.replaceFirstChar { it.uppercase() },
                    color = Color.White,
                    fontSize = 10.sp
                )
                Spacer(modifier = Modifier.width(10.dp))
                Text(
                    text = mode.uppercase(),
                    color = Color(0xFF64B5F6),
                    fontSize = 10.sp
                )
                Spacer(modifier = Modifier.weight(1f))
                Text(
                    text = if (homeSet) "Home: SET" else "Home: --",
                    color = if (homeSet) Color(0xFF4CAF50) else Color(0xFF888888),
                    fontSize = 9.sp
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "WP: $wpCount",
                    color = if (wpCount > 0) Color(0xFF64B5F6) else Color(0xFF888888),
                    fontSize = 9.sp
                )
            }
        }

        // Row 2: Pose data
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            val poseDotColor = if (poseOk) Color(0xFF4CAF50) else Color(0xFFF44336)
            Box(
                modifier = Modifier
                    .size(6.dp)
                    .background(poseDotColor, shape = RoundedCornerShape(3.dp))
            )
            Spacer(modifier = Modifier.width(4.dp))
            Text(
                text = if (poseOk) "Pose: (${"%.2f".format(robotX)}, ${"%.2f".format(robotY)})"
                       else "Pose: waiting for /pose...",
                color = if (poseOk) Color(0xFFCCCCCC) else Color(0xFFFF8A80),
                fontSize = 9.sp
            )
        }
    }
}


// ==================== E-Stop Button ====================

@Composable
fun EStopButton(viewModel: VideoCamSettingsViewModel) {
    val isActive = viewModel.eStopActive.value

    val bgColor = if (isActive) Color(0xFFF44336).copy(alpha = 0.6f)
    else Color(0xFFF44336).copy(alpha = 0.15f)

    val borderColor = if (isActive) Color(0xFFF44336)
    else Color(0xFFF44336).copy(alpha = 0.5f)

    val textColor = if (isActive) Color.White else Color(0xFFEF5350)

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .background(bgColor, shape = RoundedCornerShape(8.dp))
            .border(1.dp, borderColor, RoundedCornerShape(8.dp))
            .clickable { viewModel.toggleEStop() }
            .padding(vertical = 8.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = if (isActive) "E-STOP ACTIVE" else "E-STOP",
            color = textColor,
            fontWeight = FontWeight.Bold,
            style = MaterialTheme.typography.bodyLarge
        )
    }
}


// ==================== Navigation Row ====================

@Composable
fun NavigationRow(viewModel: VideoCamSettingsViewModel) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        // Set Home
        ControllerActionButton(
            label = "Set Home",
            iconText = "Pin",
            borderColor = Color(0xFF2196F3).copy(alpha = 0.4f),
            bgColor = Color(0xFF2196F3).copy(alpha = 0.1f),
            textColor = Color(0xFF64B5F6),
            onClick = { viewModel.setHomeFromCurrentPose() },
            modifier = Modifier.weight(1f)
        )
        // Go Home
        ControllerActionButton(
            label = "Go Home",
            iconText = "Home",
            borderColor = Color(0xFF2196F3).copy(alpha = 0.4f),
            bgColor = Color(0xFF2196F3).copy(alpha = 0.1f),
            textColor = Color(0xFF64B5F6),
            onClick = { viewModel.navigateHome() },
            modifier = Modifier.weight(1f)
        )
        // Reset Home
        ControllerActionButton(
            label = "Reset",
            iconText = "Reset",
            borderColor = Color(0xFF2196F3).copy(alpha = 0.4f),
            bgColor = Color(0xFF2196F3).copy(alpha = 0.1f),
            textColor = Color(0xFF64B5F6),
            onClick = { viewModel.resetHome() },
            modifier = Modifier.weight(1f)
        )
        // Cancel
        ControllerActionButton(
            label = "Cancel",
            iconText = "X",
            borderColor = Color(0xFFFF9800).copy(alpha = 0.4f),
            bgColor = Color(0xFFFF9800).copy(alpha = 0.08f),
            textColor = Color(0xFFFFB74D),
            onClick = { viewModel.cancelAll() },
            modifier = Modifier.weight(1f)
        )
    }
}


// ==================== Autonomy Row ====================

@Composable
fun AutonomyRow(viewModel: VideoCamSettingsViewModel) {
    val recording = viewModel.isRecording.value
    val replaying = viewModel.isReplaying.value
    val lineFollowing = viewModel.isLineFollowing.value
    val wpCount = viewModel.waypointCount.value

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        // Record toggle
        ControllerActionButton(
            label = if (recording) "Stop Rec" else "Record",
            iconText = if (recording) "||" else "Rec",
            borderColor = if (recording) Color(0xFFF44336) else Color(0xFFF44336).copy(alpha = 0.4f),
            bgColor = if (recording) Color(0xFFF44336).copy(alpha = 0.35f)
            else Color(0xFFF44336).copy(alpha = 0.08f),
            textColor = if (recording) Color.White else Color(0xFFEF9A9A),
            onClick = { viewModel.toggleRecording() },
            modifier = Modifier.weight(1f)
        )
        // Replay toggle
        ControllerActionButton(
            label = if (replaying) "Stop" else "Replay ($wpCount)",
            iconText = if (replaying) "||" else "Play",
            borderColor = if (replaying) Color(0xFF4CAF50) else Color(0xFF4CAF50).copy(alpha = 0.4f),
            bgColor = if (replaying) Color(0xFF4CAF50).copy(alpha = 0.35f)
            else Color(0xFF4CAF50).copy(alpha = 0.08f),
            textColor = if (replaying) Color.White else Color(0xFF81C784),
            onClick = { viewModel.toggleReplay() },
            modifier = Modifier.weight(1f)
        )
        // Line Follow toggle
        ControllerActionButton(
            label = if (lineFollowing) "Stop LF" else "Line Follow",
            iconText = if (lineFollowing) "||" else "LF",
            borderColor = if (lineFollowing) Color(0xFF4CAF50)
            else Color(0xFF4CAF50).copy(alpha = 0.4f),
            bgColor = if (lineFollowing) Color(0xFF4CAF50).copy(alpha = 0.35f)
            else Color(0xFF4CAF50).copy(alpha = 0.08f),
            textColor = if (lineFollowing) Color.White else Color(0xFF81C784),
            onClick = { viewModel.toggleLineFollow() },
            modifier = Modifier.weight(1f)
        )
    }
}


// ==================== Joystick Row ====================

@Composable
fun JoystickRow(
    viewModel: VideoCamSettingsViewModel,
    maxLinSpeed: Float,
    maxAngSpeed: Float,
    modifier: Modifier = Modifier
) {
    val primaryColorInt = MaterialTheme.colorScheme.primary.toArgb()

    Row(
        modifier = modifier
            .fillMaxWidth()
            .padding(horizontal = 2.dp),
        horizontalArrangement = Arrangement.SpaceEvenly
    ) {
        // Horizontal joystick (rotation)
        Box(
            contentAlignment = Alignment.Center,
            modifier = Modifier
                .weight(1f)
                .fillMaxHeight()
        ) {
            AndroidView(
                factory = { context ->
                    JoystickView(context).apply {
                        setButtonColor(primaryColorInt)
                        setBorderColor(primaryColorInt)
                        setBackgroundColor(android.graphics.Color.BLACK)
                        setButtonDirection(-1)
                        setFixedCenter(true)
                        setButtonSizeRatio(0.2f)
                        setOnMoveListener { angle, strength ->
                            val angVel = maxAngSpeed * strength / 100.0
                            if (angle == 180) {
                                viewModel.controlRover(0.0, angVel)
                            } else {
                                viewModel.controlRover(0.0, -angVel)
                            }
                        }
                    }
                },
                modifier = Modifier.fillMaxSize()
            )
            Row(
                modifier = Modifier
                    .wrapContentWidth()
                    .padding(horizontal = 10.dp),
                horizontalArrangement = Arrangement.spacedBy(70.dp)
            ) {
                Text("◀", color = MaterialTheme.colorScheme.primary,
                    fontSize = 24.sp,
                    modifier = Modifier.align(Alignment.CenterVertically))
                Text("▶", color = MaterialTheme.colorScheme.primary,
                    fontSize = 24.sp,
                    modifier = Modifier.align(Alignment.CenterVertically))
            }
        }

        // Vertical joystick (drive)
        Box(
            contentAlignment = Alignment.Center,
            modifier = Modifier
                .weight(1f)
                .fillMaxHeight()
        ) {
            AndroidView(
                factory = { context ->
                    JoystickView(context).apply {
                        setButtonColor(primaryColorInt)
                        setBorderColor(primaryColorInt)
                        setBackgroundColor(android.graphics.Color.BLACK)
                        setButtonDirection(1)
                        setFixedCenter(true)
                        setButtonSizeRatio(0.2f)
                        setOnMoveListener { angle, strength ->
                            val vel = maxLinSpeed * strength / 100.0
                            if (angle == 90) {
                                viewModel.controlRover(vel, 0.0)
                            } else if (angle == 270) {
                                viewModel.controlRover(-vel, 0.0)
                            }
                        }
                    }
                },
                modifier = Modifier.fillMaxSize()
            )
            Column(
                modifier = Modifier
                    .wrapContentHeight()
                    .padding(vertical = 10.dp),
                verticalArrangement = Arrangement.spacedBy(70.dp)
            ) {
                Text("▲", color = MaterialTheme.colorScheme.primary,
                    fontSize = 24.sp, textAlign = TextAlign.Center,
                    modifier = Modifier.fillMaxWidth())
                Text("▼", color = MaterialTheme.colorScheme.primary,
                    fontSize = 24.sp, textAlign = TextAlign.Center,
                    modifier = Modifier.fillMaxWidth())
            }
        }
    }
}


// ==================== Speed Tuning Row ====================

@Composable
fun SpeedTuningRow(viewModel: VideoCamSettingsViewModel) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically
    ) {
        // Linear speed controls
        Text("Lin", color = Color(0xFFAAAAAA), fontSize = 11.sp)
        Spacer(modifier = Modifier.width(6.dp))
        SpeedAdjustButton("-") { viewModel.adjustLinearSpeed(-0.1f) }
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = "%.1f".format(viewModel.maxLinearSpeed.floatValue),
            color = MaterialTheme.colorScheme.primary,
            fontSize = 12.sp,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.width(32.dp),
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.width(4.dp))
        SpeedAdjustButton("+") { viewModel.adjustLinearSpeed(0.1f) }

        Spacer(modifier = Modifier.width(20.dp))

        // Angular speed controls
        Text("Ang", color = Color(0xFFAAAAAA), fontSize = 11.sp)
        Spacer(modifier = Modifier.width(6.dp))
        SpeedAdjustButton("-") { viewModel.adjustAngularSpeed(-0.1f) }
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = "%.1f".format(viewModel.maxAngularSpeed.floatValue),
            color = MaterialTheme.colorScheme.primary,
            fontSize = 12.sp,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.width(32.dp),
            textAlign = TextAlign.Center
        )
        Spacer(modifier = Modifier.width(4.dp))
        SpeedAdjustButton("+") { viewModel.adjustAngularSpeed(0.1f) }
    }
}


// ==================== Reusable Components ====================

@Composable
fun SectionLabel(text: String) {
    Text(
        text = text.uppercase(),
        color = Color(0xFF555555),
        fontSize = 9.sp,
        fontWeight = FontWeight.Normal,
        letterSpacing = 1.5.sp,
        textAlign = TextAlign.Center,
        modifier = Modifier.fillMaxWidth()
    )
}

@Composable
fun ControllerActionButton(
    label: String,
    iconText: String,
    borderColor: Color,
    bgColor: Color,
    textColor: Color,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .background(bgColor, shape = RoundedCornerShape(8.dp))
            .border(1.dp, borderColor, RoundedCornerShape(8.dp))
            .clickable { onClick() }
            .padding(vertical = 6.dp, horizontal = 4.dp),
        contentAlignment = Alignment.Center
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = iconText,
                color = textColor,
                fontSize = 14.sp,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(2.dp))
            Text(
                text = label,
                color = textColor,
                fontSize = 10.sp,
                textAlign = TextAlign.Center
            )
        }
    }
}

@Composable
fun SpeedAdjustButton(symbol: String, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .size(28.dp)
            .background(
                color = Color.White.copy(alpha = 0.06f),
                shape = RoundedCornerShape(6.dp)
            )
            .border(1.dp, Color.White.copy(alpha = 0.15f), RoundedCornerShape(6.dp))
            .clickable { onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = symbol,
            color = Color.White,
            fontSize = 16.sp,
            fontWeight = FontWeight.Normal
        )
    }
}


// ==================== Video Feed Display ====================

@Composable
fun VideoFeedDisplay(videoBitmap: Bitmap?, occupancyBitmap: Bitmap?, onOccMapClick: () -> Unit = {}) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(160.dp)
            .clip(RoundedCornerShape(8.dp))
            .background(Color.Black)
    ) {
        if (videoBitmap != null) {
            Image(
                bitmap = videoBitmap.asImageBitmap(),
                contentDescription = "Video Feed",
                modifier = Modifier.fillMaxSize()
            )
        } else {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .background(Color.Gray),
                contentAlignment = Alignment.Center
            ) {
                Text("Video Feed Placeholder", color = Color.White)
            }
        }

        // Occ map mini overlay — tappable to expand
        if (occupancyBitmap != null) {
            Image(
                bitmap = occupancyBitmap.asImageBitmap(),
                contentDescription = "Occupancy Map",
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .size(80.dp)
                    .padding(8.dp)
                    .graphicsLayer(alpha = 0.7f)
                    .clip(RoundedCornerShape(8.dp))
                    .clickable { onOccMapClick() }
            )
        } else {
            Box(
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .size(80.dp)
                    .padding(8.dp)
                    .background(Color.Red.copy(alpha = 0.7f))
                    .clip(RoundedCornerShape(8.dp))
                    .clickable { onOccMapClick() },
                contentAlignment = Alignment.Center
            ) {
                Text("Occ Map", color = Color.White, fontSize = 8.sp)
            }
        }
    }
}


// ==================== Gradient Button (kept for compatibility) ====================

@Composable
fun GradientButton(
    text: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .background(
                brush = Brush.linearGradient(
                    listOf(
                        Color(0xFFAFFD86).copy(alpha = 0.25f),
                        Color(0xFF7A8A80).copy(alpha = 0.25f)
                    )
                ),
                shape = RoundedCornerShape(8.dp)
            )
            .border(
                width = 1.dp,
                color = Color.White.copy(alpha = 0.5f),
                shape = RoundedCornerShape(8.dp)
            )
            .clickable { onClick() }
            .padding(horizontal = 24.dp, vertical = 12.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            color = MaterialTheme.colorScheme.primary,
            style = MaterialTheme.typography.bodyLarge
        )
    }
}
