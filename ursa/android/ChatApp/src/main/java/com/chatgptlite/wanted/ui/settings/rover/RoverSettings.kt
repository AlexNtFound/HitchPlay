package com.chatgptlite.wanted.ui.settings.rover

import android.graphics.Bitmap
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.Canvas
import androidx.compose.ui.graphics.*
import androidx.lifecycle.viewmodel.compose.viewModel
import com.chatgptlite.wanted.MainViewModel
import com.chatgptlite.wanted.ui.common.AppBar

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SettingsScreen(
    mainViewModel: MainViewModel = viewModel(),
    viewModel: RoverSettingsViewModel = viewModel(),
    onBackPressed: () -> Unit
) {
    LaunchedEffect(Unit) {
        viewModel.syncWithMainViewModel(mainViewModel) // Sync states
    }

    val roverState by viewModel.roverState.collectAsState()

    var ipAddress by remember { mutableStateOf("10.0.0.1") }
    var port by remember { mutableStateOf("8000") }
    var textToSend by remember { mutableStateOf("ros2 topic list") }

//    val xCoordinate by viewModel.xCoordinate.collectAsState()
//    val yCoordinate by viewModel.yCoordinate.collectAsState()
//    val heading by viewModel.heading.collectAsState()
//    val linearVelocity by viewModel.linear_velocity.collectAsState()
//    val angularVelocity by viewModel.angular_velocity.collectAsState()
//    val battery by viewModel.battery.collectAsState()

    val xCoordinate by viewModel.xCoordinate.collectAsState() // Collecting the state
    val yCoordinate by viewModel.yCoordinate.collectAsState()
    val zCoordinate by viewModel.zCoordinate.collectAsState()
    val linear_velocity by viewModel.linear_velocity.collectAsState()
    val angular_velocity by viewModel.angular_velocity.collectAsState()
    val heading by viewModel.heading.collectAsState()
    val battery by viewModel.battery.collectAsState()

    LaunchedEffect(Unit) {
        val config = viewModel.loadConfig()
        config?.let {
            ipAddress = it.ipAddress
            port = it.port
            textToSend = it.textToSend
        }

        viewModel.startCoordinatesWebSocket()
        viewModel.startVelocityWebSocket()
        viewModel.startBatteryWebSocket()
        viewModel.startOccupancyWebSocket()
        viewModel.receiveFeed("10.0.0.1", "8080", "/stream?topic=/camera/image_raw&type=ros_compressed") //different port number and IP Address
    }

    Scaffold(
//        topBar = {
//            CenterAlignedTopAppBar(
//                title = {
//                    Text(
//                        text = "Status",
//                        fontWeight = FontWeight.Bold,
//                        textAlign = TextAlign.Center,
//                        color = MaterialTheme.colorScheme.primary,
//                    )
//                },
//                modifier = Modifier.height(80.dp).padding(top = 6.dp),
//                colors = TopAppBarDefaults.topAppBarColors(
//                    containerColor = MaterialTheme.colorScheme.background
//                )
//            )
//        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 16.dp)
                //.verticalScroll(rememberScrollState())
        ) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .aspectRatio(16 / 9f)
                    .background(Color.Black, shape = RoundedCornerShape(8.dp))
            ) {
                VideoFeedDisplay(viewModel.currentFrame.value)
            }

            Spacer(modifier = Modifier.height(8.dp))

            Row(
                modifier = Modifier
                    .fillMaxSize()
                    .weight(1f),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                // Left Column: Telemetry Details
                Column(
                    modifier = Modifier
                        .weight(1f)
                        .padding(end = 8.dp)
                ) {
                    TelemetryBlock("State", roverState, isState = true) // Updated state
                    Spacer(modifier = Modifier.height(8.dp))
                    TelemetryBlock("Battery", "$battery%")
                    Spacer(modifier = Modifier.height(8.dp))
                    TelemetryBlock("Linear Velocity (x)", "$linear_velocity")
                    Spacer(modifier = Modifier.height(8.dp))
                    TelemetryBlock("Angular Velocity (z)", "$angular_velocity")
                }

                // Right Column: Occupancy Map & Battery
                Column(
                    modifier = Modifier
                        .weight(1f)
                        .padding(start = 8.dp)
                ) {
                    OccupancyDisplay(viewModel.occupancyBitmap.value)
                    Spacer(modifier = Modifier.height(8.dp))
                    TelemetryBlock("Coordinates (X, Y)", "$xCoordinate, $yCoordinate")
                    Spacer(modifier = Modifier.height(8.dp))
                    TelemetryBlock("Heading (W)", "$heading")
                }
            }
        }
    }
}

@Composable
fun TelemetryBlock(title: String, value: String? = null, isState: Boolean? = false) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(
                brush = Brush.linearGradient(
                    listOf(
                        Color(0xFFAFFD86).copy(alpha = 0.25f),
                        Color(0xFF7A8A80).copy(alpha = 0.25f)
                    )
                ),
                shape = RoundedCornerShape(8.dp),
            )
    ) {
        if (value != null && isState == false) {
            Text(
                text = title,
                color = Color.White,
                fontSize = 12.sp,
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(vertical = 8.dp)
            )
            Divider(color = Color.White, thickness = 0.5.dp, modifier = Modifier.fillMaxWidth())

            Text(
                text = value,
                fontWeight = FontWeight.Bold,
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.primary,
                textAlign = TextAlign.Center,
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(vertical = 12.dp)
            )
        } else if (isState == true) {
            // Display state text with proper color
            val stateColor = when (value) {
                "Executing" -> Color(0xFFFFD700) // Yellow
                "Successful" -> Color(0xFF4CAF50) // Green
                else -> Color.White // Default Green for Idle
            }

            Text(
                text = title,
                color = Color.White,
                fontSize = 12.sp,
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
                    .padding(vertical = 8.dp)
            )
            Divider(color = Color.White, thickness = 0.5.dp, modifier = Modifier.fillMaxWidth())

            Text(
                text = value ?: "Unknown",
                fontWeight = FontWeight.Bold,
                fontSize = 15.sp,
                style = MaterialTheme.typography.bodyLarge,
                color = stateColor,
                textAlign = TextAlign.Center,
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)
            )

            Text(
                text = when (value) {
                    "Idle" -> "Rover is ready"
                    "Executing" -> "Rover is processing"
                    "Successful" -> "Command completed"
                    else -> ""
                },
                fontSize = 12.sp, // Smaller description text
                color = Color.White,
                modifier = Modifier
                    .align(Alignment.CenterHorizontally)

            )

            Spacer(modifier = Modifier.height(12.dp))

        }
    }
}


//
//@Composable
//fun TelemetryBlock(title: String, value: String? = null, isState: Boolean? = false) {
//    Column(
//        modifier = Modifier
//            .fillMaxWidth()
//            .background(
//                brush = Brush.linearGradient(
//                    listOf(
//                        Color(0xFFAFFD86).copy(alpha = 0.25f),
//                        Color(0xFF7A8A80).copy(alpha = 0.25f)
//                    )),
//                shape = RoundedCornerShape(8.dp),
//            )
//    ) {
//        if (value != null && isState == false) {
//        Text(
//            text = title,
//            color = Color.White,
//            fontSize = 12.sp,
//            modifier = Modifier
//                .align(Alignment.CenterHorizontally)
//                .padding(vertical = 8.dp)
//        )
//        Divider(color = Color.White, thickness = 0.5.dp, modifier = Modifier.fillMaxWidth())
//
//            Text(
//                text = value,
//                fontWeight = FontWeight.Bold,
//                style = MaterialTheme.typography.bodyLarge,
//                color = MaterialTheme.colorScheme.primary,
//                textAlign = TextAlign.Center,
//                modifier = Modifier.align(Alignment.CenterHorizontally).padding(vertical = 12.dp)
//            )
//        } else {
//            // To Lisa: 改这里显示state文字
//        }
//    }
//}

@Composable
fun OccupancyDisplay(bitmap: Bitmap?) {
    Box(
        modifier = Modifier
            .aspectRatio(1f)
            .border(1.dp, color = MaterialTheme.colorScheme.primary.copy(alpha=0.3f), shape = RoundedCornerShape(8.dp))
            .background(Color.Black, shape = RoundedCornerShape(8.dp)),
        contentAlignment = Alignment.Center
    ) {
        if (bitmap != null) {
            Image(
                bitmap = bitmap.asImageBitmap(),
                contentDescription = "Video Feed",
                modifier = Modifier.fillMaxSize()
            )
        } else {
            Text(
                text = "Loading occupancy map...",
                modifier = Modifier.align(Alignment.Center),
                textAlign = TextAlign.Center, // Centers text horizontally
            )
        }
    }
}

@Composable
fun VideoFeedDisplay(bitmap: Bitmap?) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(300.dp)
            .background(Color.Black, shape = RoundedCornerShape(8.dp))
    ) {
        if (bitmap != null) {
            Image(
                bitmap = bitmap.asImageBitmap(),
                contentDescription = "Video Feed",
                modifier = Modifier.fillMaxSize()
            )
        } else {
            Text(
                text = "No video feed",
                modifier = Modifier.align(Alignment.Center)
            )
        }
    }
}

@Composable
fun BatteryDisplay(batteryLevel: Int) {
    Box(
        modifier = Modifier
            .size(100.dp)
            .background(Color.Transparent),
        contentAlignment = Alignment.Center
    ) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val strokeWidth = 12.dp.toPx()
            val radius = size.minDimension / 2 - strokeWidth / 2
            val sweepAngle = batteryLevel / 100f * 360f

            // Draw outer circle
            drawCircle(
                color = Color.Gray,
                radius = radius,
                style = androidx.compose.ui.graphics.drawscope.Stroke(width = strokeWidth)
            )

            // Draw battery percentage arc
            drawArc(
                color = Color.Green,
                startAngle = -90f,
                sweepAngle = sweepAngle,
                useCenter = false,
                style = androidx.compose.ui.graphics.drawscope.Stroke(width = strokeWidth)
            )
        }

        // Display battery percentage text
        Text(
            text = "$batteryLevel%",
            fontWeight = FontWeight.Bold,
            fontSize = 14.sp,
            color = Color.White,
            modifier = Modifier.align(Alignment.Center)
        )
    }
}

