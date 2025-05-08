package com.chatgptlite.wanted.ui.settings.controller

import android.graphics.Bitmap
import android.util.Log
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.ui.viewinterop.AndroidView

import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.requiredWidthIn
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.wrapContentHeight
import androidx.compose.foundation.layout.wrapContentWidth
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.LaunchedEffect

import androidx.compose.ui.Alignment
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.sp
import io.github.controlwear.virtual.joystick.android.JoystickView
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.draw.clip


@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun VideoStreamingSetting(
    viewModel: VideoCamSettingsViewModel,
    onBackPressed: () -> Unit
) {
    var ipAddress by remember { mutableStateOf("10.0.0.1") }
    var port by remember {
        mutableStateOf("8080")
    }
    var route by remember {
        mutableStateOf("/stream?topic=/camera/image_raw&type=ros_compressed")
    }

    var MAX_ANGULAR_VELOCITY = 1.2
    var MAX_VELOCITY = 1
    LaunchedEffect(Unit) {
        val config = viewModel.loadConfig()
        config?.let {
            ipAddress = it.ipAddress
            port = it.port
            route = it.route
        }
        viewModel.receiveFeed(ipAddress, port, route)
        viewModel.createWebSocket()
    }

    Scaffold(
//        topBar = {
//            TopAppBar(
//                title = {
//                    Text(
//                        text = "Controller",
//                        fontWeight = FontWeight.Bold,
//                        modifier = Modifier.fillMaxWidth()
//                            .wrapContentHeight(align = Alignment.CenterVertically),
//                        textAlign = TextAlign.Center,
//                        color = MaterialTheme.colorScheme.primary
//                    )
//                },
//                modifier = Modifier.height(80.dp), // Reduce top bar height
//                colors = TopAppBarDefaults.topAppBarColors(
//                    containerColor = MaterialTheme.colorScheme.background,
//                    titleContentColor = MaterialTheme.colorScheme.onPrimary
//                )
//            )
//        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp)
                //.verticalScroll(rememberScrollState())
        ) {

            Spacer(modifier = Modifier.height(16.dp))

            // Add the video feed display
            VideoFeedDisplay(viewModel.currentFrame.value, viewModel.occupancyBitmap.value)

            Spacer(modifier = Modifier.height(30.dp))
            val primaryColorInt = MaterialTheme.colorScheme.primary.toArgb()

            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(horizontal = 2.dp),
                horizontalArrangement = Arrangement.SpaceEvenly
            ) {
                // Horizontal joystick with arrows
                Box(
                    contentAlignment = Alignment.Center,
                    modifier = Modifier
                        .weight(1f) // 👈 take half width
                        .aspectRatio(1f) // 👈 keep it square (height = width)
                        .requiredWidthIn(min = 200.dp)
//                        .padding(4.dp) // optional: slight inner padding
                ) {
                    AndroidView(
                        factory = { context ->
                            JoystickView(context).apply {
                                setButtonColor(primaryColorInt)
                                setBorderColor(primaryColorInt)
                                setBackgroundColor(android.graphics.Color.BLACK)
                                setButtonDirection(-1) // Horizontal only
                                setFixedCenter(true)
                                setButtonSizeRatio(0.2f)

                                setOnMoveListener { angle, strength ->
                                    Log.d("Joystick", "Angle: $angle, Strength: $strength")
                                    // int angle, int strength
                                    // max strength = 100
                                    // onHorizontalMove(angle, strength)
                                    val angularVelocity = MAX_ANGULAR_VELOCITY * strength / 100.0
                                    if (angle == 180) {
                                        viewModel.controlRover(0.0, angularVelocity)
                                    } else {
                                        viewModel.controlRover(0.0,  -angularVelocity)
                                    }


                                }
                            }
                        },
                        modifier = Modifier.fillMaxSize()
                    )

                    // Left and right arrows
                    Row(
                        modifier = Modifier
                            .wrapContentWidth()
                            .padding(horizontal = 10.dp), // only small distance from center
                        horizontalArrangement = Arrangement.spacedBy(70.dp) // tighter spacing between arrows
                    ) {
                        Text(
                            text = "◀",
                            color = MaterialTheme.colorScheme.primary,
                            fontSize = 24.sp,
                            modifier = Modifier.align(Alignment.CenterVertically)
                        )
                        Text(
                            text = "▶",
                            color = MaterialTheme.colorScheme.primary,
                            fontSize = 24.sp,
                            modifier = Modifier.align(Alignment.CenterVertically)
                        )
                    }

                }

//               Spacer(modifier = Modifier.width(5.dp))

                // Vertical joystick with arrows
                Box(
                    contentAlignment = Alignment.Center,
                    modifier = Modifier
                        .weight(1f) // 👈 also half width
                        .aspectRatio(1f) // 👈 also square
                        .requiredWidthIn(min = 200.dp)
//                        .padding(4.dp)
                ) {
                    AndroidView(
                        factory = { context ->
                            JoystickView(context).apply {
                                setButtonColor(primaryColorInt)
                                setBorderColor(primaryColorInt)
                                setBackgroundColor(android.graphics.Color.BLACK)
                                setButtonDirection(1) // Vertical only
                                setFixedCenter(true)
                                setButtonSizeRatio(0.2f)

                                setOnMoveListener { angle, strength ->
                                    Log.d("Joystick", "Angle: $angle, Strength: $strength")

                                    // Convert strength (0–100) to velocity scale (e.g., max 1.0)
                                    val velocity = MAX_VELOCITY * strength / 100.0

                                    if (angle == 90) {
                                        // Move forward
                                        viewModel.controlRover(velocity, 0.0)
                                    } else if (angle == 270) {
                                        // Move backward
                                        viewModel.controlRover(-velocity, 0.0)
                                    }
                                }

                            }
                        },
                        modifier = Modifier.fillMaxSize()
                    )

                    // Top and bottom arrows
                    Column(
                        modifier = Modifier
                            .wrapContentHeight()
                            .padding(vertical = 10.dp), // small distance from center
                        verticalArrangement = Arrangement.spacedBy(70.dp) // tighter spacing between arrows
                    ) {
                        Text(
                            text = "▲",
                            color = MaterialTheme.colorScheme.primary,
                            fontSize = 24.sp,
                            textAlign = TextAlign.Center,
                            modifier = Modifier.fillMaxWidth()
                        )
                        Text(
                            text = "▼",
                            color = MaterialTheme.colorScheme.primary,
                            fontSize = 24.sp,
                            textAlign = TextAlign.Center,
                            modifier = Modifier.fillMaxWidth()
                        )
                    }

                }
            }

            Spacer(modifier = Modifier.width(16.dp))
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.Center
            ) {
                GradientButton(
                    text = "Return to Base",
                    onClick = { viewModel.returnToBase() }
                )
            }


        }
    }
}

fun onHorizontalMove(angle: Int, strength: Int) {
    TODO("Not yet implemented")
}

fun onVerticalMove(angle: Int, strength: Int) {
    TODO("Not yet implemented")
}

@Composable
fun VideoFeedDisplay(videoBitmap: Bitmap?, occupancyBitmap: Bitmap?) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(300.dp)
            .clip(RoundedCornerShape(8.dp)) // <-- matching 8dp corner radius
            .background(Color.Black) // Optional: default background if bitmap is transparent
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

        if (occupancyBitmap != null) {
            Image(
                bitmap = occupancyBitmap.asImageBitmap(),
                contentDescription = "Occupancy Map",
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .size(100.dp)
                    .padding(8.dp)
                    .graphicsLayer(alpha = 0.7f)
                    .clip(RoundedCornerShape(8.dp))
            )
        } else {
            Box(
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .size(100.dp)
                    .padding(8.dp)
                    .background(Color.Red.copy(alpha = 0.7f))
                    .clip(RoundedCornerShape(8.dp)),
                contentAlignment = Alignment.Center
            ) {
                Text("Occupancy Map", color = Color.White, fontSize = 10.sp)
            }
        }
    }
}

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
            .padding(horizontal = 24.dp, vertical = 12.dp), // <-- extra horizontal padding to make it a bit wider
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            color = MaterialTheme.colorScheme.primary,
            style = MaterialTheme.typography.bodyLarge
        )
    }
}
