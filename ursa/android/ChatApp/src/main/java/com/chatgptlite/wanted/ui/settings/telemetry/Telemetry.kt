package com.chatgptlite.wanted.ui.settings.rover

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.chatgptlite.wanted.ui.common.AppBar

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun TelemetryScreen(
    viewModel: TelemetryViewModel = viewModel(),
    onBackPressed: () -> Unit
) {
    var ipAddress by remember { mutableStateOf("10.0.0.1") }
    var port by remember {
        mutableStateOf("8000")
    }
    var textToSend by remember { mutableStateOf("ros2 topic list") }
    val xCoordinate by viewModel.xCoordinate.collectAsState() // Collecting the state
    val yCoordinate by viewModel.yCoordinate.collectAsState()
    val zCoordinate by viewModel.zCoordinate.collectAsState()
    val velocity by viewModel.velocity.collectAsState()
    val heading by viewModel.heading.collectAsState()
    val battery by viewModel.battery.collectAsState()
    val pingResult by viewModel.pingResult.collectAsState()
    val messageResult by viewModel.messageResult.collectAsState()

    LaunchedEffect(Unit) {
        val config = viewModel.loadConfig()
        config?.let {
            ipAddress = it.ipAddress
            port = it.port
            textToSend = it.textToSend
        }
    }

    Scaffold(
        topBar = {
            AppBar(
                title = "Telemetry",
                onBackPressed = onBackPressed
            )
        }
    ) { innerPadding ->
        // Make the content scrollable
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp)
                .verticalScroll(rememberScrollState()) // Enable vertical scrolling
        ) {
            Text(
                text = "Terminal",
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 16.dp)
            )
            // IP Address and Port row
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                OutlinedTextField(
                    value = ipAddress,
                    onValueChange = { ipAddress = it },
                    label = { Text("IP Address") },
                    modifier = Modifier.weight(2f)
                )
                Spacer(modifier = Modifier.width(8.dp))
                OutlinedTextField(
                    value = port,
                    onValueChange = { port = it },
                    label = { Text("Port") },
                    modifier = Modifier.weight(1f)
                )
            }
            Spacer(modifier = Modifier.height(16.dp))
            OutlinedTextField(
                value = textToSend,
                onValueChange = { textToSend = it },
                label = { Text("Text to send") },
                modifier = Modifier.fillMaxWidth()
            )
            Spacer(modifier = Modifier.height(16.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.sendPing("$ipAddress:$port") },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Send Ping")
                }
                Spacer(modifier = Modifier.width(16.dp))
                Button(
                    onClick = { viewModel.sendMessage(textToSend = textToSend, "$ipAddress:$port") },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Send Message")
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.saveConfig(ipAddress, port, textToSend) },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Save")
                }
            }

            pingResult?.let { Text("Ping result: $it") }
            messageResult?.let { Text("Message result: $it") }

            // Telemtries Section
            Spacer(modifier = Modifier.height(32.dp))
            Text(
                text = "Telemetry",
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 16.dp)
            )

            // Coordinates Display

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(text = "Coordinates    X: $xCoordinate")
                Spacer(modifier = Modifier.height(6.dp))
                Text(text = "Y: $yCoordinate")
                Spacer(modifier = Modifier.height(6.dp))
                Text(text = "Z: $zCoordinate")
            }

            Spacer(modifier = Modifier.height(8.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.startCoordinatesWebSocket() }, // Trigger retrieval of XYZ coordinates
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Get Coordinates")
                }
            }
            Spacer(modifier = Modifier.height(16.dp))
            // Velocity Display

            Text(text = "Velocity: $velocity")
            Spacer(modifier = Modifier.height(8.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.startVelocityWebSocket() },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Start Velocity Updates")
                }
                Spacer(modifier = Modifier.width(16.dp))
                Button(
                    onClick = { viewModel.stopWebSocket("/cmd_vel") },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Stop Velocity Updates")
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // Heading Display

            Text(text = "Heading: $heading")
            Spacer(modifier = Modifier.height(8.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.startHeadingWebSocket() }, // Trigger retrieval of Heading
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Get Heading")
                }
            }

            Spacer(modifier = Modifier.height(16.dp))
            Text(text = "Battery: $battery")
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = { viewModel.startBatteryWebSocket() }, // Trigger retrieval of Heading
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Get Battery")
                }
            }
        }
    }
}


//@OptIn(ExperimentalMaterial3Api::class)
//@Composable
//fun SettingsScreen(
//    viewModel: RoverSettingsViewModel = viewModel(),
//    onBackPressed: () -> Unit
//) {
//    var ipAddress by remember { mutableStateOf("10.0.0.9") }
//    var port by remember {
//        mutableStateOf("8000")
//    }
//    var textToSend by remember { mutableStateOf("ros2 topic list") }
//
//
//
//    val pingResult by viewModel.pingResult.collectAsState()
//    val messageResult by viewModel.messageResult.collectAsState()
//
//    LaunchedEffect(Unit) {
//        val config = viewModel.loadConfig()
//        config?.let {
//            ipAddress = it.ipAddress
//            port = it.port
//            textToSend = it.textToSend
//        }
//    }
//
//    Scaffold(
//        topBar = {
//            AppBar(
//                title = "Rover Testing",
//                onBackPressed = onBackPressed
//            )
//        }
//    ) { innerPadding ->
//        Column(
//            modifier = Modifier
//                .fillMaxSize()
//                .padding(innerPadding)
//                .padding(16.dp)
//        ) {
//            Text(
//                text = "Terminal",
////                style = MaterialTheme.typography.h5, // You can adjust this based on your theme
//                modifier = Modifier.fillMaxWidth()
//                    .padding(bottom = 16.dp) // Add some space below the title
//            )
//            // IP Address and Port row
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                OutlinedTextField(
//                    value = ipAddress,
//                    onValueChange = { ipAddress = it },
//                    label = { Text("IP Address") },
//                    modifier = Modifier.weight(2f)
//                )
//                Spacer(modifier = Modifier.width(8.dp))
//                OutlinedTextField(
//                    value = port,
//                    onValueChange = { port = it },
//                    label = { Text("Port") },
//                    modifier = Modifier.weight(1f)
//                )
//            }
//            Spacer(modifier = Modifier.height(16.dp))
//            OutlinedTextField(
//                value = textToSend,
//                onValueChange = { textToSend = it },
//                label = { Text("Text to send") },
//                modifier = Modifier.fillMaxWidth()
//            )
//            Spacer(modifier = Modifier.height(16.dp))
//
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                Button(
//
//                    onClick = { viewModel.sendPing("$ipAddress:$port") },
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Send Ping")
//                }
//                Spacer(modifier = Modifier.width(16.dp))
//                Button(
//                    onClick = { viewModel.sendMessage(textToSend = textToSend, "$ipAddress:$port") },
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Send Message")
//                }
//            }
//
//            Spacer(modifier = Modifier.height(16.dp))
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                Button(
//                    onClick = { viewModel.saveConfig(ipAddress, port, textToSend) },
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Save")
//                }
//            }
//
//
//            pingResult?.let { Text("Ping result: $it") }
//            messageResult?.let { Text("Message result: $it") }
//
//            // Telemtries Section
//            Spacer(modifier = Modifier.height(32.dp))
//            Text(
//                text = "Telemetry",
//                modifier = Modifier
//                    .fillMaxWidth()
//                    .padding(bottom = 16.dp)
//            )
//
//            // Coordinates Display
//            val xCoordinate = viewModel.xCoordinate
//            val yCoordinate = viewModel.yCoordinate
//            val zCoordinate = viewModel.zCoordinate
//
//            Text(text = "Coordinates: X: $xCoordinate, Y: $yCoordinate, Z: $zCoordinate")
//            Spacer(modifier = Modifier.height(8.dp))
//
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                Button(
//                    onClick = { viewModel.getCoordinates() }, // Trigger retrieval of XYZ coordinates
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Get Coordinates")
//                }
//            }
//
//            // Velocity Display
//            val velocity = viewModel.velocity
//            Text(text = "Velocity: $velocity")
//            Spacer(modifier = Modifier.height(8.dp))
//
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                Button(
//                    onClick = { viewModel.getVelocity() }, // Trigger retrieval of Velocity
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Get Velocity")
//                }
//            }
//
//            Spacer(modifier = Modifier.height(8.dp))
//
//            // Heading Display
//            val heading = viewModel.heading
//            Text(text = "Heading: $heading")
//            Spacer(modifier = Modifier.height(8.dp))
//
//            Row(
//                modifier = Modifier.fillMaxWidth(),
//                horizontalArrangement = Arrangement.SpaceBetween
//            ) {
//                Button(
//                    onClick = { viewModel.getHeading() }, // Trigger retrieval of Heading
//                    modifier = Modifier.weight(1f)
//                ) {
//                    Text("Get Heading")
//                }
//            }
//
//
//            Spacer(modifier = Modifier.height(16.dp))
//
//
//        }
//
//    }
//}


