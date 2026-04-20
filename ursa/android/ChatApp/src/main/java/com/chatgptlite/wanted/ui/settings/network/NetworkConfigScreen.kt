package com.chatgptlite.wanted.ui.settings.network

import android.content.Context
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun NetworkConfigScreen(
    onBackPressed: () -> Unit
) {
    val context = LocalContext.current
    val prefs = context.getSharedPreferences("network_config", Context.MODE_PRIVATE)

    var roverIp by remember { mutableStateOf(prefs.getString("rover_ip", "10.0.0.1") ?: "10.0.0.1") }
    var wsPort by remember { mutableStateOf(prefs.getString("ws_port", "9090") ?: "9090") }
    var httpPort by remember { mutableStateOf(prefs.getString("http_port", "8080") ?: "8080") }
    var cameraRoute by remember {
        mutableStateOf(
            prefs.getString("camera_route", "/stream?topic=/camera/image_raw&type=ros_compressed")
                ?: "/stream?topic=/camera/image_raw&type=ros_compressed"
        )
    }

    Scaffold(
        containerColor = MaterialTheme.colorScheme.background
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 16.dp, vertical = 12.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            ConfigField("Rover IP Address", roverIp) { roverIp = it }
            ConfigField("WebSocket Port", wsPort) { wsPort = it }
            ConfigField("HTTP Port", httpPort) { httpPort = it }
            ConfigField("Camera Route", cameraRoute) { cameraRoute = it }

            Spacer(modifier = Modifier.height(4.dp))

            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(
                        color = MaterialTheme.colorScheme.primary,
                        shape = RoundedCornerShape(8.dp)
                    )
                    .clickable {
                        prefs.edit()
                            .putString("rover_ip", roverIp)
                            .putString("ws_port", wsPort)
                            .putString("http_port", httpPort)
                            .putString("camera_route", cameraRoute)
                            .apply()
                    }
                    .padding(vertical = 12.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "Save Config",
                    color = Color.Black,
                    fontWeight = FontWeight.Bold,
                    fontSize = 14.sp
                )
            }
        }
    }
}

@Composable
private fun ConfigField(label: String, value: String, onValueChange: (String) -> Unit) {
    Column {
        Text(
            text = label,
            color = Color(0xFF888888),
            fontSize = 12.sp,
            modifier = Modifier.padding(bottom = 4.dp)
        )
        OutlinedTextField(
            value = value,
            onValueChange = onValueChange,
            modifier = Modifier.fillMaxWidth(),
            textStyle = LocalTextStyle.current.copy(
                color = MaterialTheme.colorScheme.primary,
                fontSize = 14.sp,
                fontFamily = FontFamily.Monospace
            ),
            colors = OutlinedTextFieldDefaults.colors(
                unfocusedBorderColor = Color(0xFF444444),
                focusedBorderColor = MaterialTheme.colorScheme.primary,
                cursorColor = MaterialTheme.colorScheme.primary,
                unfocusedContainerColor = Color(0xFF2A2E2E),
                focusedContainerColor = Color(0xFF2A2E2E)
            ),
            shape = RoundedCornerShape(8.dp),
            singleLine = true
        )
    }
}
