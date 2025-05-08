package com.chatgptlite.wanted.ui.settings.occupancy

import android.graphics.Bitmap
import android.widget.Toast
import androidx.compose.foundation.layout.*
import androidx.compose.material3.Button
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.Image
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.ui.Alignment
import androidx.compose.ui.graphics.asImageBitmap
import androidx.lifecycle.viewmodel.compose.viewModel
import com.chatgptlite.wanted.ui.common.AppBar

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun Occupancy(
    viewModel: OccupancyViewModel = viewModel(),
    onBackPressed: () -> Unit
) {
    val occupancyBitmap = viewModel.occupancyBitmap.value
    // Automatically start WebSocket when this page is opened
    LaunchedEffect(Unit) {
        viewModel.startOccupancyWebSocket()
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Box(
                        modifier = Modifier.fillMaxWidth(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = "Occupancy Map",
                            style = MaterialTheme.typography.titleMedium // Adjust text style if needed
                        )
                    }
                },
                modifier = Modifier.height(48.dp), // Reduce height of the TopAppBar
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    titleContentColor = MaterialTheme.colorScheme.onPrimary
                )
            )
        }

//        topBar = {
//            AppBar(
//                title = "Occupancy Map",
//                onBackPressed = onBackPressed
//            )
//        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp)
        ) {

            // Render the occupancy map as a canvas-based Bitmap
            if (occupancyBitmap != null) {
                Image(
                    bitmap = occupancyBitmap.asImageBitmap(),
                    contentDescription = "Occupancy Map",
                    modifier = Modifier
                        .fillMaxWidth()
                        .aspectRatio(1f) // Adjust aspect ratio as needed
                )
            } else {
                Text(
                    text = "Loading occupancy map...",
                )
            }

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = {
                        // viewModel.testOccupancyBitmap() // This is for testing

                        // Uncomment below for real time data
                        viewModel.startOccupancyWebSocket()
                    },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("[Test Only] Get Occupancy Map")
                }
            }


        }
    }
}