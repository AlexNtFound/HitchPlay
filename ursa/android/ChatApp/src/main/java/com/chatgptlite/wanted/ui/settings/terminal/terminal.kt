package com.chatgptlite.wanted.ui.settings.terminal

import android.util.Log
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.*
import androidx.compose.material3.OutlinedTextFieldDefaults
import androidx.compose.runtime.*
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.FocusRequester
import androidx.compose.ui.focus.focusRequester
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalSoftwareKeyboardController
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.chatgptlite.wanted.ui.common.AppBar

@OptIn(ExperimentalComposeUiApi::class)
@Composable
fun TerminalScreen(
    viewModel: TerminalViewModel = viewModel(),
    onBackPressed: () -> Unit
) {
    var inputText by remember { mutableStateOf("") }
    val messageResult by viewModel.messageResult.collectAsState()
    val terminalHistory = remember {
        mutableStateListOf(
            "Ursa Rover Control Terminal - v1.3.2",
            "Connected to Rover: URSA-01",
            "System Status: ONLINE",
            "Last Sync: 2025-02-04 14:23 UTC",
            "Type 'help' for a list of commands.\n"
        )
    }

    val scrollState = rememberScrollState()
    val keyboardController = LocalSoftwareKeyboardController.current
    val focusRequester = remember { FocusRequester() }

    // Append message result to terminal when it changes
    LaunchedEffect(messageResult) {
        messageResult?.let {
            terminalHistory.add(it)
        }
    }

    Scaffold(
        containerColor = MaterialTheme.colorScheme.background
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 16.dp, vertical = 12.dp)
        ) {
            // Scrollable terminal text area
            Column(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
                    .verticalScroll(scrollState)
            ) {
                terminalHistory.forEach { line ->
                    Text(
                        text = line,
                        color = Color.Green,
                        fontFamily = FontFamily.Monospace,
                        fontSize = 14.sp,
                        modifier = Modifier.padding(bottom = 4.dp)
                    )
                }
            }

            Spacer(modifier = Modifier.height(12.dp))

            // Terminal input field
            OutlinedTextField(
                value = inputText,
                onValueChange = { inputText = it },
                modifier = Modifier
                    .fillMaxWidth()
                    .focusRequester(focusRequester),
                placeholder = { Text("Enter command", color = Color.Gray) },
                textStyle = LocalTextStyle.current.copy(
                    color = Color.Green,
                    fontFamily = FontFamily.Monospace
                ),
                colors = OutlinedTextFieldDefaults.colors(
                    unfocusedBorderColor = Color.Green,
                    focusedBorderColor = Color.Green,
                    cursorColor = Color.Green
                ),
                singleLine = true,
                keyboardOptions = KeyboardOptions(imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(
                    onSend = {
                        if (inputText.isNotBlank()) {
                            terminalHistory.add("> $inputText")
                            viewModel.sendMessage(inputText, null) // null = use default IP
                            inputText = ""
                            keyboardController?.hide()
                        }
                    }
                )
            )
            Spacer(modifier = Modifier.height(40.dp))
        }
    }
}


