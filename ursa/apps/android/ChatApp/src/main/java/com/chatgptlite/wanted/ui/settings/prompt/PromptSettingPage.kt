package com.chatgptlite.wanted.ui.settings.prompt

import android.widget.Toast
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button

import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.material3.OutlinedTextField
import androidx.compose.ui.platform.LocalContext
import com.chatgptlite.wanted.ui.common.AppBar

@Composable
fun PromptSettingPage(
    onBackPressed: () -> Unit
) {
    var prompt by remember {
        mutableStateOf("test")
    }

    val context = LocalContext.current

    Scaffold(
        topBar = {
            AppBar(
                title = "Prompt Settings",
                onBackPressed = onBackPressed
            )
        }
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(16.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = {
                        if (prompt.contains("{content}")) {
//                            viewModel.chatState.extraPrompt = prompt
                            Toast.makeText(context, "save successfully", Toast.LENGTH_SHORT).show()
                        }
                        else {
                            Toast.makeText(context, "fail to save. miss {content}", Toast.LENGTH_SHORT).show()
                        }
                    },
                    modifier = Modifier.weight(1f)
                ) {
                    Text("save")
                }
            }

            OutlinedTextField(
                value = prompt,
                onValueChange = {
                    prompt = it
                },
                label = { Text("prompt") },
                modifier = Modifier.fillMaxWidth().fillMaxHeight()
            )
        }
    }
}