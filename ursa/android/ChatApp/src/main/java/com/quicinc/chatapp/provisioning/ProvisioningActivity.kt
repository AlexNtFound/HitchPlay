// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.app.Activity
import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.chatgptlite.wanted.ui.theme.ChatGPTLiteTheme
import com.quicinc.chatapp.R

/**
 * Pre-chat gate: ensures the on-device LLM bins are present and verified.
 *
 * Caller passes the model name via [EXTRA_MODEL_NAME]. On success the activity
 * sets [Activity.RESULT_OK] and finishes; the caller can resume its normal flow.
 *
 * Designed to be re-entrant — if the user kills the app mid-download, the next
 * launch will re-verify and resume only the missing files.
 */
class ProvisioningActivity : ComponentActivity() {

    private val viewModel: ProvisioningViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val modelName = intent.getStringExtra(EXTRA_MODEL_NAME)
            ?: error("ProvisioningActivity requires EXTRA_MODEL_NAME")

        setContent {
            ChatGPTLiteTheme {
                Surface(modifier = Modifier.fillMaxSize()) {
                    val state by viewModel.state.collectAsState()
                    LaunchedEffect(Unit) { viewModel.start(modelName) }
                    LaunchedEffect(state) {
                        if (state is ProvisionState.Done) {
                            setResult(RESULT_OK)
                            finish()
                        }
                    }
                    ProvisionScreen(
                        state = state,
                        onRetry = { viewModel.retry(modelName) },
                        onCancel = {
                            setResult(RESULT_CANCELED)
                            finish()
                        }
                    )
                }
            }
        }
    }

    companion object {
        const val EXTRA_MODEL_NAME = "ursa.modelName"
    }
}

@Composable
private fun ProvisionScreen(
    state: ProvisionState,
    onRetry: () -> Unit,
    onCancel: () -> Unit
) {
    Box(modifier = Modifier.fillMaxSize().padding(24.dp), contentAlignment = Alignment.Center) {
        Column(
            verticalArrangement = Arrangement.spacedBy(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.fillMaxWidth()
        ) {
            Text(
                text = stringResource(R.string.provisioning_title),
                fontSize = 22.sp,
                style = MaterialTheme.typography.titleLarge
            )
            when (state) {
                is ProvisionState.Idle -> Text(stringResource(R.string.provisioning_starting))

                is ProvisionState.Verifying -> {
                    Text(
                        stringResource(
                            R.string.provisioning_verifying,
                            state.fileIndex + 1,
                            state.totalFiles
                        )
                    )
                    LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                }

                is ProvisionState.Downloading -> {
                    Text(
                        text = stringResource(
                            R.string.provisioning_downloading_file,
                            state.fileIndex + 1,
                            state.totalFiles,
                            state.fileName
                        )
                    )
                    LinearProgressIndicator(
                        progress = { state.overallPercent / 100f },
                        modifier = Modifier.fillMaxWidth()
                    )
                    Text(
                        text = "${state.overallPercent}% — " +
                            "${humanSize(state.overallBytesDone)} / " +
                            humanSize(state.overallBytesTotal),
                        fontSize = 14.sp
                    )
                }

                is ProvisionState.WaitingForWifi -> {
                    Text(stringResource(R.string.provisioning_waiting_wifi))
                    LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                }

                is ProvisionState.Done -> Text(stringResource(R.string.provisioning_done))

                is ProvisionState.Failed -> {
                    Text(
                        text = stringResource(R.string.provisioning_failed, state.message),
                        color = MaterialTheme.colorScheme.error
                    )
                    Button(onClick = onRetry) {
                        Text(stringResource(R.string.provisioning_retry))
                    }
                    Button(onClick = onCancel) {
                        Text(stringResource(R.string.provisioning_cancel))
                    }
                }
            }
        }
    }
}

private fun humanSize(bytes: Long): String {
    if (bytes < 1024) return "$bytes B"
    val units = arrayOf("KB", "MB", "GB", "TB")
    var v = bytes.toDouble() / 1024.0
    var i = 0
    while (v >= 1024.0 && i < units.lastIndex) { v /= 1024.0; i++ }
    return "%.1f %s".format(v, units[i])
}
