// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Drives [ModelProvisioner] from the UI layer.
 *
 * Owns the provisioner instance and re-exposes its state. Calling [start] is idempotent —
 * a second call while a download is in flight is a no-op.
 */
class ProvisioningViewModel(app: Application) : AndroidViewModel(app) {

    private var provisioner: ModelProvisioner? = null
    private var job: Job? = null

    private val _state = MutableStateFlow<ProvisionState>(ProvisionState.Idle)
    val state: StateFlow<ProvisionState> = _state.asStateFlow()

    fun start(modelName: String) {
        if (job?.isActive == true) return
        val app = getApplication<Application>()
        val manifest = ModelManifest.load(app, modelName)
        // Provisioner derives its modelDir from getExternalFilesDir(...) internally —
        // this is the only path DownloadManager reliably writes to on Android 11+.
        val p = ModelProvisioner(app, manifest).also { provisioner = it }
        job = viewModelScope.launch {
            p.state.collect { _state.value = it }
        }
        viewModelScope.launch { p.ensureProvisioned() }
    }

    fun retry(modelName: String) {
        provisioner?.cancel()
        job?.cancel()
        job = null
        provisioner = null
        _state.value = ProvisionState.Idle
        start(modelName)
    }

    override fun onCleared() {
        provisioner?.cancel()
        super.onCleared()
    }
}
