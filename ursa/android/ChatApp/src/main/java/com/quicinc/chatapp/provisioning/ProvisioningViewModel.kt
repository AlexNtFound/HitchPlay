// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import kotlinx.coroutines.flow.StateFlow

/**
 * Thin proxy in front of [ModelDownloadService]. The service owns the actual
 * [ModelProvisioner] instance and survives the activity, so leaving the app
 * mid-download no longer stops or restarts it.
 */
class ProvisioningViewModel(app: Application) : AndroidViewModel(app) {

    /** Process-level state from the foreground service. Survives across re-entries. */
    val state: StateFlow<ProvisionState> = ModelDownloadService.state

    /** Starts the download service (idempotent — no-op if already running). */
    fun start(modelName: String) {
        ModelDownloadService.start(getApplication(), modelName)
    }

    /** Cancels any active download, clears state, then re-starts fresh. */
    fun retry(modelName: String) {
        ModelDownloadService.cancel(getApplication())
        ModelDownloadService.resetState()
        ModelDownloadService.start(getApplication(), modelName)
    }
}
