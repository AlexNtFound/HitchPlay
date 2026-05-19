// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.util.Log
import androidx.core.app.NotificationCompat
import com.quicinc.chatapp.R
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Foreground service that hosts a [ModelProvisioner] and keeps the model bin
 * download running when the app is backgrounded or the launching activity is killed.
 *
 * The service exposes its state via a process-level [StateFlow] ([Companion.state])
 * so the UI can observe progress without binding to the service explicitly.
 *
 * Idempotent: calling [start] while a download is in progress is a no-op (the existing
 * service keeps running). [cancel] stops the active download and the service.
 */
class ModelDownloadService : Service() {

    private val tag = "ModelDownloadService"
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    private var provisioner: ModelProvisioner? = null
    private var observerJob: Job? = null
    private var driverJob: Job? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        ensureNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val action = intent?.action
        if (action == ACTION_CANCEL) {
            Log.i(tag, "received cancel intent")
            provisioner?.cancel()
            scope.cancel()
            stopForegroundCompat()
            stopSelf()
            return START_NOT_STICKY
        }

        val modelName = intent?.getStringExtra(EXTRA_MODEL_NAME)
            ?: run {
                Log.e(tag, "started without EXTRA_MODEL_NAME; stopping")
                stopSelf()
                return START_NOT_STICKY
            }

        // Already running? Don't restart it.
        if (provisioner != null) {
            Log.i(tag, "already running for ${provisioner?.manifestModelName}, ignoring")
            return START_STICKY
        }

        startForegroundWithInitialNotification()
        beginProvisioning(modelName)
        return START_STICKY
    }

    private fun beginProvisioning(modelName: String) {
        val manifest = ModelManifest.load(this, modelName)
        val p = ModelProvisioner(this, manifest).also { provisioner = it }

        observerJob = scope.launch {
            p.state.collect { s ->
                _state.value = s
                updateNotification(s)
                if (s is ProvisionState.Done || s is ProvisionState.Failed) {
                    Log.i(tag, "terminal state $s; stopping service")
                    stopForegroundCompat()
                    stopSelf()
                }
            }
        }
        driverJob = scope.launch { p.ensureProvisioned() }
    }

    override fun onDestroy() {
        super.onDestroy()
        provisioner?.cancel()
        observerJob?.cancel()
        driverJob?.cancel()
        scope.cancel()
        provisioner = null
    }

    // ----- foreground service plumbing -----

    private fun startForegroundWithInitialNotification() {
        val n = buildNotification(ProvisionState.Idle)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            startForeground(NOTIFICATION_ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC)
        } else {
            startForeground(NOTIFICATION_ID, n)
        }
    }

    private fun stopForegroundCompat() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            stopForeground(STOP_FOREGROUND_REMOVE)
        } else {
            @Suppress("DEPRECATION")
            stopForeground(true)
        }
    }

    private fun ensureNotificationChannel() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) return
        val nm = getSystemService(NotificationManager::class.java) ?: return
        if (nm.getNotificationChannel(CHANNEL_ID) != null) return
        val ch = NotificationChannel(
            CHANNEL_ID,
            getString(R.string.provisioning_notif_channel_name),
            NotificationManager.IMPORTANCE_LOW
        ).apply {
            description = getString(R.string.provisioning_notif_channel_desc)
            setShowBadge(false)
            setSound(null, null)
            enableVibration(false)
        }
        nm.createNotificationChannel(ch)
    }

    private fun updateNotification(state: ProvisionState) {
        val nm = getSystemService(NotificationManager::class.java) ?: return
        nm.notify(NOTIFICATION_ID, buildNotification(state))
    }

    private fun buildNotification(state: ProvisionState): Notification {
        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.stat_sys_download)
            .setOnlyAlertOnce(true)
            .setOngoing(state !is ProvisionState.Done && state !is ProvisionState.Failed)
            .setContentIntent(makeOpenAppIntent())
            .addAction(
                0,
                getString(R.string.provisioning_cancel),
                makeCancelIntent()
            )

        when (state) {
            is ProvisionState.Idle -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_setup_title))
                    .setContentText(getString(R.string.provisioning_starting))
                    .setProgress(0, 0, true)
            }
            is ProvisionState.Verifying -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_setup_title))
                    .setContentText(
                        getString(
                            R.string.provisioning_verifying,
                            state.fileIndex + 1,
                            state.totalFiles
                        )
                    )
                    .setProgress(0, 0, true)
            }
            is ProvisionState.Downloading -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_downloading_title))
                    .setContentText(
                        getString(
                            R.string.provisioning_notif_downloading_text,
                            state.fileIndex + 1,
                            state.totalFiles,
                            state.overallPercent
                        )
                    )
                    .setProgress(100, state.overallPercent, false)
            }
            is ProvisionState.WaitingForWifi -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_setup_title))
                    .setContentText(getString(R.string.provisioning_waiting_wifi))
                    .setProgress(0, 0, true)
            }
            is ProvisionState.Done -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_done_title))
                    .setContentText(getString(R.string.provisioning_done))
                    .setProgress(0, 0, false)
                    .setOngoing(false)
            }
            is ProvisionState.Failed -> {
                builder.setContentTitle(getString(R.string.provisioning_notif_failed_title))
                    .setContentText(state.message)
                    .setProgress(0, 0, false)
                    .setOngoing(false)
            }
        }
        return builder.build()
    }

    private fun makeOpenAppIntent(): PendingIntent {
        // Re-launch the app's launcher activity (the chat MainActivity).
        val launch = packageManager.getLaunchIntentForPackage(packageName)
            ?: Intent()
        return PendingIntent.getActivity(
            this, 0, launch,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
    }

    private fun makeCancelIntent(): PendingIntent {
        val i = Intent(this, ModelDownloadService::class.java).apply { action = ACTION_CANCEL }
        return PendingIntent.getService(
            this, 1, i,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
    }

    // Convenience: expose the active manifest's model name for log messages.
    private val ModelProvisioner.manifestModelName: String
        get() = modelDir.name

    companion object {
        private const val CHANNEL_ID = "ursa.model_provisioning"
        private const val NOTIFICATION_ID = 4201

        const val EXTRA_MODEL_NAME = "ursa.modelName"
        private const val ACTION_CANCEL = "com.quicinc.chatapp.provisioning.ACTION_CANCEL"

        // Process-level state. Survives activity destruction; observed by ViewModel/UI.
        // Resets to Idle only when a new download is started.
        private val _state = MutableStateFlow<ProvisionState>(ProvisionState.Idle)
        val state: StateFlow<ProvisionState> = _state.asStateFlow()

        /** Starts the foreground download (idempotent). */
        fun start(context: Context, modelName: String) {
            val i = Intent(context, ModelDownloadService::class.java).apply {
                putExtra(EXTRA_MODEL_NAME, modelName)
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                context.startForegroundService(i)
            } else {
                context.startService(i)
            }
        }

        /** Cancels the active download and stops the service. */
        fun cancel(context: Context) {
            val i = Intent(context, ModelDownloadService::class.java).apply {
                action = ACTION_CANCEL
            }
            // Use startService (not startForegroundService) — we're delivering an action,
            // not requesting a fresh foreground start. The service handles the action.
            context.startService(i)
        }

        /** Resets the observable state. Called when the UI wants a fresh retry. */
        fun resetState() {
            _state.value = ProvisionState.Idle
        }
    }
}
