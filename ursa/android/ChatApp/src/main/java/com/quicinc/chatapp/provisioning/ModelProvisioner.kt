// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.content.Context
import android.util.Log
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Call
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.IOException
import java.security.MessageDigest
import java.util.concurrent.TimeUnit

/**
 * Provisions a model's `.bin` files into a local directory:
 *   1. Verifies each file's size + SHA-256 against the [ModelManifest].
 *   2. Downloads any missing/corrupt file via OkHttp (streamed to a `.part` file).
 *   3. Re-verifies and atomically renames each file into place.
 *
 * We deliberately bypass [android.app.DownloadManager]: on several OEM ROMs (notably ASUS
 * ROG, some Xiaomi, some Vivo) DM silently leaves requests in `STATUS_PENDING` forever,
 * with no error visible to the app. OkHttp gives us deterministic behavior on every device.
 *
 * Caller observes [state] for UI updates. Idempotent: calling [ensureProvisioned] when
 * everything is already valid is cheap (size + hash check only).
 */
class ModelProvisioner(
    private val context: Context,
    private val manifest: ModelManifest,
    private val ioDispatcher: CoroutineDispatcher = Dispatchers.IO
) {
    private val tag = "ModelProvisioner"

    // Stored under /storage/emulated/0/Android/data/<pkg>/files/models/<modelName>/.
    // App-private external storage; survives uninstall? No — wiped on uninstall.
    // Survives data clears? No. Survives reboots and app updates? Yes.
    private val externalFilesRoot: File = context.getExternalFilesDir(null)
        ?: error("No external files dir available — device may have no external storage")

    /** Absolute on-device dir holding all artifacts for this model (bins + tokenizer + config). */
    val modelDir: File = File(externalFilesRoot, "models/${manifest.modelName}")

    private val _state = MutableStateFlow<ProvisionState>(ProvisionState.Idle)
    val state: StateFlow<ProvisionState> = _state.asStateFlow()

    /** Held so [cancel] can interrupt an in-flight HTTP call. */
    @Volatile private var activeCall: Call? = null

    private val httpClient: OkHttpClient = OkHttpClient.Builder()
        // Generous timeouts: a slow but progressing download must not be killed.
        // The read timeout applies between successive reads, not to the overall transfer.
        .connectTimeout(30, TimeUnit.SECONDS)
        .readTimeout(60, TimeUnit.SECONDS)
        .writeTimeout(60, TimeUnit.SECONDS)
        .callTimeout(0, TimeUnit.MILLISECONDS) // no overall cap — files are GBs
        .followRedirects(true) // GitHub Releases 302s to a CDN; required.
        .followSslRedirects(true)
        .build()

    /**
     * Verifies, then downloads, then re-verifies until every file in [manifest] is valid
     * inside [modelDir]. Suspends until done, error, or cancellation.
     */
    suspend fun ensureProvisioned() = withContext(ioDispatcher) {
        Log.i(tag, "ensureProvisioned: modelDir=$modelDir, files=${manifest.files.size}")
        if (!modelDir.exists() && !modelDir.mkdirs()) {
            _state.value = ProvisionState.Failed("Cannot create model directory: $modelDir")
            return@withContext
        }
        try {
            _state.value = ProvisionState.Verifying(0, manifest.files.size)
            val needed = mutableListOf<ModelManifest.ModelFile>()
            manifest.files.forEachIndexed { idx, f ->
                _state.value = ProvisionState.Verifying(idx, manifest.files.size)
                if (!isFileValid(File(modelDir, f.name), f)) needed += f
            }
            Log.i(tag, "verify done: ${needed.size} of ${manifest.files.size} files need download")
            if (needed.isEmpty()) {
                _state.value = ProvisionState.Done
                return@withContext
            }

            val totalBytes = needed.sumOf { it.sizeBytes }
            var completedBytes = 0L
            needed.forEachIndexed { i, file ->
                _state.value = ProvisionState.Downloading(
                    fileIndex = i,
                    totalFiles = needed.size,
                    fileName = file.name,
                    fileBytesDone = 0,
                    fileBytesTotal = file.sizeBytes,
                    overallBytesDone = completedBytes,
                    overallBytesTotal = totalBytes
                )
                downloadOne(file, needed.size, i, completedBytes, totalBytes)
                _state.value = ProvisionState.Verifying(i, needed.size)
                if (!isFileValid(File(modelDir, file.name), file)) {
                    _state.value = ProvisionState.Failed(
                        "Hash mismatch after download: ${file.name}"
                    )
                    return@withContext
                }
                completedBytes += file.sizeBytes
            }
            _state.value = ProvisionState.Done
        } catch (t: Throwable) {
            Log.e(tag, "Provisioning failed", t)
            _state.value = ProvisionState.Failed(t.message ?: "Unknown error")
        }
    }

    /** Cancels any active HTTP call. Safe to call from any thread. */
    fun cancel() {
        activeCall?.cancel()
        activeCall = null
    }

    // ---------- internals ----------

    private fun downloadOne(
        file: ModelManifest.ModelFile,
        totalFiles: Int,
        fileIndex: Int,
        prevCompleted: Long,
        totalBytes: Long
    ) {
        val finalFile = File(modelDir, file.name)
        if (finalFile.exists()) finalFile.delete()
        val tempFile = File(modelDir, "${file.name}.part")
        if (tempFile.exists()) tempFile.delete()

        Log.i(tag, "GET ${file.url} -> $tempFile (${humanSize(file.sizeBytes)})")
        val request = Request.Builder().url(file.url).build()
        val call = httpClient.newCall(request).also { activeCall = it }

        try {
            call.execute().use { response ->
                if (!response.isSuccessful) {
                    throw IOException("HTTP ${response.code} ${response.message} for ${file.name}")
                }
                val body = response.body
                    ?: throw IOException("Empty response body for ${file.name}")

                val source = body.byteStream()
                FileOutputStream(tempFile).use { sink ->
                    val buf = ByteArray(64 * 1024) // 64 KB — sweet spot for throughput/CPU
                    var bytesWritten = 0L
                    var lastReportMs = 0L
                    var lastLoggedPct = -1
                    while (true) {
                        val n = source.read(buf)
                        if (n < 0) break
                        sink.write(buf, 0, n)
                        bytesWritten += n

                        val now = System.currentTimeMillis()
                        // Throttle UI updates to ~4 Hz to keep the StateFlow cheap.
                        if (now - lastReportMs > 250) {
                            _state.value = ProvisionState.Downloading(
                                fileIndex = fileIndex,
                                totalFiles = totalFiles,
                                fileName = file.name,
                                fileBytesDone = bytesWritten,
                                fileBytesTotal = file.sizeBytes,
                                overallBytesDone = prevCompleted + bytesWritten,
                                overallBytesTotal = totalBytes
                            )
                            lastReportMs = now
                        }

                        // Sparse progress logs: every ~10 % so logcat stays useful.
                        val pct = if (file.sizeBytes > 0)
                            ((bytesWritten * 10) / file.sizeBytes).toInt() else 0
                        if (pct != lastLoggedPct) {
                            Log.i(tag, "${file.name}: ${pct * 10}% (${humanSize(bytesWritten)}/${humanSize(file.sizeBytes)})")
                            lastLoggedPct = pct
                        }
                    }
                    sink.flush()
                }
            }
            Log.i(tag, "download complete: ${file.name}, renaming part -> final")
            if (!tempFile.renameTo(finalFile)) {
                throw IOException("Failed to rename ${tempFile.name} -> ${file.name}")
            }
        } finally {
            activeCall = null
            // If anything blew up mid-download, leave no partial artifact.
            if (tempFile.exists() && !finalFile.exists()) tempFile.delete()
        }
    }

    /** Returns true iff [file] exists, has [expected.sizeBytes], and matches sha256. */
    private fun isFileValid(file: File, expected: ModelManifest.ModelFile): Boolean {
        if (!file.exists() || file.length() != expected.sizeBytes) return false
        val actual = sha256Of(file)
        val ok = actual.equals(expected.sha256, ignoreCase = true)
        if (!ok) Log.w(tag, "${file.name}: sha256 expected=${expected.sha256} actual=$actual")
        return ok
    }

    private fun sha256Of(file: File): String {
        val md = MessageDigest.getInstance("SHA-256")
        FileInputStream(file).use { input ->
            val buf = ByteArray(1 shl 20) // 1 MB
            while (true) {
                val n = input.read(buf)
                if (n <= 0) break
                md.update(buf, 0, n)
            }
        }
        return md.digest().joinToString(separator = "") { "%02x".format(it) }
    }

    private fun humanSize(bytes: Long): String {
        if (bytes < 1024) return "$bytes B"
        val units = arrayOf("KB", "MB", "GB", "TB")
        var v = bytes.toDouble() / 1024.0
        var i = 0
        while (v >= 1024.0 && i < units.lastIndex) { v /= 1024.0; i++ }
        return "%.1f %s".format(v, units[i])
    }
}

/** UI-facing provisioning state. Single source of truth for the activity/viewmodel. */
sealed interface ProvisionState {
    object Idle : ProvisionState
    data class Verifying(val fileIndex: Int, val totalFiles: Int) : ProvisionState
    data class Downloading(
        val fileIndex: Int,
        val totalFiles: Int,
        val fileName: String,
        val fileBytesDone: Long,
        val fileBytesTotal: Long,
        val overallBytesDone: Long,
        val overallBytesTotal: Long
    ) : ProvisionState {
        val overallPercent: Int
            get() = if (overallBytesTotal <= 0) 0
                    else ((overallBytesDone * 100) / overallBytesTotal).toInt().coerceIn(0, 100)
    }
    /** Retained for API compatibility; OkHttp downloader does not currently emit this. */
    data class WaitingForWifi(val fileName: String) : ProvisionState
    object Done : ProvisionState
    data class Failed(val message: String) : ProvisionState
}
