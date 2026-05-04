// ---------------------------------------------------------------------
// Copyright (c) 2026 The Ursa Authors. SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
package com.quicinc.chatapp.provisioning

import android.content.Context
import org.json.JSONObject

/**
 * Describes the model bin set for a given on-device LLM.
 *
 * The manifest ships in APK assets at `models/<modelName>/manifest.json`.
 * It is the single source of truth for: bin file names, expected sizes,
 * SHA-256 hashes, and download URLs (GitHub Release assets).
 */
data class ModelManifest(
    val modelName: String,
    val qairtVersion: String,
    val manifestVersion: Int,
    val releaseTag: String,
    val files: List<ModelFile>
) {
    data class ModelFile(
        val name: String,
        val sizeBytes: Long,
        val sha256: String,
        val url: String
    )

    val totalBytes: Long get() = files.sumOf { it.sizeBytes }

    companion object {
        private const val ASSET_PATH_TEMPLATE = "models/%s/manifest.json"

        /**
         * Loads the manifest for [modelName] from APK assets. Throws on parse failure
         * because a malformed manifest is a build-time bug, not a runtime condition.
         */
        @Throws(IllegalStateException::class)
        fun load(context: Context, modelName: String): ModelManifest {
            val path = ASSET_PATH_TEMPLATE.format(modelName)
            val raw = context.assets.open(path).bufferedReader().use { it.readText() }
            val json = JSONObject(raw)
            val arr = json.getJSONArray("files")
            val files = ArrayList<ModelFile>(arr.length())
            for (i in 0 until arr.length()) {
                val o = arr.getJSONObject(i)
                files += ModelFile(
                    name = o.getString("name"),
                    sizeBytes = o.getLong("sizeBytes"),
                    sha256 = o.getString("sha256").lowercase(),
                    url = o.getString("url")
                )
            }
            check(files.isNotEmpty()) { "manifest for '$modelName' has no files" }
            return ModelManifest(
                modelName = json.getString("modelName"),
                qairtVersion = json.getString("qairtVersion"),
                manifestVersion = json.getInt("manifestVersion"),
                releaseTag = json.getString("releaseTag"),
                files = files
            )
        }
    }
}
