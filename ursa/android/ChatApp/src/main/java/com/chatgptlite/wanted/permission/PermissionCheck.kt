package com.chatgptlite.wanted.permission

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.provider.Settings
import androidx.activity.ComponentActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

class PermissionCheck(private val context: Context) {
    private val RECORD_AUDIO_PERMISSION_REQUEST_CODE = 1

    fun checkAudioRecordingPermission(
        onPermissionGranted: () -> Unit,
        onPermissionDenied: () -> Unit
    ) {
        when {
            ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.RECORD_AUDIO
            ) == PackageManager.PERMISSION_GRANTED -> {
                // 权限已经被授予
                onPermissionGranted()
            }
            ActivityCompat.shouldShowRequestPermissionRationale(
                context as ComponentActivity,
                Manifest.permission.RECORD_AUDIO
            ) -> {
                onPermissionDenied()
            }
            else -> {
                ActivityCompat.requestPermissions(
                    context,
                    arrayOf(Manifest.permission.RECORD_AUDIO),
                    RECORD_AUDIO_PERMISSION_REQUEST_CODE
                )
            }
        }
    }

    fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
        onPermissionGranted: () -> Unit,
        onPermissionDenied: () -> Unit
    ) {
        when (requestCode) {
            RECORD_AUDIO_PERMISSION_REQUEST_CODE -> {
                if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    onPermissionGranted()
                } else {
                    onPermissionDenied()
                }
            }
        }
    }

    fun openAppSettings() {
        Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
            data = Uri.fromParts("package", context.packageName, null)
            context.startActivity(this)
        }
    }
}