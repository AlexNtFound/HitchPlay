package com.chatgptlite.wanted.helpers

import android.app.Application
import android.content.Context
import dagger.hilt.android.HiltAndroidApp
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStream

@HiltAndroidApp
class BaseApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        val extensionsToCopy = arrayOf("pcm", "bin", "wav", "tflite")
        copyAssetsWithExtensionsToDataFolder(this, extensionsToCopy)
    }

    private fun copyAssetsWithExtensionsToDataFolder(context: Context, extensions: Array<String>) {
        val assetManager = context.assets
        try {
            // Specify the destination directory in the app's data folder
            val destFolder = context.filesDir.absolutePath

            for (extension in extensions) {
                // List all files in the assets folder with the specified extension
                val assetFiles = assetManager.list("")
                for (assetFileName in assetFiles!!) {
                    if (assetFileName.endsWith(".$extension")) {
                        val outFile = File(destFolder, assetFileName)
                        if (outFile.exists()) continue

                        val inputStream = assetManager.open(assetFileName)
                        val outputStream: OutputStream = FileOutputStream(outFile)

                        // Copy the file from assets to the data folder
                        val buffer = ByteArray(1024)
                        var read: Int
                        while ((inputStream.read(buffer).also { read = it }) != -1) {
                            outputStream.write(buffer, 0, read)
                        }

                        inputStream.close()
                        outputStream.flush()
                        outputStream.close()
                    }
                }
            }
        } catch (e: IOException) {
            e.printStackTrace()
        }
    }

}