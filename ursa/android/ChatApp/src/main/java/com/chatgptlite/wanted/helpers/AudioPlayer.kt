package com.chatgptlite.wanted.helpers

import android.media.MediaPlayer
import java.io.File
import java.io.IOException

class AudioPlayer {
    private var player: MediaPlayer? = null

    fun startPlaying(file: File) {
        player = MediaPlayer().apply {
            try {
                setDataSource(file.absolutePath)
                prepare()
                start()
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }

    fun stopPlaying() {
        player?.release()
        player = null
    }
}