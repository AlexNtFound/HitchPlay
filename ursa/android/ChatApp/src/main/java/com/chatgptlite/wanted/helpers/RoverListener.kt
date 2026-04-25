package com.chatgptlite.wanted.helpers

import android.util.Log
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString
import okhttp3.Response


class RoverWebSocketListener(
    private val topic: String,
    private val onConnected: ((String) -> Unit)? = null,
    private val onFailed: ((String, String) -> Unit)? = null,
    private val onMessageReceived: (String) -> Unit
) : WebSocketListener() {

    override fun onOpen(webSocket: WebSocket, response: Response) {
        Log.d("WebSocket", "Connection Opened for $topic")
        val subscribeMessage = """
            {
                "op": "subscribe",
                "topic": "$topic"
            }
        """.trimIndent()
        webSocket.send(subscribeMessage)
        onConnected?.invoke(topic)
    }

    override fun onMessage(webSocket: WebSocket, text: String) {
        Log.d("WebSocket", "Message received: $text")
        onMessageReceived(text)
    }

    override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
        Log.d("WebSocket", "Message received (bytes): $bytes")
        onMessageReceived(bytes.utf8())
    }

    override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
        webSocket.close(1000, null)
        Log.d("WebSocket", "Connection Closing: $code / $reason")
    }

    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
        Log.e("WebSocket", "Error on $topic: ${t.message}")
        onFailed?.invoke(topic, t.message ?: "unknown error")
    }
}

