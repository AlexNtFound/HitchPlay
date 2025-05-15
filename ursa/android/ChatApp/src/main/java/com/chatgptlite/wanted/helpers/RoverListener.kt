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
    private val onMessageReceived: (String) -> Unit
) : WebSocketListener() {

    override fun onOpen(webSocket: WebSocket, response: Response) {
        Log.d("WebSocket", "Connection Opened")
        val subscribeMessage = """
            {
                "op": "subscribe",
                "topic": "$topic"
            }
        """.trimIndent()
        webSocket.send(subscribeMessage)
    }

    override fun onMessage(webSocket: WebSocket, text: String) {
        Log.d("WebSocket", "Message received: $text")
        onMessageReceived(text) // Pass the received message to the callback
    }

    override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
        Log.d("WebSocket", "Message received (bytes): $bytes")
        onMessageReceived(bytes.utf8()) // Pass the received bytes as a string
    }

    override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
        webSocket.close(1000, null)
        Log.d("WebSocket", "Connection Closing: $code / $reason")
    }

    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
        Log.e("WebSocket", "Error: ${t.message}")
    }
}

