package com.chatgptlite.wanted.helpers

import retrofit2.http.Body
import retrofit2.http.POST
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.http.GET
import okhttp3.OkHttpClient
import android.util.Log
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.*
import okhttp3.*
import okio.ByteString


interface RoverAPIService {
    @POST("/cmd")
    suspend fun sendCommand(@Body commandRequest: CommandRequest): Response<StatusResponse>

    @GET("/ping")
    suspend fun ping(): Response<PingResponse>
}

data class CommandRequest(val cmd: String)
data class StatusResponse(val status: String, val request: String?)
data class PingResponse(val status: String, val message: String)

suspend fun sendPing(addr: String, port: String) : Response<PingResponse> {
    val retrofit = Retrofit.Builder()
        .baseUrl("http://$addr:$port")
        .addConverterFactory(GsonConverterFactory.create())
        .build()
    val apiService = retrofit.create(RoverAPIService::class.java)
    val response = apiService.ping()
    return response
}

suspend fun sendMessage(addr: String, port: String, textToSend: String): Response<StatusResponse> {
    val client = OkHttpClient.Builder()
        .connectTimeout(30, TimeUnit.SECONDS) // Max time to establish connection
        .readTimeout(30, TimeUnit.SECONDS)    // Max time to read response
        .writeTimeout(30, TimeUnit.SECONDS)   // Max time to write request
        .build()

    val retrofit = Retrofit.Builder()
        .baseUrl("http://$addr:$port")
        .client(client) // Set the custom client with timeouts
        .addConverterFactory(GsonConverterFactory.create())
        .build()

    val apiService = retrofit.create(RoverAPIService::class.java)
    val commandRequest = CommandRequest(cmd = textToSend)
    val response = apiService.sendCommand(commandRequest)

    if (response.isSuccessful) {
        val statusResponse = response.body()
        Log.d("sendMessage", "Response Success: ${statusResponse?.toString() ?: "No body"}")
    } else {
        Log.e("sendMessage", "Response Error: ${response.code()} - ${response.errorBody()?.string()}")
    }
    return response
}



