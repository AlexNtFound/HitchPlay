package com.chatgptlite.wanted.data.whisper.asr;

public interface IWhisperListener {
    void onUpdateReceived(String message);
    void onResultReceived(String result);
}
