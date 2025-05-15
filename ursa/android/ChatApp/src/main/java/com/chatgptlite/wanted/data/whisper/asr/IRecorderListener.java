package com.chatgptlite.wanted.data.whisper.asr;

public interface IRecorderListener {
    void onUpdateReceived(String message);

    void onDataReceived(float[] samples);
}
