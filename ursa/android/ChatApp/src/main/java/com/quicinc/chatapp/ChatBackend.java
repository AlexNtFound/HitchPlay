package com.quicinc.chatapp;

import android.content.Context;
import android.content.res.AssetManager;
import android.os.Build;
import android.util.Log;

import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;

public class ChatBackend {
    static {
        System.loadLibrary("chatapp");
    }

    private final GenieWrapper genieWrapper;
    private final Context context;

    public ChatBackend(Context context) throws IOException {
        this.context = context;

        // Supported SoCs
        HashMap<String, String> supportedSocModel = new HashMap<>();
        supportedSocModel.put("SM8750", "qualcomm-snapdragon-8-elite.json");
        supportedSocModel.put("SM8650", "qualcomm-snapdragon-8-gen3.json");
        supportedSocModel.put("QCS8550", "qualcomm-snapdragon-8-gen2.json");

        String socModel = Build.SOC_MODEL;
        if (!supportedSocModel.containsKey(socModel)) {
            throw new RuntimeException("Unsupported device: " + socModel);
        }

        String externalDir = context.getExternalCacheDir().getAbsolutePath();
        // Copy config/tokenizer from assets (small files bundled in APK)
        copyAssetsDir("models", externalDir);
        copyAssetsDir("htp_config", externalDir);
        Log.i("ChatBackend", "external path = " + externalDir);

        Path htpConfigPath = Paths.get(externalDir, "htp_config", supportedSocModel.get(socModel));
        String modelName = "qwen2_5_7b_instruct";
        Path modelPath = Paths.get(externalDir, "models", modelName);

        // Model .bin files must be pre-loaded on device via adb push.
        // They are too large (~4.8GB) to bundle in the APK.
        // Expected location: <externalCacheDir>/models/qwen2_5_7b_instruct/*.bin
        File modelDir = modelPath.toFile();
        File[] binFiles = modelDir.listFiles((dir, name) -> name.endsWith(".bin"));
        if (binFiles == null || binFiles.length == 0) {
            throw new RuntimeException(
                "Model .bin files not found at " + modelPath + ". " +
                "Push them via: adb push <bin_files> " +
                "/storage/emulated/0/Android/data/com.quicinc.chatapp/cache/models/" + modelName + "/"
            );
        }
        Log.i("ChatBackend", "Found " + binFiles.length + " bin files in " + modelPath);

        Log.i("ChatBackend", "htp config path = " + htpConfigPath.toString());
        Log.i("ChatBackend", "model path = " + modelPath.toString());

        // Initialize Genie
        genieWrapper = new GenieWrapper(modelPath.toString(), htpConfigPath.toString());
        Log.i("ChatApp", modelName + " Loaded.");
    }

    public String getResponse(String userInput) {
        // Send user input through MessageSender and get the response
//        MessageSender sender = new MessageSender(genieWrapper);
//        return sender.sendMessageSync(userInput);  // synchronous call
        return "Mock response to: " + userInput;
    }

    private void copyAssetsDir(String inputAssetRelPath, String outputPath) throws IOException {
        File outputAssetPath = new File(Paths.get(outputPath, inputAssetRelPath).toString());
        AssetManager assets = context.getAssets();

        String[] subAssetList = assets.list(inputAssetRelPath);
        if (subAssetList == null || subAssetList.length == 0) {
            if (!outputAssetPath.exists()) {
                copyFile(inputAssetRelPath, outputAssetPath);
            }
            return;
        }

        if (!outputAssetPath.exists()) {
            outputAssetPath.mkdirs();
        }

        for (String subAssetName : subAssetList) {
            String inputSubAssetPath = Paths.get(inputAssetRelPath, subAssetName).toString();
            copyAssetsDir(inputSubAssetPath, outputPath);
        }
    }

    private void copyFile(String inputFilePath, File outputAssetFile) throws IOException {
        InputStream in = context.getAssets().open(inputFilePath);
        OutputStream out = new FileOutputStream(outputAssetFile);
        byte[] buffer = new byte[1024 * 1024];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
        }
        in.close();
        out.close();
    }


}