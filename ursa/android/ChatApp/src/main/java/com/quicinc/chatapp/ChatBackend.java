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
        copyAssetsDir("models", externalDir);
        copyAssetsDir("htp_config", externalDir);
        Log.i("ChatBackend", "external path = " + externalDir);

        Path htpConfigPath = Paths.get(externalDir, "htp_config", supportedSocModel.get(socModel));
        String modelName = "llama3_2_3b";
        Path modelPath = Paths.get(externalDir, "models", modelName);

        Log.i("ChatBackend", "htp config path = " + htpConfigPath.toString());
        Log.i("ChatBackend", "model path = " + modelPath.toString());
        File modelDir = modelPath.toFile();

        // Initialize Genie
        genieWrapper = new GenieWrapper(htpConfigPath.toString(), modelPath.toString());
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