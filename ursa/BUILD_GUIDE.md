# Ursa – Build & Model Setup Guide

## Quick Start (For Collaborators)

If you just want to build and run the app using the pre-exported Qwen 2.5-7B model, follow these steps. No QAI Hub account or model export needed.

### 1. Install Tools

| Tool | Version | Notes |
|------|---------|-------|
| Android Studio | Ladybug+ | Arctic Fox minimum, Ladybug recommended |
| Gradle | 8.9 | **Must use local installation, not wrapper** (see [Gradle Setup](#gradle-setup-important)) |
| QNN SDK (QAIRT) | 2.42.0 | From [qpm.qualcomm.com](https://qpm.qualcomm.com/#/main/tools/details/qualcomm_ai_engine_direct) (free registration required) |
| JDK | 17 | Required by `compileOptions` in build.gradle |

Android Studio will auto-install the NDK and SDK platform (compileSdk 34, minSdk 31) on first sync.

### 2. Download Model Files

Download the following from the shared Google Drive: **[Model Files](https://drive.google.com/drive/folders/1NxGWSq7-jmqbufWJBaY9LnsQNcMQu0Sp?usp=drive_link)**

| File | Size | Description |
|------|------|-------------|
| `qwen2_5_7b_instruct_part_1_of_6.bin` | ~1.1 GB | Model weights (part 1) |
| `qwen2_5_7b_instruct_part_2_of_6.bin` | ~675 MB | Model weights (part 2) |
| `qwen2_5_7b_instruct_part_3_of_6.bin` | ~675 MB | Model weights (part 3) |
| `qwen2_5_7b_instruct_part_4_of_6.bin` | ~675 MB | Model weights (part 4) |
| `qwen2_5_7b_instruct_part_5_of_6.bin` | ~675 MB | Model weights (part 5) |
| `qwen2_5_7b_instruct_part_6_of_6.bin` | ~1.0 GB | Model weights (part 6) |
| `tokenizer.json` | ~7 MB | Qwen 2.5 tokenizer |
| `genie-config.json` | ~2 KB | Genie runtime config |

Total download: **~4.8 GB**

### 3. Place Model Config Files

Copy `tokenizer.json` and `genie-config.json` into:
```
android/ChatApp/src/main/assets/models/qwen2_5_7b_instruct/
```

This directory is gitignored. You must create it and place these two files manually. The `.bin` files do **NOT** go here — they are too large for the APK (see step 6).

### 4. Set QNN SDK Path

Edit `android/ChatApp/build.gradle` line 15 to point to your local QNN SDK:
```groovy
def qnnSDKLocalPath="C:/Qualcomm/AIStack/QAIRT/2.42.0.251225"
```

Use forward slashes. The path must match the exact QAIRT version (**2.42.0**) used to compile the `.bin` files.

### 5. Build

1. Open the `android/` folder in Android Studio
2. Set Gradle to local installation (see [Gradle Setup](#gradle-setup-important))
3. Sync Gradle when prompted
4. **Build → Clean Project**, then **Build → Rebuild Project**
5. Connect your device and hit **Run**

> Clean build is recommended on first setup or after changing the QNN SDK path or native C++ code. For Java/Kotlin-only changes, a regular **Run** is sufficient.

### 6. Push Model Weights to Device

The `.bin` files must be pushed separately after the APK is installed. They cannot be bundled in the APK due to Android's ZIP32 4 GB size limit.

**PowerShell:**
```powershell
$adb = "C:\Users\franc\AppData\Local\Android\Sdk\platform-tools\adb.exe"
$src = "C:\Users\franc\Downloads\model_bins"
$dst = "/storage/emulated/0/Android/data/com.quicinc.chatapp/cache/models/qwen2_5_7b_instruct/"

& $adb push "$src\qwen2_5_7b_instruct_part_1_of_6.bin" $dst
& $adb push "$src\qwen2_5_7b_instruct_part_2_of_6.bin" $dst
& $adb push "$src\qwen2_5_7b_instruct_part_3_of_6.bin" $dst
& $adb push "$src\qwen2_5_7b_instruct_part_4_of_6.bin" $dst
& $adb push "$src\qwen2_5_7b_instruct_part_5_of_6.bin" $dst
& $adb push "$src\qwen2_5_7b_instruct_part_6_of_6.bin" $dst
```

> Replace `$adb` and `$src` with your local paths. If `adb` is in your system PATH, you can use just `adb` instead of the full path. The default Android SDK location on Windows is `C:\Users\<you>\AppData\Local\Android\Sdk\platform-tools\`.

### 7. Launch

Open the app on the device. It should load without crashing. If it crashes, see [Troubleshooting](#troubleshooting).

**You only need to push the `.bin` files once.** They persist on the device across app reinstalls. You only need to re-push if you **uninstall** the app (which wipes `Android/data/com.quicinc.chatapp/`).

---

## Gradle Setup (Important)

Android Studio defaults to the Gradle wrapper, but this project requires **Gradle 8.9** installed locally.

1. Download Gradle 8.9 from [gradle.org/releases](https://gradle.org/releases/)
2. Extract to a permanent location (e.g., `C:\Gradle\gradle-8.9`)
3. In Android Studio: **File → Settings → Build, Execution, Deployment → Build Tools → Gradle**
4. Select **"Specified location"** (not "Gradle wrapper")
5. Set the path to your extracted Gradle directory (e.g., `C:\Gradle\gradle-8.9`)
6. Click **Apply → OK**

---

## QNN SDK Setup

1. Register at [qpm.qualcomm.com](https://qpm.qualcomm.com)
2. Download **Qualcomm AI Engine Direct** (QAIRT) — version **2.42.0** for the current model bins
3. Extract to a known path (e.g., `C:\Qualcomm\AIStack\QAIRT\2.42.0.251225`)
4. Update `android/ChatApp/build.gradle` line 15:
   ```groovy
   def qnnSDKLocalPath="C:/Qualcomm/AIStack/QAIRT/2.42.0.251225"
   ```

> **Critical**: The QNN SDK version must match the QAIRT version used to compile the `.bin` files. A mismatch causes `Failed to create Genie dialog` crashes at runtime. The current bins were compiled with **QAIRT 2.42.0**.

---

## Supported Devices

The app checks `android.os.Build.SOC_MODEL` at startup. Currently supported chipsets are defined in both `MainActivity.java` and `MainActivity.kt`:

| SoC Model | Chipset | HTP Config File |
|-----------|---------|-----------------|
| SM8750 | Snapdragon 8 Elite | `qualcomm-snapdragon-8-elite.json` |
| SM8650 | Snapdragon 8 Gen 3 | `qualcomm-snapdragon-8-gen3.json` |
| QCS8550 | Snapdragon 8 Gen 2 | `qualcomm-snapdragon-8-gen2.json` |

To add a new device, add a mapping in **both** `MainActivity.java` and `MainActivity.kt` inside `supportedSocModel`, and create the corresponding HTP config JSON in `android/ChatApp/src/main/assets/htp_config/`.

---

## Model Architecture

### File Layout

```
android/ChatApp/src/main/assets/
├── models/
│   └── qwen2_5_7b_instruct/          ← gitignored, must be created manually
│       ├── genie-config.json          ← Genie runtime config (uses placeholders)
│       └── tokenizer.json             ← HuggingFace tokenizer (~7 MB)
├── htp_config/
│   ├── htp_backend_ext_config.json    ← generic HTP backend config
│   ├── qualcomm-snapdragon-8-elite.json
│   ├── qualcomm-snapdragon-8-gen2.json
│   └── qualcomm-snapdragon-8-gen3.json

On device (pushed via adb):
/storage/emulated/0/Android/data/com.quicinc.chatapp/cache/models/qwen2_5_7b_instruct/
├── genie-config.json                  ← copied from APK assets on first launch
├── tokenizer.json                     ← copied from APK assets on first launch
├── qwen2_5_7b_instruct_part_1_of_6.bin  ← pushed via adb (~1.1 GB)
├── qwen2_5_7b_instruct_part_2_of_6.bin  ← pushed via adb (~675 MB)
├── qwen2_5_7b_instruct_part_3_of_6.bin  ← pushed via adb (~675 MB)
├── qwen2_5_7b_instruct_part_4_of_6.bin  ← pushed via adb (~675 MB)
├── qwen2_5_7b_instruct_part_5_of_6.bin  ← pushed via adb (~675 MB)
└── qwen2_5_7b_instruct_part_6_of_6.bin  ← pushed via adb (~1.0 GB)
```

### Why `.bin` Files Are Side-Loaded

Android APKs use ZIP32, which has a **4 GB maximum** file size. The 6 model bins total ~4.8 GB, exceeding this limit. They must be pushed to the device's external cache via `adb` after the APK is installed.

### Path Resolution at Runtime

The C++ `LoadModelConfig` function (in `GenieWrapper.cpp`) performs regex replacement on `genie-config.json` before passing it to the Genie runtime:

| Placeholder | Replaced With |
|-------------|---------------|
| `<models_path>` | Absolute path to the model directory on device |
| `<htp_backend_ext_path>` | Absolute path to the SoC-specific HTP config |
| `<tokenizer_path>` | Absolute path to `tokenizer.json` |

**Your `genie-config.json` must use these placeholders.** Literal relative paths (like `"tokenizer.json"` or `"htp_backend_ext_config.json"`) will not be resolved and the app will crash with `Failed to create Genie dialog`.

### Asset Copy Behavior

On first launch, `copyAssetsDir()` in `MainActivity` copies assets from the APK to external cache. **It skips files that already exist.** If you update `genie-config.json` or `tokenizer.json` in the APK after the first install, you must either:
- Uninstall and reinstall the app, **or**
- Push the updated file via `adb push` to the on-device path

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| App crashes immediately | Missing `.bin` files on device | Push bins via adb (see [step 6](#6-push-model-weights-to-device)) |
| `Failed to create Genie config` in logcat | `genie-config.json` is malformed or missing | Check JSON syntax; ensure file is in assets directory |
| `Failed to create Genie dialog` in logcat | SDK version mismatch, missing placeholders, or bins not found | Verify QNN SDK version matches bins; check that genie-config uses `<placeholders>`; verify adb push path |
| `Model .bin files not found` in logcat | adb push went to wrong path | Check target path matches `getExternalCacheDir()/models/<model_name>/` |
| `Unsupported device` toast | Device SoC not in `supportedSocModel` map | Add your device's `Build.SOC_MODEL` to both MainActivity files |
| `[CXX5304] SDK XML version` warning | NDK version mismatch (non-fatal) | Ignore — does not affect the build |
| App works but model output is wrong | Updated genie-config not copied to device | Push updated config via adb (asset copy skips existing files) |

**Logcat filter for debugging:**
```
level:ERROR package:com.quicinc.chatapp
```

---

## Performance Tuning (genie-config.json)

These parameters can be adjusted in `genie-config.json` and pushed via adb:

| Parameter | Effect | Notes |
|-----------|--------|-------|
| `context.size` | Smaller = faster prefill | 2048 is enough for short rover commands; 4096 for longer conversations |
| `n-threads` | More threads = faster decode | Don't exceed available performance cores |
| `cpu-mask` | Which CPU cores to use | `0xf0` = cores 4-7 on 8 Elite; `0xe0` = cores 5-7 |
| `temp` | Lower = more deterministic | 0.3 is good for structured command output; 0.8 for creative text |
| `top-k` / `top-p` | Sampling diversity | Lower values = more focused output |

---

## Current Configuration

- **Model**: [Qwen2.5-7B-Instruct](https://huggingface.co/Qwen/Qwen2.5-7B-Instruct)
- **Compiled with**: QAIRT 2.42.0
- **QNN SDK**: 2.42.0.251225
- **Target device**: ASUS ROG Phone 9 Pro (SM8750 / Snapdragon 8 Elite)
- **Bin files**: 6 parts, ~4.8 GB total (side-loaded via adb)
- **Prompt format**: ChatML (`<|im_start|>` / `<|im_end|>`)
- **Min Android SDK**: 31 (Android 12)

---

## Swapping to a Different LLM Model (Advanced)

This section is for exporting and integrating a completely new model. Most collaborators should use the pre-exported Qwen 2.5-7B files from the Quick Start above.

### Step 1: Export Model Bins from QAI Hub

You need WSL (Windows Subsystem for Linux) with sufficient memory.

**WSL memory config** — create/edit `C:\Users\<you>\.wslconfig`:
```ini
[wsl2]
memory=24GB
swap=8GB
```

Then restart WSL: `wsl --shutdown` from PowerShell.

**In WSL:**
```bash
python3 -m venv ~/qai_env
source ~/qai_env/bin/activate
pip install "qai-hub-models[<model_name>]"
qai-hub configure --api_token YOUR_TOKEN

python3 -m qai_hub_models.models.<model_name>.export \
  --chipset qualcomm-snapdragon-8-elite \
  --skip-profiling \
  --output-dir ~/model_output/genie_bundle
```

Replace `<model_name>` with the QAI Hub model ID (e.g., `qwen2_5_7b_instruct`). Change `--chipset` to match your target device.

This uploads the model to QAI Hub cloud for compilation. The output is a set of `.bin` files. You need a QAI Hub account and API token from [aihub.qualcomm.com](https://aihub.qualcomm.com).

> **Note**: QAI Hub only produces `.bin` files. You must create `genie-config.json` yourself and download `tokenizer.json` from HuggingFace separately.

### Step 2: Download the Tokenizer

```bash
pip install transformers
python3 -c "
from transformers import AutoTokenizer
t = AutoTokenizer.from_pretrained('<huggingface_model_id>')
t.save_pretrained('./tokenizer_output')
"
```

Copy `tokenizer_output/tokenizer.json` into `android/ChatApp/src/main/assets/models/<model_name>/`.

### Step 3: Create genie-config.json

Create `android/ChatApp/src/main/assets/models/<model_name>/genie-config.json`. Use the current Qwen config or a template from `tutorials/llm_on_genie/configs/genie/` as a starting point.

**You must use placeholders for all paths:**
```json
{
    "dialog": {
        "version": 1,
        "type": "basic",
        "context": {
            "version": 1,
            "size": 2048,
            "n-vocab": 152064,
            "bos-token": -1,
            "eos-token": 151645
        },
        "sampler": {
            "version": 1,
            "seed": 42,
            "temp": 0.8,
            "top-k": 40,
            "top-p": 0.95
        },
        "tokenizer": {
            "version": 1,
            "path": "<tokenizer_path>"
        },
        "engine": {
            "version": 1,
            "n-threads": 4,
            "backend": {
                "version": 1,
                "type": "QnnHtp",
                "QnnHtp": {
                    "version": 1,
                    "use-mmap": true,
                    "spill-fill-bufsize": 0,
                    "mmap-budget": 0,
                    "poll": false,
                    "pos-id-dim": 64,
                    "cpu-mask": "0xf0",
                    "kv-dim": 128,
                    "rope-theta": 1000000,
                    "allow-async-init": false
                },
                "extensions": "<htp_backend_ext_path>"
            },
            "model": {
                "version": 1,
                "type": "binary",
                "binary": {
                    "version": 1,
                    "ctx-bins": [
                        "<models_path>/model_part_1.bin",
                        "<models_path>/model_part_2.bin"
                    ]
                }
            }
        }
    }
}
```

**Fields to update per model:**

| Field | Where to Find |
|-------|---------------|
| `n-vocab` | Model's HuggingFace `config.json` → `vocab_size` |
| `eos-token` | HuggingFace `tokenizer_config.json` → `eos_token` ID |
| `bos-token` | HuggingFace `tokenizer_config.json` → `bos_token` ID (or -1 if none) |
| `kv-dim` | `config.json` → `hidden_size / num_attention_heads` |
| `pos-id-dim` | `config.json` → `hidden_size / num_attention_heads / 2` |
| `rope-theta` | `config.json` → `rope_theta` |
| `ctx-bins` | List of `.bin` filenames from QAI Hub export, each prefixed with `<models_path>/` |

### Step 4: Update Code References

You need to update the model name string in **four files**:

**`android/ChatApp/build.gradle`** (line 17):
```groovy
def models = ["<model_name>"]
```

**`android/ChatApp/src/main/java/com/quicinc/chatapp/MainActivity.java`** (line 129):
```java
intent.putExtra(Conversation.cConversationActivityKeyModelName, "<model_name>");
```

**`android/ChatApp/src/main/java/com/chatgptlite/wanted/MainActivity.kt`** (line 317):
```kotlin
val cConversationActivityKeyModelName = "<model_name>"
```

**`android/ChatApp/src/main/java/com/quicinc/chatapp/ChatBackend.java`** (line 41):
```java
String modelName = "<model_name>";
```

Also update the button label in **`android/ChatApp/src/main/res/values/strings.xml`**:
```xml
<string name="chat_with_llama_3_2_3b">Chat with <Display Name></string>
```

### Step 5: Update the Prompt Template

Edit `android/ChatApp/src/main/cpp/PromptHandler.cpp` to match the new model's chat format.

Each model family uses different special tokens:

| Model | Format | System Prefix | User Prefix | End-of-Turn | Assistant Header |
|-------|--------|---------------|-------------|-------------|------------------|
| Llama 3.x | Llama3 | `<\|begin_of_text\|><\|start_header_id\|>system<\|end_header_id\|>\n` | `<\|start_header_id\|>user<\|end_header_id\|>\n` | `<\|eot_id\|>` | `<\|start_header_id\|>assistant<\|end_header_id\|>\n` |
| Qwen 2.5 | ChatML | `<\|im_start\|>system\n` | `<\|im_start\|>user\n` | `<\|im_end\|>\n` | `<\|im_start\|>assistant\n` |

Only the special token constants need to change. The system prompt body (rover control instructions, ROS2 service formats, examples) stays the same.

Check the model's HuggingFace page for the correct chat template.

### Step 6: Update .gitignore

Add to `android/.gitignore`:
```
/src/main/assets/models/<model_name>/**
```

### Step 7: Build and Deploy

1. **Build → Clean Project**, then **Build → Rebuild Project** in Android Studio
2. Install the APK to the device
3. Push `.bin` files via adb (see [step 6 in Quick Start](#6-push-model-weights-to-device))
4. Launch the app

### Step 8: Upload Model Files for Team

After verifying the new model works, upload these files to shared storage (Google Drive, etc.) so collaborators can use the [Quick Start](#quick-start-for-collaborators) flow:
- All `.bin` files
- `tokenizer.json`
- `genie-config.json`

Update the download link in this guide's Quick Start section.
