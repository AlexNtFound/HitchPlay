# Ursa – Build Guide

Workflow is terminal-first. You don't need Android Studio open at all to build and run, just the SDK + adb on PATH and a phone in dev mode. Android Studio works too if you prefer it.

---

## Prerequisites

- **Android SDK + platform-tools** (for `adb`). Android Studio installs these; if you don't want the IDE, install them via [`commandlinetools`](https://developer.android.com/studio#command-tools).
- **JDK 17** — install from [adoptium.net](https://adoptium.net/temurin/releases/?version=17) (Windows MSI; tick "Set JAVA_HOME" and "Add to PATH" during install). Kotlin 1.8.10's kapt is incompatible with JDK 21+, so a JDK 17 install is required even if you already have Android Studio's JBR (which is JDK 21 on Ladybug+). `build.cmd` finds Adoptium's JDK 17 automatically.
- **QNN SDK (QAIRT) 2.42.0** — register at [qpm.qualcomm.com](https://qpm.qualcomm.com/#/main/tools/details/Qualcomm_AI_Runtime_SDK?version=2.42.0.251225), download and run.
- A **Snapdragon 8 Gen 2 / Gen 3 / Elite** device with USB debugging on.

You don't need to install Gradle, set `JAVA_HOME` manually, or install a separate JDK 17 — the `build.cmd` / `build.sh` wrapper finds Android Studio's JBR automatically, and Foojay provisions JDK 17 for Kotlin compilation if needed.

---

## Setup (once per machine)

**1. Tell Gradle where your QNN SDK is.** Add to `android/local.properties`:

```properties
qnn.sdk.dir=C:/Qualcomm/AIStack/QAIRT/2.42.0.251225
```

Forward slashes, even on Windows. This file is gitignored.

**2. Place model config files.** Download `tokenizer.json` and `genie-config.json` from the [model release page](https://github.com/AlexNtFound/HitchPlay/releases/tag/model-qwen2_5_7b_instruct-v1) and drop them into:

```
android/ChatApp/src/main/assets/models/qwen2_5_7b_instruct/
```

Don't download the `.bin` files — those are fetched automatically at runtime.

**3. Make `adb` available** (once per terminal, or add to your PATH permanently):

```powershell
Set-Alias adb "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe"
```

---

## Build, install, run

From `android/`:

```powershell
.\build.cmd assembleDebug
adb install -r ChatApp\build\outputs\apk\debug\ChatApp-debug.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

(macOS/Linux: `./build.sh assembleDebug`. First time on Unix: `chmod +x build.sh gradlew`.)

`build.cmd` / `build.sh` is a thin wrapper around `gradlew` that auto-detects Java — checks `JAVA_HOME`, then Android Studio's bundled JBR, then common Adoptium / `java_home` paths. If you'd rather call Gradle directly (because you've set `JAVA_HOME` yourself), `.\gradlew.bat assembleDebug` still works.

The first run does a few one-time things automatically: downloads Gradle 8.9, provisions JDK 17, configures CMake, copies QNN libs into the build. Expect the first build to take 5–10 minutes; subsequent builds are ~30 seconds.

If the build fails with a message starting `[Ursa]`, that's an intentional error — read it; it tells you exactly what to fix.

---

## First launch — model auto-download

The first time you launch on a device that doesn't have the model yet, you'll see a **"Setting up the on-device model"** screen. The app downloads ~4.8 GB of `.bin` files from this repo's GitHub Releases over Wi-Fi (~3 minutes on fast home Wi-Fi). The phone needs to be on actual Wi-Fi — adb over USB does not give the phone internet.

Watch progress in another terminal:

```powershell
adb logcat ModelProvisioner:V *:S
```

After all 6 files download + verify, the chat UI loads. Subsequent launches skip this screen — the bins persist across reinstalls. They're only re-downloaded if you `adb uninstall`, clear app storage, or a new release changes the manifest.

---

## Android Studio (optional)

If you'd rather not use the terminal: open the `android/` folder in Android Studio, sync Gradle when prompted, hit Run. The wrapper config + Foojay JDK + Foojay-resolved toolchain make this Just Work without any Android Studio settings tweaking.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `[Ursa] Could not find a JDK` (from `build.cmd`) | Install Android Studio (bundled JBR is auto-detected) or OpenJDK 17 from [adoptium.net](https://adoptium.net) |
| `Error: JAVA_HOME is not set` (when calling `gradlew.bat` directly) | Use `.\build.cmd` instead — it auto-detects Java |
| `IllegalAccessError: superclass access check failed: ... com.sun.tools.javac` | Make sure your `gradle.properties` has the `--add-exports` block (already committed). If you edited it, re-pull |
| `Invalid Java installation found at ... .gradle\.tmp\jdks\...` | Foojay's JDK download corrupted. Run: `Remove-Item -Recurse -Force "$env:USERPROFILE\.gradle\.tmp\jdks", "$env:USERPROFILE\.gradle\jdks"` then rebuild |
| `[Ursa] QNN SDK path is not configured` | Add `qnn.sdk.dir=...` to `android/local.properties` |
| `[Ursa] Missing model assets for 'qwen2_5_7b_instruct'` | Place `tokenizer.json` and `genie-config.json` in `assets/models/qwen2_5_7b_instruct/` |
| `[Ursa] Stray model .bin files found` | Delete the `.bin` files from `assets/models/...` — bins are downloaded at runtime, not bundled |
| `Unknown Kotlin JVM target: 21` | Run `.\gradlew.bat clean` and rebuild — Foojay should fix it |
| App stuck on "Setting up the on-device model", "Unable to resolve host" | Phone has no Wi-Fi. Connect to Wi-Fi, then tap Retry |
| `Setup failed: Hash mismatch` | The release file isn't byte-identical to the manifest. Re-upload from the source-of-truth bin |
| `Unsupported device` toast | Device SoC isn't in the supported list. Add to `MainActivity.kt` and create the matching HTP config |
| App crashes immediately on launch (after bins download) | QNN SDK version mismatch. Confirm `qnn.sdk.dir` points at QAIRT **2.42.0** |

---

## Performance tuning

`genie-config.json` parameters (push to device with `adb push <file> /storage/emulated/0/Android/data/com.quicinc.chatapp/files/models/qwen2_5_7b_instruct/genie-config.json`):

- `context.size` — 2048 for short rover commands, 4096 for longer conversations
- `n-threads` — don't exceed your device's performance core count
- `cpu-mask` — `0xf0` = cores 4–7 on Snapdragon 8 Elite
- `temp` — 0.3 for structured output, 0.8 for creative
- `top-k` / `top-p` — lower = more focused

---

## File layout

```
APK assets (in repo):
  assets/models/qwen2_5_7b_instruct/{tokenizer.json, genie-config.json, manifest.json}
  assets/htp_config/{qualcomm-snapdragon-8-{elite,gen3,gen2}.json, htp_backend_ext_config.json}

On device (auto-populated):
  /storage/emulated/0/Android/data/com.quicinc.chatapp/files/models/qwen2_5_7b_instruct/
    tokenizer.json, genie-config.json, manifest.json   ← copied from APK on first launch
    qwen2_5_7b_instruct_part_{1..6}_of_6.bin           ← downloaded from GitHub Releases
```

---

## Maintainers: publishing a new bin release

1. Compile the bins (use the QAI Hub workflow if it's a new model).
2. Compute size + SHA-256 for each file:
   ```powershell
   Get-ChildItem .\*.bin | ForEach-Object {
     "{0}  {1}  {2}" -f $_.Name, $_.Length, (Get-FileHash $_.FullName -Algorithm SHA256).Hash.ToLower()
   }
   ```
3. Create a GitHub Release at https://github.com/AlexNtFound/HitchPlay/releases/new with tag `model-<modelName>-v<n>`. Upload all `.bin` files (each must be ≤2 GB). **Publish, don't draft.**
4. Update `manifest.json` at `android/ChatApp/src/main/assets/models/<modelName>/manifest.json` with the new sizes, hashes, URLs, `releaseTag`, and a bumped `manifestVersion`.
5. Verify one URL resolves to the right size:
   ```powershell
   $u = "https://github.com/AlexNtFound/HitchPlay/releases/download/<tag>/<file>.bin"
   (Invoke-WebRequest -Uri $u -Method Head -MaximumRedirection 5 -UseBasicParsing).Headers["Content-Length"]
   ```
6. Build a new APK and ship. Existing devices detect the manifest change and re-download only the affected files.

---

## Maintainers: swapping the LLM (advanced)

For exporting a brand-new model from QAI Hub, follow the QAI Hub model export workflow ([aihub.qualcomm.com](https://aihub.qualcomm.com)) to produce `.bin` files, then:

1. Get `tokenizer.json` from the model's HuggingFace page.
2. Write `genie-config.json` using `<placeholders>` for paths (`<models_path>`, `<tokenizer_path>`, `<htp_backend_ext_path>`). Update `n-vocab`, `eos-token`, `kv-dim`, `pos-id-dim`, `rope-theta`, `ctx-bins` for the new model.
3. Update `def models = [...]` in `ChatApp/build.gradle` and `cConversationActivityKeyModelName` in `MainActivity.kt`.
4. Update prompt format constants in `cpp/PromptHandler.cpp` if the chat template differs (Qwen uses ChatML; Llama 3.x uses its own headers).
5. Add the new model dir to `.gitignore` (with `!manifest.json` exception).
6. Follow the *publishing a new bin release* steps above.

The Java `MainActivity.java` and `ChatBackend.java` files are dead code (commented out in AndroidManifest); don't bother editing them.

---

## Current configuration

- **Model**: Qwen2.5-7B-Instruct
- **QNN/QAIRT**: 2.42.0.251225
- **Target device**: Snapdragon 8 Elite / 8 Gen 3 / 8 Gen 2
- **Min Android SDK**: 31 (Android 12)
- **Bin files**: 6 parts, ~4.8 GB total, auto-downloaded
