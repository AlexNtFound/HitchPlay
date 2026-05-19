# Ursa – Build Guide

Workflow is terminal-first. You don't need Android Studio open at all to build and run, just the SDK + adb on PATH and a phone in dev mode. Android Studio works too if you prefer it.

Tested on **Windows 10/11** and **Ubuntu 22.04+**. Where commands differ, both are shown side-by-side.

> **Just want to install the app, not build it?** Skip this guide entirely. Ask a maintainer for access to the private release repo at [AlexNtFound/HitchPlay-releases](https://github.com/AlexNtFound/HitchPlay-releases/releases) — the [latest v0.1.0 release](https://github.com/AlexNtFound/HitchPlay-releases/releases/tag/app-v0.1.0) has a ready-to-install APK. You'll only need `adb` on your PC, not JDK/QNN SDK/Gradle. See the README for the installer path.

---

## Prerequisites

- **Android SDK + platform-tools** (for `adb`).
  - *Windows*: Android Studio installs these. If you don't want the IDE, install them via [`commandlinetools`](https://developer.android.com/studio#command-tools).
  - *Linux*: `sudo apt install android-sdk-platform-tools` (gets `adb` only), or install Android Studio for the full SDK.
- **JDK 17** — Kotlin 1.8.10's kapt is incompatible with JDK 21+, so JDK 17 is required even if you already have Android Studio's JBR (which is JDK 21 on Ladybug+).
  - *Windows*: install from [adoptium.net](https://adoptium.net/temurin/releases/?version=17) (MSI; tick "Set JAVA_HOME" and "Add to PATH" during install). `build.cmd` finds Adoptium's JDK 17 automatically.
  - *Linux*: `sudo apt install openjdk-17-jdk`. `build.sh` finds it via `java` on PATH.
- **QNN SDK (QAIRT) 2.42.0** — register at [qpm.qualcomm.com](https://qpm.qualcomm.com/#/main/tools/details/Qualcomm_AI_Runtime_SDK?version=2.42.0.251225), download and run the installer for your OS.
- A **Snapdragon 8 Gen 2 / Gen 3 / Elite** device with USB debugging on.
- *Linux only*: install udev rules so the phone is visible to `adb`. Easiest: `sudo apt install android-sdk-platform-tools-common`, then add yourself to the `plugdev` group with `sudo usermod -aG plugdev $USER` and log out/in.

You don't need to install Gradle, set `JAVA_HOME` manually, or install a separate JDK 17 — the `build.cmd` / `build.sh` wrapper finds a JDK automatically, and Foojay provisions JDK 17 for Kotlin compilation if needed.

---

## Setup (once per machine)

**1. Tell Gradle where your QNN SDK is.** Add to `android/local.properties`:

```properties
# Windows
qnn.sdk.dir=C:/Qualcomm/AIStack/QAIRT/2.42.0.251225

# Linux
qnn.sdk.dir=/opt/qcom/aistack/qairt/2.42.0.251225
```

Forward slashes always — Java `.properties` files treat `\` as an escape character, so backslashes on Windows will silently corrupt the path. This file is gitignored.

**2. Place the bundled model assets.** From the [model release page](https://github.com/AlexNtFound/HitchPlay/releases/tag/model-qwen2_5_7b_instruct-v1), download these four files and drop them into the locations shown:

| File | Size | Destination |
|---|---|---|
| `tokenizer.json` | ~7 MB | `android/ChatApp/src/main/assets/models/qwen2_5_7b_instruct/` |
| `genie-config.json` | ~2 KB | `android/ChatApp/src/main/assets/models/qwen2_5_7b_instruct/` |
| `whisper-tiny-en.tflite` | ~41 MB | `android/ChatApp/src/main/assets/` |
| `whisper-tiny.tflite` | ~67 MB | `android/ChatApp/src/main/assets/` |

The first two are the Qwen LLM's runtime config + tokenizer. The two `.tflite` files are the Whisper speech-to-text models used for voice input.

**Don't download the `.bin` files** from that release — the 6 large model weight files are auto-downloaded by the app on first launch. Bundling them in the APK is impossible (Android's ZIP32 4 GB limit) and would slow every build to a crawl anyway.

If you skip a Whisper file the app will still build and the chat will work over text input, but voice input will crash at runtime.

**3. Make `adb` available** on your PATH.

*Windows* — either set the alias for the current PowerShell session only:

```powershell
Set-Alias adb "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe"
```

…or add it to your User PATH permanently (idempotent — re-running won't double-add; open a new PowerShell window after running for the change to take effect):

```powershell
$adb = "$env:LOCALAPPDATA\Android\Sdk\platform-tools"
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
if ($userPath -notlike "*$adb*") {
    [Environment]::SetEnvironmentVariable("Path", "$userPath;$adb", "User")
}
```

(Avoid `setx PATH` for this — it has a 1024-character truncation bug that can permanently lose entries from your existing PATH. `[Environment]::SetEnvironmentVariable` doesn't.)

*Linux* — append to `~/.bashrc` once:

```bash
echo 'export PATH="$HOME/Android/Sdk/platform-tools:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

If you installed `adb` via `apt`, it's already on PATH and you can skip this step.

---

## Build, install, run

There are two build variants. Both are built from the same source on the same `main` branch — debug vs release is purely *how the APK is packaged and signed*, not where the code lives.

### Debug vs release at a glance

| | Debug | Release |
|---|---|---|
| Command | `.\build.cmd assembleDebug` | `.\build.cmd assembleRelease` |
| Output APK | `ChatApp\build\outputs\apk\debug\ChatApp-debug.apk` | `ChatApp\build\outputs\apk\release\ChatApp-release.apk` |
| Signing | Auto-generated per-PC debug keystore (in `~/.android/debug.keystore`) | Project keystore from `ChatApp/ursa-release.keystore` — **not committed**, distributed via team password manager |
| Signature consistency across PCs | **Each PC signs differently** — installing a new APK from PC B over PC A's install fails with `INSTALL_FAILED_UPDATE_INCOMPATIBLE` | **Same signature everywhere** that has the keystore — updates Just Work |
| APK size | Larger (no minification, no resource shrinking) | Same size right now (minification is currently disabled — see note below) |
| `debuggable=true` | Yes (debugger can attach, logs verbose) | No |
| Build time | Slightly faster | Slightly slower |
| When to use | Day-to-day dev, quick testing on your own phone | Sharing with collaborators, demos, anything you'd upload to a release page |

> Minification note: the release build *could* shrink the APK by ~30 MB via R8 minification, but it's disabled today because this project uses reflection-heavy libraries (Hilt, Compose, kotlinx-serialization, OkHttp) that crash at runtime without curated proguard rules. The release APK still has all the real benefits (proper signing, no debuggable flag) — it just isn't smaller yet. Enabling minification is future work.

### Which one should you pick right now?

- **Working on the code yourself?** → Debug. Fast iteration.
- **About to hand the APK to someone else (a labmate, a teammate, anyone with a supported phone)?** → Release. They install it once, you (or anyone else with the keystore) can ship them updates later without forcing an uninstall.
- **Both?** → Build whichever you need right now; you can always build the other one later from the same code.

---

## Building a debug APK

From `android/`:

*Windows* (PowerShell):

```powershell
.\build.cmd assembleDebug
adb install -r ChatApp\build\outputs\apk\debug\ChatApp-debug.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

*Linux* (bash) — first time only, mark the wrappers executable: `chmod +x build.sh gradlew`:

```bash
./build.sh assembleDebug
adb install -r ChatApp/build/outputs/apk/debug/ChatApp-debug.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

`build.cmd` / `build.sh` is a thin wrapper around `gradlew` that auto-detects Java — checks `JAVA_HOME`, then versioned JDK install dirs (Adoptium, Corretto, Zulu, etc.), then Android Studio's bundled JBR, then `java` on PATH. If you'd rather call Gradle directly (because you've set `JAVA_HOME` yourself), `.\gradlew.bat assembleDebug` / `./gradlew assembleDebug` still works.

The first run does a few one-time things automatically: downloads Gradle 8.9, provisions JDK 17, configures CMake, copies QNN libs into the build. Expect the first build to take 5–10 minutes; subsequent builds are ~30 seconds.

If the build fails with a message starting `[Ursa]`, that's an intentional error — read it; it tells you exactly what to fix.

> If a debug APK is already installed on the phone and you rebuilt on a *different* PC, the install will fail with `INSTALL_FAILED_UPDATE_INCOMPATIBLE` because the two PCs sign with different auto-generated debug keystores. Either uninstall first (`adb uninstall com.quicinc.chatapp` — wipes downloaded model bins) or share `~/.android/debug.keystore` between your PCs (one-time copy).

---

## Building a release APK

A release APK is signed with a stable, team-controlled keystore. APKs built on different PCs sign with the same key, so updates install cleanly across machines without the `INSTALL_FAILED_UPDATE_INCOMPATIBLE` error.

> **Security note:** the keystore is intentionally **not** committed to this repo. If it were, anyone who cloned the public repo could build a malicious APK that Android would treat as a legitimate update to the real Ursa app — pushing arbitrary code to users' devices. The keystore lives only on the maintainers' machines and is distributed through a team password manager. `*.keystore` is in `.gitignore` as defense-in-depth.

Pick the path that fits your situation:

### Path A — You're a release maintainer (you have the team's keystore)

A team release manager will give you `ursa-release.keystore` plus the three passwords. Drop the keystore into `android/ChatApp/` (the `.gitignore` rule means git won't accidentally pick it up), then add the passwords to `android/local.properties`:

```properties
ursa.release.storePassword=<from-password-manager>
ursa.release.keyAlias=<from-password-manager>
ursa.release.keyPassword=<from-password-manager>
```

Then build:

*Windows*:
```powershell
.\build.cmd assembleRelease
adb install -r ChatApp\build\outputs\apk\release\ChatApp-release.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

*Linux*:
```bash
./build.sh assembleRelease
adb install -r ChatApp/build/outputs/apk/release/ChatApp-release.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

### Path B — You don't have the team keystore but want a release APK for your own testing

You can generate a personal keystore. APKs you build with it will be signed with **your** key — so they won't update over a teammate's installed app, but they'll still install and run on your own devices and are valid "release-flavor" APKs.

*Windows*:
```powershell
keytool -genkeypair -v -storetype PKCS12 `
  -keystore ChatApp\ursa-release.keystore `
  -keyalg RSA -keysize 2048 -validity 10000 `
  -alias ursa-release `
  -dname "CN=Ursa,OU=Ursa,O=Berkeley,L=Berkeley,ST=CA,C=US"
```

*Linux*:
```bash
keytool -genkeypair -v -storetype PKCS12 \
  -keystore ChatApp/ursa-release.keystore \
  -keyalg RSA -keysize 2048 -validity 10000 \
  -alias ursa-release \
  -dname "CN=Ursa,OU=Ursa,O=Berkeley,L=Berkeley,ST=CA,C=US"
```

`keytool` ships with the JDK so it's already on PATH if `java -version` works. It'll prompt you for a keystore password and a key password. Pick anything — write them down for the next step:

```properties
# android/local.properties
ursa.release.storePassword=<the-password-you-just-typed>
ursa.release.keyAlias=ursa-release
ursa.release.keyPassword=<the-password-you-just-typed>
```

Then build the same way as Path A.

### What happens if the keystore isn't configured at all

If `local.properties` doesn't have the credentials, or `ChatApp/ursa-release.keystore` doesn't exist, `assembleRelease` still **builds** — but produces an **unsigned** APK that can't be installed. That's by design: fresh clones can run `assembleRelease` to verify it compiles without forcing every developer to set up signing first. You'll get a non-fatal warning during the build. To actually install and distribute, follow Path A or B above.

### Initial team keystore creation (one-time, only the very first maintainer)

Done once, ever. The output goes into the team password manager — never the repo.

```powershell
keytool -genkeypair -v -storetype PKCS12 `
  -keystore ursa-release.keystore `
  -keyalg RSA -keysize 2048 -validity 10000 `
  -alias ursa-release `
  -dname "CN=Ursa,OU=Ursa,O=Berkeley,L=Berkeley,ST=CA,C=US"
```

Save the resulting `ursa-release.keystore` file, the keystore password, and the key password into a team-shared password manager (1Password, Bitwarden, etc.). Distribute to release maintainers via that channel — never via Slack/email/git.

---

## First launch — model auto-download

The first time you launch on a device that doesn't have the model yet, you'll see a **"Setting up the on-device model"** screen. The app downloads ~4.8 GB of `.bin` files from this repo's GitHub Releases over Wi-Fi (~3 minutes on fast home Wi-Fi). The phone needs to be on actual Wi-Fi — adb over USB does not give the phone internet.

Watch progress in another terminal (same command on Windows and Linux):

```
adb logcat ModelProvisioner:V *:S
```

After all 6 files download + verify, the chat UI loads. Subsequent launches skip this screen — the bins persist across reinstalls. They're only re-downloaded if you `adb uninstall`, clear app storage, or a new release changes the manifest.

---

## Android Studio (optional)

If you'd rather not use the terminal: open the `android/` folder in Android Studio, sync Gradle when prompted, hit Run. The wrapper config + Foojay JDK + Foojay-resolved toolchain make this Just Work without any Android Studio settings tweaking. Works on Windows and Linux equally.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `[Ursa] Could not find a JDK` | *Windows*: install Android Studio (bundled JBR is auto-detected) or OpenJDK 17 from [adoptium.net](https://adoptium.net). *Linux*: `sudo apt install openjdk-17-jdk` |
| `Error: JAVA_HOME is not set` (when calling `gradlew[.bat]` directly) | Use `.\build.cmd` / `./build.sh` instead — they auto-detect Java |
| `IllegalAccessError: superclass access check failed: ... com.sun.tools.javac` | Make sure your `gradle.properties` has the `--add-exports` block (already committed). If you edited it, re-pull |
| `Invalid Java installation found at ... .gradle/.tmp/jdks/...` | Foojay's JDK download corrupted. *Windows*: `Remove-Item -Recurse -Force "$env:USERPROFILE\.gradle\.tmp\jdks", "$env:USERPROFILE\.gradle\jdks"`. *Linux*: `rm -rf ~/.gradle/.tmp/jdks ~/.gradle/jdks`. Then rebuild |
| `[Ursa] QNN SDK path is not configured` | Add `qnn.sdk.dir=...` to `android/local.properties` (forward slashes!) |
| `[Ursa] Missing model assets for 'qwen2_5_7b_instruct'` | Place `tokenizer.json` and `genie-config.json` in `assets/models/qwen2_5_7b_instruct/` |
| `[Ursa] Stray model .bin files found` | Delete the `.bin` files from `assets/models/...` — bins are downloaded at runtime, not bundled |
| `Unknown Kotlin JVM target: 21` | Run `.\gradlew.bat clean` / `./gradlew clean` and rebuild — Foojay should fix it |
| App stuck on "Setting up the on-device model", "Unable to resolve host" | Phone has no Wi-Fi. Connect to Wi-Fi, then tap Retry |
| `Setup failed: Hash mismatch` | The release file isn't byte-identical to the manifest. Re-upload from the source-of-truth bin |
| `Unsupported device` toast | Device SoC isn't in the supported list. Add to `MainActivity.kt` and create the matching HTP config |
| App crashes immediately on launch (after bins download) | QNN SDK version mismatch. Confirm `qnn.sdk.dir` points at QAIRT **2.42.0** |
| *Linux*: `adb devices` shows phone as `no permissions` or doesn't list it | Install udev rules and add user to `plugdev`: `sudo usermod -aG plugdev $USER`, then log out/in and replug the phone |
| *Linux*: `./build.sh: Permission denied` | First-time setup step: `chmod +x build.sh gradlew` |

---

## Performance tuning

`genie-config.json` parameters (push to device with `adb push <file> /storage/emulated/0/Android/data/com.quicinc.chatapp/files/models/qwen2_5_7b_instruct/genie-config.json` — same command on Windows and Linux):

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

   *Windows* (PowerShell):
   ```powershell
   Get-ChildItem .\*.bin | ForEach-Object {
     "{0}  {1}  {2}" -f $_.Name, $_.Length, (Get-FileHash $_.FullName -Algorithm SHA256).Hash.ToLower()
   }
   ```

   *Linux* (bash):
   ```bash
   for f in *.bin; do
     printf "%s  %s  %s\n" "$f" "$(stat -c%s "$f")" "$(sha256sum "$f" | cut -d' ' -f1)"
   done
   ```

3. Create a GitHub Release at https://github.com/AlexNtFound/HitchPlay/releases/new with tag `model-<modelName>-v<n>`. Upload all `.bin` files (each must be ≤2 GB). **Publish, don't draft.**
4. Update `manifest.json` at `android/ChatApp/src/main/assets/models/<modelName>/manifest.json` with the new sizes, hashes, URLs, `releaseTag`, and a bumped `manifestVersion`.
5. Verify one URL resolves to the right size:

   *Windows* (PowerShell):
   ```powershell
   $u = "https://github.com/AlexNtFound/HitchPlay/releases/download/<tag>/<file>.bin"
   (Invoke-WebRequest -Uri $u -Method Head -MaximumRedirection 5 -UseBasicParsing).Headers["Content-Length"]
   ```

   *Linux* (bash):
   ```bash
   u="https://github.com/AlexNtFound/HitchPlay/releases/download/<tag>/<file>.bin"
   curl -sIL "$u" | grep -i '^content-length:' | tail -1
   ```

6. Build a new APK and ship. Existing devices detect the manifest change and re-download only the affected files.

---

## Maintainers: publishing a new app release

App releases live in a **separate, private repo** ([AlexNtFound/HitchPlay-releases](https://github.com/AlexNtFound/HitchPlay-releases)) so that the Qualcomm-licensed binaries inside each APK aren't accessible to anyone who hasn't accepted Qualcomm's EULA. The public code repo stays public; only the binary deliverables are restricted.

To publish a new version:

1. **Build the release APK** locally (Path A from above — uses the team keystore so signatures stay consistent across versions):
   ```powershell
   .\build.cmd assembleRelease
   ```
   Output: `ChatApp\build\outputs\apk\release\ChatApp-release.apk`

2. **Rename to a recognizable filename**:
   ```powershell
   Copy-Item ChatApp\build\outputs\apk\release\ChatApp-release.apk $env:USERPROFILE\Desktop\Ursa-v0.2.0.apk
   ```

3. **Publish the release** at https://github.com/AlexNtFound/HitchPlay-releases/releases/new:
   - Tag: `app-v<major>.<minor>.<patch>` (e.g. `app-v0.2.0`)
   - Title: `Ursa v0.2.0`
   - Description: brief changelog (what changed since the previous version, any known issues)
   - Drag-drop the renamed APK
   - Leave "Set as a pre-release" unchecked (unless it really is alpha-quality)
   - **Publish**, don't save as draft

4. **Invite any new collaborators** to the private repo so they can see the release: Settings → Collaborators → Add people.

5. **Notify the team** that v0.2.0 is out — e.g. Slack/email with the [latest release link](https://github.com/AlexNtFound/HitchPlay-releases/releases/latest).

Collaborators with the previous APK installed can update cleanly: `adb install -r Ursa-v0.2.0.apk`. The same signing keystore is used across all releases, so updates install over existing installs without wiping the downloaded model bins. First-time installers go through the ~3 min model auto-download on first launch.

> **Versioning:** semver. Patch (`v0.1.0` → `v0.1.1`) for bugfixes. Minor (`v0.1.0` → `v0.2.0`) for new features. Major (`v0.x.y` → `v1.0.0`) for breaking changes (e.g. a new rover protocol, dropping a SoC, swapping the LLM).

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
