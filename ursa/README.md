<div align="center">
  <img src="android/ChatApp/assets/ursa_logo.jpg" alt="Ursa Logo" width="80" height="80" />

  <h3>Ursa</h3>
  <p><strong>From Speech to Action: Voice Enabled Rover Control App using Large Language Model</strong></p>

  <table align="center">
    <tr>
      <td><img src="android/ChatApp/assets/eecs_logo.jpg" alt="Berkeley EECS Logo" width="160" /></td>
      <td><img src="android/ChatApp/assets/vive_logo.png" alt="Vive Center Logo" width="100" /></td>
      <td><img src="android/ChatApp/assets/qualcomm_logo.png" alt="Qualcomm Logo" width="140" /></td>
    </tr>
  </table>
</div>

<details>
<summary><strong>Table of Contents</strong></summary>

- [About the Project](#about-the-project)
- [Demo](#demo)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Technical Stack](#technical-stack)
- [Getting Started with the App](#getting-started-with-the-app)
- [Attribution](#attribution)
- [License](#license)

</details>

## About the Project

Ursa is an Android application that converts natural language user commands into machine code instructions for robotic control, leveraging an on-device large language model (LLM). This project currently employs **Qwen 2.5-7B-Instruct** (previously LLaMA 3.2-3B) and integrates Qualcomm’s [Chat App Demo](https://github.com/quic/ai-hub-apps/tree/main/android/ChatApp). Sponsored by **Qualcomm**.

## Demo

[![Watch the demo on YouTube](https://img.youtube.com/vi/QfCmIGPUlbI/0.jpg)](https://youtu.be/QfCmIGPUlbI)

## Key Features
- Natural language to ROS2 code translation using Qwen 2.5-7B-Instruct and Whisper-tiny.en
- On-device model inference with Qualcomm Genie runtime and QAI Hub binaries
- Support for both manual control and real-time voice input
- Real-time telemetry, video streaming, and occupancy map display
- Fully offline operation; secure, responsive, and mobile-optimized

## System Architecture

```plaintext
[ Android UI: Voice/Text Input ]
               ↓
[ Whisper Model (STT) ]
               ↓
[ Qwen 2.5-7B-Instruct Inference (Genie Runtime) ]
               ↓
[ ROS2 Code Generation ]
               ↓
[ Rover Communication Layer ]
```

## Technical Stack

<img src="android/ChatApp/assets/tech-stack.png" alt="Tech Stack" />

**Frontend**: Kotlin/Java (Android Studio)  
**Backend**:  
- Whisper-tiny.en (speech-to-text)  
- Qwen 2.5-7B-Instruct (natural language to code generation)  
- Qualcomm Genie runtime for inference

**Hardware**: Qualcomm Snapdragon 8 Elite / 8 Gen 3 / 8 Gen 2


## Getting Started with the App

See **[BUILD_GUIDE.md](BUILD_GUIDE.md)** for complete instructions covering:
- Prerequisites (JDK 17, QNN SDK)
- Building the APK (debug and release variants)
- First-launch auto-download of model weights
- Troubleshooting and performance tuning
- How to swap to a different LLM
- Publishing a new model release

### TL;DR for collaborators

If you just want to install the app on a supported phone (Snapdragon 8 Gen 2 / Gen 3 / Elite) **without setting up a build environment**, ask the team for a release APK and run:

```
adb install -r Ursa.apk
adb shell am start -n com.quicinc.chatapp/com.chatgptlite.wanted.MainActivity
```

The app downloads the 4.8 GB of model weights automatically over Wi-Fi on first launch (~3 minutes). After that it works fully offline.

If you want to build from source instead, follow [BUILD_GUIDE.md](BUILD_GUIDE.md).

## Attribution

Portions of the codebase and documentation are adapted from the Qualcomm Chat App Demo, 
including the Llama wrapper and Genie runtime integration guides. 
Modifications have been made to align with the project’s natural language to machine code 
conversion goals.

All original Qualcomm copyrights and license terms apply.

## LICENSE

This project includes licensed components from Qualcomm Technologies, Inc. Qualcomm® AI Hub Apps is licensed under BSD-3. See the [LICENSE file](../LICENSE).
