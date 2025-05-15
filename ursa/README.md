<div align="center">
  <img src="apps/android/ChatApp/assets/ursa_logo.jpg" alt="Ursa Logo" width="80" height="80" />

  <h3>Ursa</h3>
  <p><strong>From Speech to Action: Voice Enabled Rover Control App using Large Language Model</strong></p>

  <table align="center">
    <tr>
      <td><img src="apps/android/ChatApp/assets/eecs_logo.jpg" alt="Berkeley EECS Logo" width="160" /></td>
      <td><img src="apps/android/ChatApp/assets/vive_logo.png" alt="Vive Center Logo" width="100" /></td>
      <td><img src="apps/android/ChatApp/assets/qualcomm_logo.png" alt="Qualcomm Logo" width="140" /></td>
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

Ursa is an Android application that converts natural language user commands into machine code instructions for robotic control, leveraging an on-device large language model (LLM). This project employs the **LlaMA 3.2-3B** model and integrates Qualcomm’s [Chat App Demo](https://github.com/quic/ai-hub-apps/tree/main/apps/android/ChatApp). Sponsored by **Qualcomm**.

## Demo

[![Watch the demo on YouTube](https://img.youtube.com/vi/QfCmIGPUlbI/0.jpg)](https://youtu.be/QfCmIGPUlbI)

## Key Features
- Natural language to ROS2 code translation using LLaMA 3.2-3B and Whisper-tiny.en
- On-device model inference with Qualcomm Genie runtime and AI Hub binaries
- Support for both manual control and real-time voice input
- Real-time telemetry, video streaming, and occupancy map display
- Fully offline operation; secure, responsive, and mobile-optimized

## System Architecture

```plaintext
[ Android UI: Voice/Text Input ]
               ↓
[ Whisper Model (STT) ]
               ↓
[ LLaMA 3.2-3B Inference (Genie Runtime) ]
               ↓
[ ROS2 Code Generation ]
               ↓
[ Rover Communication Layer ]
```

## Technical Stack

<img src="apps/android/ChatApp/assets/tech-stack.png" alt="Tech Stack" />

**Frontend**: Kotlin/Java (Android Studio)  
**Backend**:  
- Whisper-tiny.en (speech-to-text)  
- LLaMA 3.2-3B (natural language to code generation)  
- Qualcomm Genie runtime for inference

**Hardware**: Qualcomm Snapdragon 8 Gen 3 / Snapdragon X Elite


## Getting Started with the App

Navigate to the source directory: [apps/android/ChatApp](apps/android/ChatApp).

Follow the README there for build instructions, dependencies, and model setup.

## Attribution

Portions of the codebase and documentation are adapted from the Qualcomm Chat App Demo, 
including the Llama wrapper and Genie runtime integration guides. 
Modifications have been made to align with the project’s natural language to machine code 
conversion goals.

All original Qualcomm copyrights and license terms apply.

## LICENSE

This project includes licensed components from Qualcomm Technologies, Inc. Qualcomm® AI Hub Apps is licensed under BSD-3. See the [LICENSE file](../LICENSE).
