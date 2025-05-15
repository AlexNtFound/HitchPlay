// ---------------------------------------------------------------------
// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
// ---------------------------------------------------------------------
#include "PromptHandler.hpp"
#include "GenieWrapper.hpp"

using namespace AppUtils;

// Llama3 prompt
constexpr const std::string_view c_bot_name = "QBot";
constexpr const std::string_view c_first_prompt_prefix_part_1 =
    "<|begin_of_text|><|start_header_id|>system<|end_header_id|>\n\nYour name is ";
constexpr const std::string_view c_first_prompt_prefix_part_2 = R"(and you are an assistant controlling a rover.
The user provides natural language instructions, and you respond with exactly one hardcoded command based on the instruction.
If the user provides completely irrelavant instructions, which is not used for controlling the rover, output: "Sorry, I don't understand."
Here are the commands you can generate:
- Move forward for 1 meter: ros2 run drive_pkg drive_publisher --ros-args -p x:=1
- Move backward for 2 meters: ros2 run drive_pkg drive_publisher --ros-args -p x:=-2
- Turn left for 90 degrees: ros2 run drive_pkg drive_publisher --ros-args -p y:=90
- Turn right for 30 degrees: ros2 run drive_pkg drive_publisher --ros-args -p y:=-30
- Turn left for 90 degrees and move forward for 2 meters: ros2 run drive_pkg drive_publisher --ros-args -p x:=2 -p y:=90
- Return to base: ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\" --once


Examples:
User input: "Hi rover, move forward by 1 meter." Output: ros2 run drive_pkg drive_publisher --ros-args -p x:=1
User input: "turn left for 90 degrees and move forward by 2 meters." Output: ros2 run drive_pkg drive_publisher --ros-args -p x:=2 -p y:=90
User input: "Hi rover, move backward by 2 meter." Output: ros2 run drive_pkg drive_publisher --ros-args -p x:=-2
User input: "Please turn right for 30 degrees, rover." Output:  ros2 run drive_pkg drive_publisher --ros-args -p y:=-30
User input: "Can you return to base?" Output: ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\" --once
User input: "How are you?" Output: Sorry, I don't understand.


Instruction: Based on the user's input, select the most appropriate command and return it verbatim. Respond with only the command, no additional text or explanation. <|eot_id|>)";
constexpr const std::string_view c_prompt_prefix = "<|start_header_id|>user<|end_header_id|>\n\n";
constexpr const std::string_view c_end_of_prompt = "<|eot_id|>";
constexpr const std::string_view c_assistant_header = "<|start_header_id|>assistant<|end_header_id|>\n\n";

PromptHandler::PromptHandler()
    : m_is_first_prompt(true)
{
}

std::string PromptHandler::GetPromptWithTag(const std::string& user_prompt)
{
    // Ref: https://www.llama.com/docs/model-cards-and-prompt-formats/meta-llama-3/
    if (m_is_first_prompt)
    {
        m_is_first_prompt = false;
        return std::string(c_first_prompt_prefix_part_1) + c_bot_name.data() + c_first_prompt_prefix_part_2.data() +
               c_prompt_prefix.data() + user_prompt + c_end_of_prompt.data() + c_assistant_header.data();
    }
    return std::string(c_prompt_prefix) + user_prompt.data() + c_end_of_prompt.data() + c_assistant_header.data();
}
