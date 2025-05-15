# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html.
# For more examples on how to use CMake, see https://github.com/android/ndk-samples.

# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name. The project name can be accessed via ${ PROJECT_NAME},
# Since this is the top level CMakeLists.txt, the project name is also accessible
# with ${CMAKE_PROJECT_NAME} (both CMake variables are in-sync within the top level
# build script scope).
project("chatapp")

set(GENIE_HEADERS_PATH "${QNN_SDK_ROOT_PATH}/include/Genie")
set(GENIE_LIB_PATH "${QNN_SDK_ROOT_PATH}/lib/aarch64-android/libGenie.so")

include_directories("includes")

# Include Genie headers here
include_directories(${GENIE_HEADERS_PATH})

add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this CMakeLists.txt.
        PromptHandler.cpp
        GenieWrapper.cpp
        GenieLib.cpp)

# Link libGenie.so here
target_link_libraries(${CMAKE_PROJECT_NAME}
        "${GENIE_LIB_PATH}"
        android
        log)

# Define your project and its version
cmake_minimum_required(VERSION 3.0)
project(TFLiteEngine VERSION 1.0)

# Specify where to find the header files for TF Lite C++
set(INCLUDE_DIRS
        ${CMAKE_CURRENT_LIST_DIR}/tf-lite-api/tensorflow_src
        ${CMAKE_CURRENT_LIST_DIR}/tf-lite-api/include/flatbuffers/include
)

# # Check if the build is Android
if (ANDROID)
    add_library(audioEngine SHARED TFLiteEngine.cpp TFLiteEngineJNI.cpp)
    target_include_directories(audioEngine PRIVATE ${INCLUDE_DIRS})

    # Add 'tflite' library (imported)
    add_library(tflite SHARED IMPORTED)
    set_target_properties(tflite PROPERTIES IMPORTED_LOCATION
            ${CMAKE_CURRENT_LIST_DIR}/tf-lite-api/generated-libs/${ANDROID_ABI}/libtensorflowlite.so)

    target_link_libraries(audioEngine tflite)
endif ()

# Check if the build is not Android
if (NOT ANDROID)
    add_library(audioEngine SHARED TFLiteEngine.cpp)
    target_include_directories(audioEngine PRIVATE ${INCLUDE_DIRS})

    # Add 'tflite' library (imported)
    add_library(tflite SHARED IMPORTED)
    set_target_properties(tflite PROPERTIES IMPORTED_LOCATION
            ${CMAKE_CURRENT_LIST_DIR}/tf-lite-api/generated-libs/x86_64/libtensorflowlite.so)

    # Create an executable target that links with 'audioEngine' and 'tflite' libraries
    add_executable(my_audio_app main.cpp)
    target_link_libraries(my_audio_app PRIVATE audioEngine tflite)
endif ()






