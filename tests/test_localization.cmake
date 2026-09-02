if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(USER_FACING_FILES
    "${SOURCE_DIR}/src/main.cpp"
    "${SOURCE_DIR}/src/app_config.cpp"
    "${SOURCE_DIR}/src/hand_tracker.cpp"
    "${SOURCE_DIR}/src/orbbec_camera.cpp"
    "${SOURCE_DIR}/src/settings_window.cpp"
    "${SOURCE_DIR}/tools/orbbec_stream_probe.cpp"
    "${SOURCE_DIR}/mediapipe_bridge/bridge.cc")

set(SOURCE_TEXT "")
foreach(FILE_PATH IN LISTS USER_FACING_FILES)
    file(READ "${FILE_PATH}" FILE_TEXT)
    string(APPEND SOURCE_TEXT "\n${FILE_TEXT}")
endforeach()

set(REQUIRED_ZH_TW_TEXT
    "Gemini 2 空中鍵盤"
    "將手指移到 1 鍵左上角"
    "請移到 3 鍵右上角"
    "請移到 0 鍵正下方"
    "校正完成"
    "相機啟動失敗"
    "深度工作模式"
    "深度精度"
    "相機 FPS"
    "重新啟動後生效"
    "裝置數量："
    "探測失敗：")

foreach(REQUIRED_TEXT IN LISTS REQUIRED_ZH_TW_TEXT)
    string(FIND "${SOURCE_TEXT}" "${REQUIRED_TEXT}" MATCH_INDEX)
    if(MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "缺少繁體中文介面文字：${REQUIRED_TEXT}")
    endif()
endforeach()

set(FORBIDDEN_ENGLISH_TEXT
    "Press C to set the keypad area"
    "\"Fingertip pixel: "
    "\"Tracker: "
    "\"Touch: "
    "\"Calibration: "
    "\"Status: "
    "C calibrate"
    "Point at keypad start"
    "Cannot capture:"
    "Captured "
    "Capture start, right and down"
    "Calibration ready"
    "Calibration rejected:"
    "Reset complete"
    "Fatal error:"
    "Missing configuration value:"
    "Configuration contains invalid"
    "MediaPipe hand bridge DLL could not be loaded"
    "Orbbec start failed:"
    "profiles for "
    ": started; depth="
    ": FAILED: "
    "devices: "
    "device: "
    "sensors: "
    "probe FAILED: "
    "Model path is empty"
    "Unknown exception while creating Hand Landmarker"
    "Invalid detect_rgb argument"
    "Unknown exception during Hand Landmarker detection")

foreach(FORBIDDEN_TEXT IN LISTS FORBIDDEN_ENGLISH_TEXT)
    string(FIND "${SOURCE_TEXT}" "${FORBIDDEN_TEXT}" MATCH_INDEX)
    if(NOT MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "仍有英文介面文字：${FORBIDDEN_TEXT}")
    endif()
endforeach()
