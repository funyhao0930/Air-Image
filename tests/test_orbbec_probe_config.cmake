if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

file(READ "${SOURCE_DIR}/tools/orbbec_stream_probe.cpp" PROBE_SOURCE)

foreach(REQUIRED_TEXT IN ITEMS
        "pipeline->enableFrameSync();"
        "pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE)"
        "config->setAlignMode(ALIGN_D2C_HW_MODE);"
        "SetConsoleOutputCP(CP_UTF8)")
    string(FIND "${PROBE_SOURCE}" "${REQUIRED_TEXT}" MATCH_INDEX)
    if(MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "探測工具未使用相容設定：${REQUIRED_TEXT}")
    endif()
endforeach()

foreach(FORBIDDEN_TEXT IN ITEMS
        "profile_pipeline"
        "OB_FORMAT_Y16"
        "try_start(\"僅深度\""
        "try_start(\"僅彩色\"")
    string(FIND "${PROBE_SOURCE}" "${FORBIDDEN_TEXT}" MATCH_INDEX)
    if(NOT MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "探測工具仍含不相容設定：${FORBIDDEN_TEXT}")
    endif()
endforeach()
