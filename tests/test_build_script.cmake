if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

file(READ "${SOURCE_DIR}/scripts/build_mediapipe_bridge.ps1" SCRIPT_TEXT)

set(REQUIRED_TEXT
    "$ResolvedOutput = Join-Path $ProjectRoot $OutputDirectory"
    "$OpenCvRoot = Join-Path $ResolvedOutput \"vcpkg_installed/x64-windows\""
    "mediapipe_opencv_repo_"
    "$ResolvedMediaPipeGitPath = $ResolvedMediaPipe.Replace('\\', '/')"
    "safe.directory=$ResolvedMediaPipeGitPath"
    "if ($LASTEXITCODE -ne 0 -or -not $ActualCommitOutput)")

foreach(TEXT IN LISTS REQUIRED_TEXT)
    string(FIND "${SCRIPT_TEXT}" "${TEXT}" MATCH_INDEX)
    if(MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "MediaPipe bridge build script is not output-directory aware: ${TEXT}")
    endif()
endforeach()

string(FIND "${SCRIPT_TEXT}" "build/windows-debug/vcpkg_installed/x64-windows" FIXED_DEBUG_PATH_INDEX)
if(NOT FIXED_DEBUG_PATH_INDEX EQUAL -1)
    message(FATAL_ERROR "MediaPipe bridge build script still hard-codes the debug OpenCV installation")
endif()
