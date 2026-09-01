# Gemini 2 空中虛擬鍵盤 MVP

Windows 原生 C++ 原型：Orbbec Gemini 2 RGB-D、MediaPipe Hand Landmarker、三點平面校正、數字鍵盤與單次觸控事件。

## 已實作

- Orbbec SDK 2.9.3 同步 RGB／Depth 擷取。
- 優先使用硬體 Depth-to-Color；不支援時改用 `ob::Align(OB_STREAM_COLOR)` 軟體對齊。
- 5×5 ROI 有效深度中位數與 Orbbec 官方 2D-to-3D 座標轉換。
- MediaPipe v0.10.35 原生 C++ Hand Landmarker DLL；固定官方模型及 SHA-256。
- O／U／V 三點校正、近距離與近共線拒絕、朝向相機的平面 normal。
- YAML 定義 30 mm 數字鍵、5 mm 間距及 touch/release 門檻。
- 靠近方向、單次觸發、離開後 re-arm、追蹤遺失取消的狀態機。
- OpenCV 骨架、XYZ、Plane UV、距離、狀態、FPS、鍵位與最近事件顯示。
- `S` 開啟原生參數設定視窗，可調整完整 YAML 設定、即時預覽距離並套用保存。

ASKA3D、雙手及系統按鍵注入不在本 MVP 範圍。

## 環境

- Windows 11
- Visual Studio 2026 Community 或相容的 MSVC Build Tools
- CMake、Ninja
- Bazelisk
- MSYS2（MediaPipe Bazel genrule 需要 `bash` 與 `awk`）
- Orbbec SDK 2.9.3：`C:\Program Files\OrbbecSDK 2.9.3`

OpenCV 與 yaml-cpp 由 `vcpkg.json` manifest 管理。

## 建置主程式

在 Visual Studio Developer PowerShell 或 Developer Command Prompt 中：

```powershell
cmake --preset windows-release
cmake --build --preset windows-release
ctest --preset windows-release --output-on-failure
```

輸出位於：

```text
build/windows-release/aerial_touch_app.exe
```

每次建置會將 `OrbbecSDK.dll`、`OrbbecSDKConfig.xml` 及 `extensions/` 一併部署到輸出目錄。`extensions/frameprocessor/ob_frame_processor.dll` 是 Gemini 2 depth stream 的必要 runtime；請從 `build/windows-release` 執行，不要只複製單一 EXE。

## 建置 MediaPipe bridge

先取得固定版本原始碼：

```powershell
git clone --depth 1 --branch v0.10.35 https://github.com/google-ai-edge/mediapipe.git third_party/mediapipe
```

下載並驗證模型：

```powershell
.\scripts\fetch_hand_model.ps1
```

建置 bridge：

```powershell
.\scripts\build_mediapipe_bridge.ps1 -MediaPipeSource .\third_party\mediapipe
```

腳本會驗證 MediaPipe commit、套用鎖定於 v0.10.35 的 VS 2026 相容補丁，使用短 Bazel output root 避開 Windows 路徑長度限制，並複製下列 runtime 到 `build/windows-release`：

- `mediapipe_hand_bridge.dll`
- `opencv_core4.dll`
- `opencv_imgproc4.dll`
- `z.dll`

版本與雜湊記錄於 `mediapipe_bridge/VERSION.txt`。

## 執行

從輸出目錄啟動，讓預設相對路徑可找到設定、模型及 DLL：

```powershell
Set-Location build/windows-release
.\aerial_touch_app.exe
```

也可明確指定路徑：

```powershell
.\aerial_touch_app.exe --config config/default.yaml --bridge mediapipe_hand_bridge.dll --model assets/models/hand_landmarker.task
```

操作鍵：

- `C`：開始校正。
- `S`：開啟或關閉參數設定視窗。
- `Space`：依序擷取 O（左上）、U（右側）、V（下側）。
- `Enter`：解算平面。
- `R`：重設校正與觸控狀態。
- `Q`／`Esc`：離開。

事件只顯示於 OpenCV UI 並輸出 stdout，不注入 Windows 鍵盤。

按 `S` 可調整深度取樣、觸控門檻、追蹤逾時、鍵盤尺寸／間距與校正點最小距離。設定視窗按「套用並儲存」後會立即更新目前執行中的參數，並寫回目前 `--config` 指定的 YAML；目前校正平面不會被清除，校正距離會在下一次校正時使用。

若需要確認 SDK 是否可存取相機，可執行：

```powershell
.\orbbec_stream_probe.exe
```

它會列出 SDK 偵測到的感測器與串流設定，並以主程式相同的硬體對齊設定測試 RGB-D 影像。

## 測試

`ctest` 包含：

- 深度 ROI median 與 invalid depth。
- 平面、UV 投影、normal、鍵位 mapping。
- 單次觸發、停留不重複、release re-arm、tracking loss。
- YAML 設定載入及 RGB-D frame 完整性。
- 真實 MediaPipe DLL／模型 smoke test：黑圖安全回傳無手；官方固定影像回傳 21 landmarks。

## 尚待實機驗收

Gemini 2 實機已驗證可啟動硬體對齊的 RGB-D 串流，深度與彩色影像皆約 30 FPS；主程式也已持續運作超過 8 秒。仍需完成互動層的實機驗收：

- RGB、Depth、D2C 與指尖 XYZ 正常。
- 即時畫面至少 15 FPS。
- 觸控事件端到端延遲不超過 150 ms。
- 三點校正後完成「靠近一次只輸出一次、停留不重複、離開後可再按」。
