# Gemini 2 空中虛擬鍵盤 MVP

Windows 原生 C++ 原型：Orbbec Gemini 2 RGB-D、MediaPipe Hand Landmarker、七點平面與鍵盤幾何校正、數字鍵盤與單次觸控事件。

## 已實作

- Orbbec SDK 2.9.3 同步 RGB／Depth 擷取，可由 YAML 選擇裝置支援的深度模式、深度精度、30／60 FPS 與 50／60 Hz 防閃爍。
- 優先使用硬體 Depth-to-Color；不支援時改用 `ob::Align(OB_STREAM_COLOR)` 軟體對齊。
- 裝置支援時依序套用 SDK Temporal、低強度 Spatial，再以 5×5 ROI 中位數與跳值拒絕穩定深度；Hole Filling 預設關閉。
- MediaPipe v0.10.35 原生 C++ Hand Landmarker DLL；固定官方模型及 SHA-256。
- 食指指尖 XY 使用 One Euro Filter；短暫失追只保留畫面位置，不具按鍵事件資格。
- 七個校正位置各收集 18 筆有效 XYZ，以中位數與 MAD 排除離群值並顯示散布。
- 鍵盤總尺寸、按鍵尺寸與間距由七點校正自動計算；YAML 只保留 2 mm 邊界遲滯及 touch/release 門檻。
- 靠近方向、單次觸發、離開後 re-arm，以及短失追重建速度基線、逾時取消的狀態機。
- OpenCV 顯示實際深度模式／精度／FPS／D2C、raw／filtered 距離、校正散布與最近事件。
- `S` 開啟原生參數設定視窗，可調整主要互動參數、相機深度工作模式／精度／FPS、即時預覽距離並套用保存；三個相機欄位只列出目前 Gemini 2 由 SDK 回報可用的選項，不再接受任意模式名稱或顯示裝置不支援的精度／FPS。其餘相機與濾波進階項目先以 YAML 設定。

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
- `Space`：依序取樣 7 個鍵盤邊界點，讓程式自動取得平面與鍵盤尺寸。
- `Enter`：解算平面與鍵盤幾何。
- `R`：重設校正與觸控狀態。
- `Q`／`Esc`：離開。

事件只顯示於 OpenCV UI 並輸出 stdout，不注入 Windows 鍵盤。

按 `S` 可調整相機深度工作模式、深度精度、FPS、深度取樣、觸控門檻、追蹤逾時、邊界遲滯與校正點最小距離。鍵盤尺寸與間距不在設定視窗中編輯，而是在每次 `C` 校正時由 7 個實體邊界點自動計算。深度工作模式、精度與 FPS 都由目前連線的 Gemini 2 實際能力清單建立；YAML 若原本填了不在清單內的值，設定視窗不會把該值重新加入下拉選單。互動參數會立即更新；三個相機參數會寫回 YAML，重新啟動程式後套用到 Gemini 2。其餘 One Euro、深度歷史、校正取樣與邊界遲滯等進階欄位仍保留於 YAML；目前校正幾何只在本次執行期間有效。

若需要確認 SDK 是否可存取相機，可執行：

```powershell
.\orbbec_stream_probe.exe
```

它會列出 SDK 偵測到的感測器與串流設定，並以主程式相同的硬體對齊設定測試 RGB-D 影像。

## 測試

`ctest` 包含：

- One Euro 靜止收斂與快速移動反應。
- 深度 ROI median、單幀跳值、新深度重新鎖定與 invalid depth 歷史清除。
- 平面、UV 投影、normal、鍵位 mapping。
- 校正中位數／MAD 離群排除與 2 mm 按鍵邊界遲滯。
- 單次觸發、release re-arm、短暫 tracking loss 不產生幽靈事件且不取消既有 armed 狀態。
- 新舊 YAML 設定載入、安全相機設定 fallback 及 RGB-D frame 完整性。
- 真實 MediaPipe DLL／模型 smoke test：黑圖安全回傳無手；官方固定影像回傳 21 landmarks。

## 尚待實機驗收

Gemini 2 實機已驗證可啟動硬體對齊的 RGB-D 串流，深度與彩色影像皆約 30 FPS；主程式也已持續運作超過 8 秒。仍需完成互動層的實機驗收：

- RGB、Depth、D2C 與指尖 XYZ 正常。
- 即時畫面至少 15 FPS。
- 靜止平面距離抖動至少降低 30%，連續靜止 5 分鐘不得誤觸。
- 快速點擊不漏觸，觸碰延遲不可比原版本增加超過 30 ms。
- 七點校正後完成「靠近一次只輸出一次、停留不重複、離開後可再按」。
