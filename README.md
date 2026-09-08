# Gemini 2 空中虛擬鍵盤 MVP

Windows 原生 C++ 原型：Orbbec Gemini 2 RGB-D、MediaPipe Hand Landmarker、表面掃描加三點鍵盤幾何校正、桌面實地鍵盤投影與單次觸控事件。

## 已實作

- Orbbec SDK 2.9.3 同步 RGB／Depth 擷取，可由 YAML 選擇裝置支援的深度模式、深度精度、30／60 FPS 與 50／60 Hz 防閃爍。
- 優先使用硬體 Depth-to-Color；不支援時改用 `ob::Align(OB_STREAM_COLOR)` 軟體對齊。
- 裝置支援時依序套用 SDK Temporal、低強度 Spatial，再以 5×5 ROI 中位數與跳值拒絕穩定深度；Hole Filling 預設關閉。
- MediaPipe v0.10.35 原生 C++ Hand Landmarker DLL；固定官方模型及 SHA-256。
- 食指指尖 XY 使用 One Euro Filter；短暫失追只保留畫面位置，不具按鍵事件資格。
- 觸控距離不再直接讀指尖像素的深度。指尖落在手指輪廓邊緣，該處深度會把手指與桌面混在同一個 ROI；改為沿「指尖→食指 DIP 關節」向後取兩個探測點，再把深度梯度外推回指尖。決定按哪一鍵的仍是指尖像素，只有距離來自探測點。
- 校正前會從指尖周邊深度掃描目標表面；環形取樣捨棄比指尖再近超過 `finger_clearance_mm` 的樣本，避免打在手指上的像素污染平面擬合。手指從指尖往後會抬離桌面，因此手指像素比指尖更靠近相機，而桌面像素與指尖同深或更遠——指尖貼著桌面掃描時也成立。
- 三個校正位置以像素射線與表面交點收集 18 筆樣本，避免指尖高度差混入鍵盤幾何。
- 鍵盤總尺寸由三點校正自動計算，按鍵為連續無間隙的 3×4 格；YAML 只保留 2 mm 邊界遲滯及 touch/release 門檻。
- 校正完成後，10 個按鍵會用與觸控數學同一組相機模型反投影回影片，直接畫在實體桌面上，讓虛擬鍵盤的位置肉眼可見、可核對。
- 靠近方向、單次觸發、離開後 re-arm，以及短失追重建速度基線、逾時取消的狀態機。
- 接近速度閘門採 latch：同一段接近過程中只要曾達到門檻即可觸發。人指向目標時會自然減速，若只在跨越門檻的那一幀取瞬時速度，正常速度的按壓會被判定為不合格而靜默漏按。latch 的有效範圍嚴格限定在那一段接近：手指回退超過 3 mm、進入觸控門檻卻沒有觸發（例如當下不在任何鍵上）、追蹤中斷或修改參數，都會作廢既有的接近證據，避免橫向滑進某個鍵時用舊的下降動作誤觸。
- 另有 dwell 觸發：停留在觸控門檻內達 `dwell_ms` 即送出按鍵，補足過慢而無法通過速度閘門的按壓；設 0 可關閉。
- OpenCV 顯示實際深度模式／精度／FPS／D2C、raw／filtered 距離、接近／停留狀態、校正散布與最近事件。
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

- `C`：開始校正並準備表面掃描。
- `S`：開啟或關閉參數設定視窗。
- `Space`：先開始表面掃描；完成後依序取樣 3 個鍵盤邊界點（1 鍵左上角、3 鍵右上角、0 鍵正下方），讓程式自動取得鍵盤尺寸。
- `Enter`：解算平面與鍵盤幾何。
- `R`：重設校正與觸控狀態。
- `Q`／`Esc`：離開。

事件只顯示於 OpenCV UI 並輸出 stdout，不注入 Windows 鍵盤。

按 `S` 可調整相機深度工作模式、深度精度、FPS、深度取樣、觸控門檻、追蹤逾時、邊界遲滯與校正點最小距離。鍵盤尺寸不在設定視窗中編輯，而是在每次 `C` 校正時先掃描實體平面，再由 3 個實體邊界點自動計算：外框確定後直接除以 3×4，按鍵彼此相連沒有間隙。深度工作模式、精度與 FPS 都由目前連線的 Gemini 2 實際能力清單建立；YAML 若原本填了不在清單內的值，設定視窗不會把該值重新加入下拉選單。互動參數會立即更新；三個相機參數會寫回 YAML，重新啟動程式後套用到 Gemini 2。其餘 One Euro、深度歷史、校正取樣、邊界遲滯，以及 `touch.dwell_ms`、`depth.finger_clearance_mm`、`fingertip.depth_probe_near_ratio`／`depth_probe_far_ratio` 等進階欄位仍保留於 YAML；目前校正幾何只在本次執行期間有效。

若需要確認 SDK 是否可存取相機，可執行：

```powershell
.\orbbec_stream_probe.exe
```

它會列出 SDK 偵測到的感測器與串流設定，並以主程式相同的硬體對齊設定測試 RGB-D 影像。

## 測試

`ctest` 包含：

- One Euro 靜止收斂與快速移動反應。
- 深度 ROI median、單幀跳值、新深度重新鎖定與 invalid depth 歷史清除。
- 平面、UV 投影、normal、鍵位 mapping，以及平面 UV 與相機像素之間的來回轉換必須互為反函數（桌面鍵盤投影的正確性前提）。
- 三點校正推得的連續 3×4 格線，並確認鍵與鍵之間不存在會讓按壓落空的死區。
- 指尖深度探測的梯度外推、指向相機時的退化保護與異常梯度夾制。
- 表面掃描環形取樣會排除比指尖更近的樣本。
- 校正中位數／MAD 離群排除與 2 mm 按鍵邊界遲滯。
- 單次觸發、release re-arm、短暫 tracking loss 不產生幽靈事件且不取消既有 armed 狀態。
- 減速接近仍能觸發（速度閘門 latch 的回歸測試）、慢速按壓改由 dwell 觸發、滑到別的鍵會重新計時、停留不會自動連發。
- latch 不得超出它所屬的那一段接近：橫向滑進按鍵不算按壓、回退後的慢速爬行仍受最低接近速度限制、改設定會作廢舊證據。
- 新舊 YAML 設定載入、安全相機設定 fallback 及 RGB-D frame 完整性。
- 真實 MediaPipe DLL／模型 smoke test：黑圖安全回傳無手；官方固定影像回傳 21 landmarks。

## 尚待實機驗收

Gemini 2 實機已驗證可啟動硬體對齊的 RGB-D 串流，深度與彩色影像皆約 30 FPS；主程式也已持續運作超過 8 秒。仍需完成互動層的實機驗收：

- RGB、Depth、D2C 與指尖 XYZ 正常。
- 即時畫面至少 15 FPS。
- 靜止平面距離抖動至少降低 30%，連續靜止 5 分鐘不得誤觸。
- 快速點擊不漏觸，觸碰延遲不可比原版本增加超過 30 ms。
- 三點校正後完成「靠近一次只輸出一次、停留不重複、離開後可再按」。
- 投影到桌面的鍵盤方框與實體標記重合；若明顯偏移，代表對齊後的深度內參與影像尺寸不符（程式會在 HUD 與 stderr 提出相機警告）。
