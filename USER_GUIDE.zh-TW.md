# Gemini 2 空中虛擬鍵盤操作指南

本指南適合第一次使用本 MVP 原型的使用者，說明如何從啟動程式開始，完成三點校正並操作空中數字鍵盤。

> 目前按鍵事件只會顯示在 OpenCV 視窗與 PowerShell 終端機，不會替 Windows 或其他程式實際輸入數字。

## 一、啟動前準備

請先確認：

1. 電腦是 Windows 11，且已連接 Orbbec Gemini 2。
2. 相機前方有足夠空間讓食指伸出、移動與靠近平面。
3. 建置輸出目錄保留完整 runtime，不要只複製 `aerial_touch_app.exe`。
4. 沒有其他程式正在獨占 Gemini 2 的影像串流。

`build/windows-release` 至少應包含下列項目：

```text
aerial_touch_app.exe
OrbbecSDK.dll
OrbbecSDKConfig.xml
extensions/frameprocessor/ob_frame_processor.dll
extensions/                 （整個目錄）
mediapipe_hand_bridge.dll
assets/models/hand_landmarker.task
config/default.yaml
```

其中 `extensions/frameprocessor/ob_frame_processor.dll` 是 Gemini 2 深度串流所需的 runtime。缺少它時，可能仍看得到部分相機資訊，但無法正常取得深度資料。

如果尚未建置，請在專案根目錄的 Visual Studio Developer PowerShell 執行：

```powershell
cmake --preset windows-release
cmake --build --preset windows-release
ctest --preset windows-release --output-on-failure
```

若尚未準備 MediaPipe bridge，先取得固定版本原始碼、下載模型，再建置 bridge：

```powershell
git clone --depth 1 --branch v0.10.35 https://github.com/google-ai-edge/mediapipe.git third_party/mediapipe
.\scripts\fetch_hand_model.ps1
.\scripts\build_mediapipe_bridge.ps1 -MediaPipeSource .\third_party\mediapipe
```

完成後確認 `build/windows-release/mediapipe_hand_bridge.dll` 存在，再啟動主程式。

## 二、啟動程式

在 PowerShell 執行：

```powershell
Set-Location build/windows-release
.\aerial_touch_app.exe
```

必須從 `build/windows-release` 啟動，因為程式預設會用相對路徑尋找設定檔、MediaPipe bridge 與手部模型。

成功啟動後會出現 `Gemini 2 空中鍵盤` 視窗，畫面包含：

- Gemini 2 的 RGB 影像。
- 手部骨架；食指指尖會以較大的紅色標記顯示。
- 視窗右下角的 1–9、0 數字鍵盤。
- FPS、追蹤、指尖座標、校正、觸控與最近事件等資訊。

若要從專案根目錄啟動，可使用輸出目錄的明確相對路徑：

```powershell
.\build\windows-release\aerial_touch_app.exe `
  --config .\build\windows-release\config\default.yaml `
  --bridge .\build\windows-release\mediapipe_hand_bridge.dll `
  --model .\build\windows-release\assets\models\hand_landmarker.task
```

## 三、先確認畫面狀態

| 畫面項目 | 意義 |
| --- | --- |
| `FPS` | 目前影像更新速度。 |
| `對齊` | 顯示硬體 D2C 或軟體 D2C；兩者都是 RGB 與深度的對齊方式。 |
| `追蹤：偵測到手部` | MediaPipe 已偵測到手部。 |
| `追蹤：未偵測到手部` | 目前沒有可靠的手部追蹤結果。 |
| `追蹤：無法使用` | MediaPipe bridge 或手部模型未成功載入。 |
| `指尖像素` | 指尖在影像中的像素位置。 |
| `XYZ` | 指尖在相機座標中的三維位置，單位是 mm。 |
| `平面 UV` | 指尖投影到校正平面後的位置，單位是 mm。 |
| `距離` | 指尖與虛擬平面的帶正負距離，單位是 mm。 |
| `按鍵` | 指尖目前落在哪個數字鍵；不在鍵內時顯示 `-`。 |
| `觸控：可觸發` | 已準備好接受下一次觸控。 |
| `觸控：等待手指離開` | 仍視為手指停留在按鍵附近，需先離開才能再次觸發。 |
| `校正：尚未設定` | 尚未完成三點校正。 |
| `校正：進行中` | 正在擷取校正點。 |
| `校正：完成` | 已建立虛擬鍵盤平面，可以操作。 |
| `最近按鍵` | 最近一次產生的按鍵事件。 |

## 四、開啟參數設定視窗

在主程式的 OpenCV 視窗按 `S`，會開啟「參數設定」視窗。視窗分成觸控判定、深度與校正、觸控距離預覽及鍵盤幾何四區；每個參數都有滑桿與可精確輸入的數值框，右側預覽會顯示目前指尖距離與觸控狀態。

可調整的項目包含：

- 觸控門檻、離開門檻、最低接近速度與追蹤逾時。
- 深度取樣半徑與校正點最小距離。
- 按鍵寬度、高度、水平間距與垂直間距。

輸入值不合法時，欄位會顯示錯誤並停用套用；離開門檻必須大於觸控門檻。按「恢復預設值」只會載入表單，仍須按「套用並儲存」才會寫入設定檔。按「取消」或關閉視窗會捨棄尚未套用的修改。

按「套用並儲存」後，設定會立即套用到目前執行中的程式並寫回啟動時使用的 YAML。現有校正平面與觸控狀態會保留；校正點最小距離會在下一次按 `C` 重新校正時生效。

## 五、完成三點校正

校正用來設定虛擬鍵盤的起點、右方方向與下方方向。每次重新啟動程式，或相機／操作平面位置改變後，都應重新校正。

### 1. 開始校正

1. 將手放在預定的空中操作平面附近。
2. 確認畫面能看到手部骨架與紅色食指指尖標記。
3. 按鍵盤上的 `C`。

畫面會提示將手指移到 `1 鍵左上角`。這裡指的是空中虛擬鍵盤的幾何位置，不是 OpenCV 視窗或電腦螢幕的左上角。

### 2. 依序擷取三個位置

面向相機，想像空中有一個 3 欄 × 4 列的數字鍵盤：

```text
第 1 點：1 鍵左上角 ───── 第 2 點：往右，朝 3 鍵方向
       |
       |
第 3 點：往下，朝 0 鍵方向
```

依下列順序操作；每一點都先讓食指穩定，再按一次 `Space`：

1. 將指尖放在預定的 `1` 鍵左上角，按 `Space`。
2. 將指尖移到起點右方、朝 `3` 鍵方向的位置，按 `Space`。
3. 將指尖移到起點下方、朝 `0` 鍵方向的位置，按 `Space`。

第 2、3 點不必精確落在鍵面中心，但應與第 1 點保持明顯距離；預設至少約 80 mm，且右方與下方兩個方向不可接近同一直線。第三點記錄後，狀態會提示按 `Enter` 完成校正。

如果畫面顯示 `無法擷取：指尖深度資料無效`，先讓指尖保持在相機可取得深度的範圍內，穩定後再按一次 `Space`。未成功擷取時不會增加校正點數。

### 3. 解算平面

確認畫面顯示已記錄 3/3 後按 `Enter`：

- 成功時顯示 `校正完成`，校正狀態變成完成，即可操作。
- 失敗時顯示 `校正失敗：三個位置距離太近或幾乎位於同一直線`。按 `R` 重設後，重新擷取距離更大的三個位置。

## 六、操作虛擬數字鍵盤

校正完成後：

1. 將食指移到視窗右下角對應的數字鍵上方。
2. 確認畫面上的 `按鍵` 顯示目標數字。
3. 從平面上方朝該鍵靠近，直到穿過觸控門檻。
4. 在視窗的 `最近按鍵` 或 PowerShell 輸出中確認事件。
5. 要觸發下一個鍵時，先把食指移離平面，再移向下一個鍵。

每次靠近只觸發一次。停留在同一個鍵上不會連續重複觸發；必須先離開，`觸控` 回到 `可觸發` 後，下一次靠近才會再次產生事件。

成功事件會在終端機輸出類似：

```text
按鍵事件 按鍵=5 時間戳記毫秒=... 指尖XYZ毫米=(...) 平面UV毫米=(...)
```

這只是事件紀錄，不會把數字自動輸入記事本、瀏覽器或其他 Windows 程式。

## 七、操作按鍵總覽

| 按鍵 | 功能 |
| --- | --- |
| `C` | 清除目前校正，開始重新指定三個校正點。 |
| `S` | 開啟或關閉參數設定視窗。 |
| `Space` | 在校正進行中擷取目前指尖位置。其他時機按下不會擷取校正點。 |
| `Enter` | 在校正進行中，以三個已擷取點解算平面。 |
| `R` | 重設校正與觸控狀態，回到等待 `C` 的狀態。 |
| `Q` 或 `Esc` | 離開程式。 |

結束前建議先移開手指，再按 `Q` 或 `Esc`。程式會停止相機串流並關閉影像視窗。

## 八、常見問題

### 啟動後立即結束，或顯示相機啟動失敗

確認 Gemini 2 已連接、沒有被其他程式使用，並從 `build/windows-release` 啟動。再確認下列檔案存在：

```text
OrbbecSDK.dll
OrbbecSDKConfig.xml
extensions/frameprocessor/ob_frame_processor.dll
```

若要單獨確認 SDK 是否能存取相機，可在 `build/windows-release` 執行：

```powershell
.\orbbec_stream_probe.exe
```

它會列出偵測到的感測器與串流設定，並以主程式相同的硬體對齊設定測試 RGB-D 影像。若出現 `Device changed! removed`，先檢查 USB 線、接頭、Hub 與供電；相機傾斜或移動時不要拉扯 USB 線。

### 顯示 `追蹤：無法使用`

確認下列檔案存在於啟動目錄，且檔案版本互相匹配：

```text
mediapipe_hand_bridge.dll
opencv_core4.dll
opencv_imgproc4.dll
z.dll
assets/models/hand_landmarker.task
```

若模型不存在，執行 `scripts/fetch_hand_model.ps1`；若 bridge 不存在，依本指南第一節重新執行 MediaPipe bridge 建置命令。

### 顯示 `追蹤：未偵測到手部`

把整隻手移到相機畫面內，確保光線充足，並讓食指伸直、不要被其他物體遮住。只有偵測到完整且有效的手部 landmarks，程式才會顯示指尖資料。

### 看得到指尖，但無法擷取校正點

影像能看到手部不代表深度資料一定有效。請讓指尖不要太靠近或太遠離相機，避開強反光或極暗表面，保持穩定後再按 `Space`。

### 校正失敗

按 `R` 後重新校正。第 2 點應明確位於第 1 點右方，第 3 點應明確位於第 1 點下方；三點要分開足夠距離，且不可幾乎排成一直線。

### 校正完成但沒有按鍵事件

確認：

1. `校正` 顯示完成。
2. `按鍵` 顯示目標數字，而不是 `-`。
3. 指尖是從平面上方靠近，而不是一開始就停在平面附近。
4. 若 `觸控` 顯示 `等待手指離開`，先把指尖移遠，等它回到 `可觸發`。

## 九、目前版本限制

- 只支援單手追蹤。
- 目前只有數字鍵盤。
- 按鍵事件只輸出到視窗與終端機，不注入 Windows 系統鍵盤事件。
- 不包含 ASKA3D 或雙手操作。
- 實機互動效果仍會受相機位置、USB 穩定性、光線、手部距離與校正品質影響。
