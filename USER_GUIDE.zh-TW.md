# Gemini 2 空中虛擬鍵盤：保母級操作指南

這份指南是寫給第一次使用本專案、第一次操作 Orbbec Gemini 2，甚至不熟悉 PowerShell 的使用者。請按照順序做，不要跳步；每個步驟都會告訴你「應該看到什麼」。

> 這是 MVP 原型。程式目前只會在 OpenCV 視窗和 PowerShell 顯示按鍵事件，不會把數字真的輸入到記事本、瀏覽器或其他 Windows 程式。

## 先看懂：你最後要完成什麼

完整流程只有四件事：

1. 讓電腦看見 Gemini 2。
2. 從正確的輸出資料夾啟動程式。
3. 用 `C` 和七個校正點告訴程式「虛擬鍵盤在哪裡、大小是多少」。
4. 用食指靠近平面，讓程式產生按鍵事件。

第一次使用時，請把「校正」當成必要步驟。程式重新啟動後，七點校正結果不會保留，必須重新校正。

## 第一部分：啟動前先準備好

### 1. 硬體擺放

請先做以下準備：

1. 將 Orbbec Gemini 2 接上電腦。
2. 優先使用主機板直連的 USB 3.x 連接埠，不要先接 USB Hub。
3. 將相機固定好，避免操作時碰到相機或拉扯 USB 線。
4. 相機前方留出手指可以伸出、移動、靠近平面的空間。
5. 保持光線穩定；避免指尖反光、過暗，或被其他物體遮住。
6. 如果有開啟 Orbbec Viewer 或其他會使用相機的程式，先關閉它們。

如果相機在移動或傾斜時出現 `Device changed! removed`，先停下來檢查 USB 線、接頭、Hub 和供電，不要先把問題當成程式碼錯誤。

### 2. 你需要的軟體

建置程式需要以下環境：

| 軟體 | 用途 |
| --- | --- |
| Windows 11 | 執行平台。 |
| Visual Studio 2026 Community 或相容的 MSVC Build Tools | 編譯 C++ 程式。 |
| CMake、Ninja | 設定與建置專案。 |
| Bazelisk | 建置 MediaPipe bridge。 |
| MSYS2 | 提供 MediaPipe 建置所需的 `bash` 與 `awk`。 |
| Orbbec SDK 2.9.3 | 連接 Gemini 2；預設位置是 `C:\Program Files\OrbbecSDK 2.9.3`。 |

OpenCV 和 yaml-cpp 會由本專案的 vcpkg manifest 管理。

### 3. 你要在哪裡輸入指令

本指南假設專案位於：

```text
D:\Codex Project\Air Image
```

如果你的專案放在別的資料夾，以下指令中的路徑請換成你的實際位置。

最簡單的做法是：在檔案總管打開專案資料夾，按住 `Shift` 後在空白處按滑鼠右鍵，選擇「在終端機中開啟」。

也可以開啟 Visual Studio 的 **Developer PowerShell for VS 2026**，再執行：

```powershell
Set-Location -LiteralPath "D:\Codex Project\Air Image"
```

執行後，提示字元最後應該會顯示類似：

```text
PS D:\Codex Project\Air Image>
```

如果你看到的是其他資料夾，不要急著執行建置指令；先重新執行上面的 `Set-Location`。

## 第二部分：如果你已經有建置好的程式

如果 `build\windows-release` 已經存在，而且裡面有完整 runtime，可以直接跳到[第四部分：啟動程式](#第四部分啟動程式)。

### 先檢查輸出檔案

在專案根目錄執行：

```powershell
$required = @(
  ".\build\windows-release\aerial_touch_app.exe",
  ".\build\windows-release\OrbbecSDK.dll",
  ".\build\windows-release\OrbbecSDKConfig.xml",
  ".\build\windows-release\extensions\frameprocessor\ob_frame_processor.dll",
  ".\build\windows-release\mediapipe_hand_bridge.dll",
  ".\build\windows-release\assets\models\hand_landmarker.task",
  ".\build\windows-release\config\default.yaml"
)
$required | ForEach-Object {
  [pscustomobject]@{ Exists = Test-Path -LiteralPath $_; Path = $_ }
} | Format-Table -AutoSize
```

每一列的 `Exists` 都應該是 `True`。如果 `mediapipe_hand_bridge.dll` 是 `False`，請不要直接啟動；請按照下一部分重新建置 bridge。

## 第三部分：第一次建置程式

如果你只是要啟動既有輸出，可以略過本部分。如果這是第一次使用，請完整做完本部分。

### 1. 確認工具能被找到

在專案根目錄逐行執行：

```powershell
cmake --version
ninja --version
bazelisk --version
```

每一行都應該顯示版本號。如果顯示「不是內部或外部命令」，先安裝對應工具，或改用 Visual Studio Developer PowerShell，讓 Visual Studio 的編譯工具加入目前環境。

### 2. 先建立 CMake 輸出與 vcpkg 相依套件

```powershell
cmake --preset windows-release
```

成功時不應出現 `CMake Error`，並會建立：

```text
build\windows-release
```

這一步先做，不要跳過；MediaPipe bridge 建置需要這一步產生的 OpenCV 檔案。

### 3. 取得固定版本的 MediaPipe

如果 `third_party\mediapipe` 尚未存在，執行：

```powershell
git clone --depth 1 --branch v0.10.35 https://github.com/google-ai-edge/mediapipe.git third_party/mediapipe
```

如果資料夾已經存在，不要再次 `git clone`；直接進行下一步。

### 4. 下載並驗證手部模型

```powershell
.\scripts\fetch_hand_model.ps1
```

成功時會顯示已驗證的 `hand_landmarker.task` 路徑。這個腳本會同時檢查 SHA-256；如果出現 checksum mismatch，先保留錯誤訊息，不要自行下載其他模型替代。

### 5. 建置 MediaPipe bridge

```powershell
.\scripts\build_mediapipe_bridge.ps1 -MediaPipeSource .\third_party\mediapipe
```

這一步可能比一般 CMake 建置久。腳本會檢查 MediaPipe v0.10.35、套用 Windows 相容補丁，最後把 bridge 和必要 DLL 複製到 `build\windows-release`。

成功後，至少應看到：

```text
build\windows-release\mediapipe_hand_bridge.dll
build\windows-release\opencv_core4.dll
build\windows-release\opencv_imgproc4.dll
build\windows-release\z.dll
```

### 6. 編譯主程式

```powershell
cmake --build --preset windows-release
```

成功後應有：

```text
build\windows-release\aerial_touch_app.exe
build\windows-release\orbbec_stream_probe.exe
```

CMake 也會把 Orbbec SDK 的 DLL、設定檔和完整 `extensions` 目錄部署到輸出資料夾。請不要只把 EXE 單獨複製到另一個資料夾。

### 7. 執行測試

```powershell
ctest --preset windows-release --output-on-failure
```

測試成功時會顯示所有測試通過。若 PowerShell 找不到 `ctest`，請改用 Visual Studio Developer PowerShell，或確認 CMake 已加入 PATH；不要把「沒有執行測試」當成「測試通過」。

## 第四部分：啟動程式

### 建議方式：從輸出資料夾啟動

在專案根目錄執行：

```powershell
Set-Location -LiteralPath ".\build\windows-release"
.\aerial_touch_app.exe
```

這是最不容易出錯的方式。程式預設會從目前資料夾尋找：

```text
config\default.yaml
mediapipe_hand_bridge.dll
assets\models\hand_landmarker.task
```

### 從專案根目錄直接啟動

如果你不想切換資料夾，可以使用完整相對路徑：

```powershell
.\build\windows-release\aerial_touch_app.exe `
  --config .\build\windows-release\config\default.yaml `
  --bridge .\build\windows-release\mediapipe_hand_bridge.dll `
  --model .\build\windows-release\assets\models\hand_landmarker.task
```

### 啟動成功時你應該看到什麼

1. PowerShell 不會立刻回到提示字元，而是持續輸出程式訊息。
2. 出現名為 `Gemini 2 空中鍵盤` 的 OpenCV 視窗。
3. 視窗中有 Gemini 2 的彩色影像。
4. 視窗底部或右下角顯示狀態文字與數字鍵盤區域。
5. PowerShell 會列出目前的對齊方式、深度模式、深度精度與 FPS。

第一次啟動時沒有看到手是正常的；先把手放進畫面，下一部分會說明如何判斷是否真的正常。

> 不建議直接雙擊 EXE。若程式立即結束，雙擊視窗可能一閃就消失，你會看不到錯誤原因。請從 PowerShell 啟動。

## 第五部分：啟動後先確認相機和手部追蹤

### 1. 確認手部位置

把整隻手放進相機畫面，讓食指伸直。不要只伸出一小截手指，也不要讓食指被手掌、桌面或其他物體遮住。

### 2. 看主視窗左上方的狀態

目前主視窗會顯示下列資訊：

| 顯示內容 | 代表什麼 | 第一次使用時怎麼做 |
| --- | --- | --- |
| `主程式 FPS` | 主程式畫面更新速度。 | 持續有數字即可；實機驗收目標至少 15 FPS。 |
| `相機：... FPS ...` | 相機實際使用的 FPS、深度模式和深度精度。 | 先記下來，通常不用修改。 |
| `對齊：硬體 D2C` 或 `軟體 D2C` | RGB 和深度影像如何對齊。 | 兩者都可以工作。 |
| `追蹤：本幀有效` | 這一幀的指尖位置可用於校正和觸控。 | 看到它再開始校正。 |
| `追蹤：未取得本幀觀測` | 目前沒有可確認的手部觀測。 | 調整手的位置、光線和食指姿勢。 |
| `追蹤：無法使用` | MediaPipe bridge 或模型沒有載入成功。 | 先看[故障排除](#第十部分故障排除)。 |
| `指尖像素（濾波）` | One Euro Filter 後的指尖像素位置；只有本幀有效時才可操作。 | 看到穩定標記即可。 |
| `XYZ` | 指尖三維座標，單位 mm。 | 校正時要有這一行。 |
| `平面 UV` | 指尖投影到校正平面的位置。 | 完成校正後才會出現。 |
| `距離` | 濾波後與原始的平面距離，單位 mm。 | 觸控判定使用濾波值。 |

如果你看到手部骨架、食指指尖標記，以及會更新的 `XYZ`，表示可以進入校正。

## 第六部分：`S` 參數設定視窗

第一次使用建議先保留預設值。只有在已完成一次正常校正、知道自己要調整什麼時，才使用 `S`。

### 開啟和關閉

1. 確認焦點在 `Gemini 2 空中鍵盤` 主視窗。
2. 按 `S`。
3. 會出現「參數設定」視窗；再按一次 `S` 可隱藏它。

### 視窗裡可以調整什麼

| 設定 | 單位 | 白話意思 |
| --- | --- | --- |
| 觸控門檻 | mm | 指尖靠近到這個距離以內，才可能觸發。 |
| 離開門檻 | mm | 指尖離開到這個距離以外，才重新允許下一次觸發。 |
| 最低接近速度 | mm/s | 指尖必須以足夠的速度靠近，避免慢慢漂移造成誤觸。 |
| 追蹤逾時 | ms | 手部追蹤消失多久後，程式才清除暫存狀態。 |
| 深度取樣半徑 | px | 取得指尖深度時使用的影像鄰近範圍。 |
| 校正點最小距離 | mm | 前三個平面基準點至少要分開多遠。 |
| 深度工作模式 | 下拉選單 | Gemini 2 實際支援的深度模式。 |
| 深度精度 | 下拉選單 | Gemini 2 實際支援的深度精度。 |
| FPS | 下拉選單 | RGB 和深度能配對使用的 FPS。 |

相機的三個下拉選單只會列出 Gemini 2 目前回報支援的選項，不要自行輸入不存在的模式名稱。

### 修改設定的正確順序

1. 用滑桿調整，或在數值框輸入數字。
2. 確認沒有紅色錯誤提示。
3. 確認離開門檻大於觸控門檻。
4. 按「套用並儲存」。
5. 若只想放棄修改，按「取消」或關閉視窗。

「恢復預設值」只是把預設值載入表單，還不會寫入設定檔；要真的保存，仍要按「套用並儲存」。

觸控、深度取樣等互動設定會立即套用；深度模式、深度精度和 FPS 會顯示「重新啟動後生效」，必須關閉程式再重新啟動。校正點和鍵盤尺寸不會在這裡輸入，而是由七點校正自動計算。

### 預覽區怎麼看

設定視窗右側的「觸控距離預覽」會顯示：

- 目前距離是否有效。
- 目前是否偵測到手。
- 指尖目前對應的按鍵。
- `可觸發` 或 `等待手指離開`。
- 觸控門檻和離開門檻。

第一次操作不需要調整進階 YAML 設定。RGB 防閃爍、SDK 濾波器、One Euro 濾波、深度跳值、校正樣本數等進階欄位仍在 `config\default.yaml` 管理；如果你不熟悉這些名詞，先不要修改。

## 第七部分：七點校正——最重要的操作

### 先理解七點是什麼

這七個點不是要按七次鍵，也不是點數字的中心；它們是用食指指向「虛擬鍵盤的邊界角點」，讓程式量出平面方向、鍵盤大小、按鍵大小和按鍵間距。

請想像你在相機前方放了一張有 3 欄、4 列數字鍵的紙：

```text
P1 ●────────────────────────● P2
    ┌─────┐ ┌─────┐ ┌─────┐
    │  1  │ │  2  │ │  3  │
    └─────┘ └─────┘ └─────┘
    ┌─────┐ ┌─────┐ ┌─────┐
    │  4  │ │  5  │ │  6  │
    └─────┘ └─────┘ └─────┘
    ┌─────┐ ┌─────┐ ┌─────┐
    │  7  │ │  8  │ │  9  │
    └─────┘ └─────┘ └─────┘
              ┌─────┐
              │  0  │
              └─────┘
                    ● P3
```

實際操作時，P3 要放在 `0` 鍵正下方的整張鍵盤下邊界中央，不是紙張右下角，也不是 `0` 鍵右下角。

### 七個點的固定順序

請記住這句話：

> `1 左上 → 3 右上 → 0 正下方 → 1 右上 → 2 右上 → 1 左下 → 4 左下`

| 點 | 要指的位置 | 這個點用來量什麼 |
| --- | --- | --- |
| P1 | `1` 鍵左上角 | 鍵盤原點。 |
| P2 | `3` 鍵右上角 | 整張鍵盤的右邊界。 |
| P3 | `0` 鍵正下方的鍵盤下邊界中央 | 整張鍵盤的下邊界；不是右下角。 |
| P4 | `1` 鍵右上角 | 一個按鍵的寬度。 |
| P5 | `2` 鍵右上角 | 水平按鍵間距。 |
| P6 | `1` 鍵左下角 | 一個按鍵的高度。 |
| P7 | `4` 鍵左下角 | 垂直按鍵間距。 |

所有點都要指向邊界或角點，不要指向數字字形中心。P1、P2、P3 要在同一個實際平面上，而且前三點要有明顯距離；預設 P2、P3 距離 P1 至少約 80 mm。

### 開始校正

1. 先確認畫面顯示 `追蹤：本幀有效` 和 `XYZ`。
2. 將食指放到準備好的 P1，也就是 `1` 鍵左上角。
3. 按 `C`。
4. 不要移動手指，先閱讀畫面上的 `狀態` 提示。

正常時會看到類似：

```text
將手指移到 1 鍵左上角，保持不動後按空白鍵開始取樣
```

如果你還沒有把手指放到 P1，現在才移過去；不要在錯誤位置直接按空白鍵。

### 每一個點都用同一個小流程

對 P1 到 P7，每個點都重複以下五步：

1. 把食指尖端放到指定邊界角點。
2. 讓手指完全停住，不要邊移動邊按鍵。
3. 按一次 `Space`，開始收集這個位置的樣本。
4. 看到 `校正點取樣：.../18，請保持不動` 後，繼續保持不動。
5. 等畫面顯示 `已記錄` 和下一個位置後，才移到下一點。

預設每個點需要 18 筆有效 XYZ。如果你曾在 YAML 修改樣本數，畫面上的實際數字可能不同；以畫面顯示為準。參數設定視窗目前不能修改樣本數。

取樣期間不要再按 `Space`，也不要按 `Enter`。取樣完成後，程式會自動記錄該點並顯示散布資訊。

### 按照順序完成七個點

現在照著下面的口令做，一次只做一點：

#### P1：1 鍵左上角

把指尖放在 `1` 鍵左上角，按 `Space`，保持不動直到畫面顯示 `已記錄 1 鍵左上角`。

#### P2：3 鍵右上角

把指尖水平移到 `3` 鍵右上角，按 `Space`，保持不動直到畫面顯示 `已記錄 3 鍵右上角`。

#### P3：0 鍵正下方

把指尖移到 `0` 鍵正下方的整張鍵盤下邊界中央，按 `Space`。

這一步最容易做錯：P3 不是鍵盤右下角，也不是 `0` 鍵右下角。成功後，畫面會顯示平面基準已建立，並要求前往 P4。

#### P4：1 鍵右上角

把指尖放到 `1` 鍵右上角，按 `Space`，保持不動直到記錄完成。

#### P5：2 鍵右上角

把指尖放到 `2` 鍵右上角，按 `Space`，保持不動直到記錄完成。

#### P6：1 鍵左下角

把指尖放到 `1` 鍵左下角，按 `Space`，保持不動直到記錄完成。

#### P7：4 鍵左下角

把指尖放到 `4` 鍵左下角，按 `Space`，保持不動直到記錄完成。

### 完成七點校正

P7 完成後，畫面會提示按 `Enter`。確認：

1. `校正：進行中 7/7`。
2. 沒有顯示「校正點正在取樣」。
3. 手指目前仍在 P7 附近即可；解算時不需要再按 `Space`。

然後按 `Enter`。

成功時，狀態會顯示類似：

```text
校正完成：鍵盤 ... × ... mm；按鍵 ... × ... mm；水平間距 ... mm；垂直間距 ... mm
```

此時：

- `校正` 會變成完成。
- 視窗右下角會出現可用的數字鍵盤。
- 程式已根據七點自動計算鍵盤總寬高、單鍵寬高和水平／垂直間距。

如果按 `Enter` 後失敗，程式會清除七個點，請按畫面提示從 P1 重新開始；不要拿著前一次的點繼續猜。

## 第八部分：實際按數字鍵

### 第一次測試只按一個鍵

校正完成後，先不要快速連按。只測試一個鍵，例如 `5`：

1. 將食指移到視窗右下角的 `5` 鍵附近。
2. 觀察畫面的 `按鍵` 是否顯示 `5`。
3. 如果顯示 `-`，代表指尖不在鍵的有效範圍內，先調整左右或上下位置。
4. 確認 `按鍵：5` 後，把手指移到虛擬平面上方。
5. 以清楚、穩定的動作朝平面靠近。
6. 看到 `最近按鍵：5`，或在 PowerShell 看到按鍵事件，就代表第一次測試成功。

### 為什麼不能一直停在鍵上

程式是「靠近觸發」而不是「停留連發」：

1. 第一次從平面上方靠近，產生一次事件。
2. 手指停在同一個鍵上，不會重複產生事件。
3. 要再按一次，必須先把手指移遠，直到 `觸控：可觸發`。
4. 再次靠近平面，才會產生下一次事件。

預設還要求有最低接近速度；如果你非常慢地把手指漂向平面，可能只會看到位置變化而沒有事件。請用自然、明確的點擊動作。

### 顏色與畫面提示

- 數字鍵顯示黃色或橘黃色：指尖目前懸停在該鍵。
- 數字鍵顯示綠色：剛剛產生了按鍵事件。
- `觸控：可觸發`：可以進行下一次靠近。
- `觸控：等待手指離開`：先移開手指，不要繼續往平面壓。

PowerShell 的事件內容會類似：

```text
按鍵事件 按鍵=5 時間戳記毫秒=... 指尖XYZ毫米=(...) 平面UV毫米=(...) 原始距離毫米=... 濾波距離毫米=...
```

## 第九部分：所有操作按鍵

請先點一下主 OpenCV 視窗，確保它取得鍵盤焦點，再按以下按鍵：

| 按鍵 | 功能 | 什麼時候使用 |
| --- | --- | --- |
| `C` | 清除目前校正，開始七點校正。 | 相機、紙張或虛擬鍵盤位置改變時。 |
| `S` | 開啟／隱藏「參數設定」視窗。 | 調整參數或查看距離預覽時。 |
| `Space` | 開始目前校正點的連續取樣。 | 只在校正進行中使用。 |
| `Enter` | 用七個已記錄校正點計算平面和鍵盤尺寸。 | 七點都顯示已記錄後使用。 |
| `R` | 清除校正、鍵盤幾何、觸控和濾波暫存狀態。 | 校正做錯、卡住或想完整重來時。 |
| `Q` 或 `Esc` | 離開程式。 | 結束使用時。 |

`Space` 或 `Enter` 在非校正狀態按下不會完成任何數字輸入；本程式沒有系統按鍵注入功能。

## 第十部分：故障排除——照症狀處理

### A. 程式一啟動就關閉

請不要雙擊 EXE。從 PowerShell 重新啟動，讀取最後幾行錯誤。

依序確認：

1. 目前位於 `build\windows-release`。
2. `aerial_touch_app.exe` 存在。
3. `OrbbecSDK.dll` 和 `OrbbecSDKConfig.xml` 存在。
4. `extensions\frameprocessor\ob_frame_processor.dll` 存在。
5. Gemini 2 已接上，且沒有被其他程式使用。

這個 frame processor 是深度串流需要的 runtime；不能只複製 EXE。

### B. 顯示「相機啟動失敗」

先做硬體排除：

1. 拔除再重新插入 Gemini 2。
2. 換到主機板直連的 USB 3.x 連接埠。
3. 拿掉 USB Hub 和過長／鬆動的延長線。
4. 關閉 Orbbec Viewer 或其他相機程式。
5. 重新從 `build\windows-release` 啟動。

如果仍失敗，用相機探測工具確認 SDK：

```powershell
Set-Location -LiteralPath "D:\Codex Project\Air Image\build\windows-release"
.\orbbec_stream_probe.exe
```

它會列出感測器、串流設定，並測試 depth-only、color-only 和 RGB-D。若探測工具也失敗，優先處理 USB、SDK runtime 或裝置連線，不要先重做校正。

### C. 顯示「追蹤：無法使用」

確認目前啟動資料夾有：

```text
mediapipe_hand_bridge.dll
opencv_core4.dll
opencv_imgproc4.dll
z.dll
assets\models\hand_landmarker.task
```

缺少模型時執行：

```powershell
Set-Location -LiteralPath "D:\Codex Project\Air Image"
.\scripts\fetch_hand_model.ps1
```

缺少 bridge 時重新執行：

```powershell
.\scripts\build_mediapipe_bridge.ps1 -MediaPipeSource .\third_party\mediapipe
```

### D. 顯示「追蹤：未取得本幀觀測」

這通常是暫時沒有可靠的手部觀測，不一定是故障。請：

1. 把整隻手放進影像內。
2. 讓食指伸直。
3. 增加光線，但不要讓指尖強烈反光。
4. 讓手指不要被手掌或物體遮住。
5. 停住約一秒，再看是否回到 `本幀有效`。

短暫失追只保留畫面顯示，不會拿暫存位置產生按鍵事件，這是正常的安全行為。

### E. 看到指尖，但沒有 XYZ 或無法開始取樣

影像看得到指尖，不代表深度一定有效。請：

1. 把指尖移到相機能取得深度的距離。
2. 避免純黑、透明、鏡面或強反光表面。
3. 保持指尖不動。
4. 等 `XYZ` 出現後再按 `Space`。

如果畫面顯示「等待有效指尖深度後開始收集」，不要連續按鍵；保持位置，等深度恢復即可。

### F. 校正取樣中斷或進度不動

取樣期間請不要移動手指，也不要再次按 `Space`。如果顯示：

- `校正取樣中斷：相機影像暫時無效`：檢查 USB 和相機串流，恢復後按 `Space` 重試。
- `校正取樣中斷：追蹤逾時`：重新把整隻手放回畫面，等 `本幀有效` 後按 `Space`。
- `校正點散布過大或有效樣本不足`：把手指放穩，避免手腕晃動，再按 `Space`。

### G. 七點完成但按 Enter 校正失敗

按 `R`，從 P1 重新做，不要只重做最後一點。重做時逐項檢查：

1. P1 是 `1` 鍵左上角。
2. P2 是 `3` 鍵右上角。
3. P3 是 `0` 鍵正下方中央，不是右下角。
4. P4、P5 是 `1`、`2` 鍵的右上角。
5. P6、P7 是 `1`、`4` 鍵的左下角。
6. 七點都在同一個平面上。
7. 每個點取樣時都保持不動。

### H. 校正完成，但 `按鍵` 顯示 `-`

這表示指尖的平面投影沒有落在任何數字鍵內。請先移動手指的左右／上下位置，直到 `按鍵` 顯示目標數字；不要先往下壓。

### I. `按鍵` 正確，但沒有事件

依序確認：

1. `校正` 顯示完成。
2. `追蹤` 顯示本幀有效。
3. `按鍵` 顯示目標數字，不是 `-`。
4. `觸控` 顯示 `可觸發`。
5. 先把手指移到平面上方，再以明確動作靠近。
6. 如果設定了最低接近速度，不要慢慢漂移靠近。

### J. 第一個鍵成功，第二個鍵沒有事件

這通常是還沒有完成 re-arm。把手指移遠，直到 `觸控：可觸發`，再移到下一個鍵並重新靠近。停留在第一個鍵上不會連續觸發。

### K. 設定已儲存，但相機模式沒有立刻改變

這是預期行為。觸控和深度取樣參數可立即套用；深度工作模式、精度和 FPS 必須：

1. 在設定視窗按「套用並儲存」。
2. 關閉主程式。
3. 從 `build\windows-release` 重新啟動。
4. 查看 PowerShell 和主視窗顯示的相機實際設定。

## 第十一部分：正常結束與重新開始

### 結束程式

1. 先把食指移離虛擬平面。
2. 點一下主 OpenCV 視窗。
3. 按 `Q` 或 `Esc`。
4. 等 PowerShell 回到提示字元。

程式會停止相機串流並關閉 OpenCV 視窗。

### 下次重新啟動

每次重新啟動都做以下流程：

```powershell
Set-Location -LiteralPath "D:\Codex Project\Air Image\build\windows-release"
.\aerial_touch_app.exe
```

啟動後重新執行：

```text
C → P1 Space → P2 Space → P3 Space → P4 Space → P5 Space → P6 Space → P7 Space → Enter
```

其中每次按 `Space` 後都必須等該點顯示 `已記錄`，不能快速連按。

## 第十二部分：目前版本的限制

- 只支援單手追蹤。
- 目前只有 1–9、0 數字鍵盤。
- 按鍵事件只顯示在 OpenCV 視窗和終端機，不會注入 Windows 系統鍵盤。
- 不包含 ASKA3D 或雙手操作。
- 校正幾何只保留在本次執行期間，重新啟動後要再做七點校正。
- 實機效果會受 USB 穩定性、相機位置、光線、手部距離和校正品質影響。

## 最短成功檢查表

如果你只想確認自己有沒有做對，最後應該全部符合：

```text
[ ] PowerShell 位於 build\windows-release
[ ] aerial_touch_app.exe 可以啟動
[ ] OpenCV 視窗顯示 Gemini 2 影像
[ ] 追蹤顯示本幀有效，且有 XYZ
[ ] C 後依序完成 P1～P7
[ ] 校正顯示完成
[ ] 按鍵顯示目標數字
[ ] 觸控顯示可觸發
[ ] 食指靠近平面後出現最近按鍵和 PowerShell 事件
[ ] 移開手指後才能再次觸發
```
