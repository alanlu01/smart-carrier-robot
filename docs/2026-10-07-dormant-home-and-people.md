# 自動回家與人流智慧待機：已準備、預設停用

## 本次範圍

準備功能 1、3；LLM／語音下單暫不實作。**不啟用、不填假座標、不切換地圖。**
既有 INA 電池電壓門檻／量測來源、一般服務與待機座標、導航速度、定位門檻、
Hailo 模型與相機推論率、1060 核心及停用自動更新均不變。

權威設定：`smart_delivery_core/config/optional_idle_features.yaml`（套件 share 下亦安裝）。
兩個 enabled 都為 false；home pose、觀察區與新增候車點皆未填。
關閉時不建立 OptionalIdleRuntime、不新增 ROS 節點／視覺模型／觀察計時器。
Nav2 仍讀原設定，僅正常 BT 明確指定原本的 general_goal_checker，容許誤差不變。

## 1. 低電量回家

- 只接受「已確認的大電池來源」之專用 `/vehicle_battery/verified_status`。
  不訂閱目前量到行充小電池的 `/vehicle_battery_status` 或 `/power_status`。
  以下是**未實作的量測來源適配器將來應提供的訊息格式**，不表示現在已有該 topic：
  `{"source_id":"main_battery","sensor_ok":true,"stamp_sec":1790000000.0,"voltage":10.1}`。
- 觸發電壓暫為 null，需量測後確認；既有 9.8/9.5 V INA 設定未動。
  確認電壓後要求低於觸發值連續 15 秒；讀取中斷、過期、無效與重複時間戳
  不累積時間。觸發後持久鎖存，電壓反彈或配送重啟不自行解鎖。
- Bridge 停止新 claim，但仍處理 heartbeat、對帳、取消、可靠結果重送。
  已開始的導航／取還／定位暫停任務繼續正常處理；尚未出發的車端任務逐筆
  透過既有 released 流程釋回雲端，不默默刪除。claim 已在途者會經對帳處理。
- 沒有新服務工作後返回 home pose；home 專用 BT 使用獨立 goal checker。
  預備到站要求為 XY 2 cm、yaw 約 2°，且新鮮 TF、定位可信、里程計停妥
  持續 1 秒；僅 Nav2 SUCCEEDED 不算到家。一般停靠仍維持原 0.10 m／0.10 rad。
- 失敗／取消／精確到站未確認時停車並標記 home_failed，不自行無限重試。
  已到家且大電池電壓恢復至觸發值以上 0.5 V、資料健康且停妥，才可人工呼叫
  `/smart_carrier/reset_home_latch`（std_srvs/srv/Trigger）解除鎖定。
- 本階段沒有自動充電、接點偵測、退站、ArUco 或視覺伺服。2 cm 是軟體判定
  門檻，**不是保證實車在紙箱總寬多 5 cm 的空間可以安全停入**。需實測停車
  誤差、地圖定位誤差、制動與箱體可偵測性，不足時再加入 ArUco 最終對位。

## 3. 人流智慧待機

- 預備流程：無訂單持續 60 秒 → 前往量測好的開闊觀察區 → 確認已停止且
  即時雷達足夠可用 → 一次 360° 觀察 → 選擇方向需求最高、距離較近的既定
  待機點。可填 3～4 個候選點，不改目前兩個正常待機點。
- 沿用現有 Person 與相機時間戳，不新增推論模型、不提高 FPS。
  以取像時的 TF 對齊方向、拒絕舊影像／重複時間戳、最低信心 0.65。
  每方向格需要至少三幀，以幀內人數中位數計分，不把每幀加總成「獨立人數」。
  這是**方向人流需求估計**，不宣稱身份追蹤／精準去重或人物距離估測。
- 自旋使用 NavigateToPose 擁有的 Spin 子樹：服務訂單到達、低電量、定位下降、
  odom／雷達過期、超時均取消；終止後須確認停止才交還服務控制權。
  既有配送目標 heartbeat guard 亦可取消整棵樹，避免另外一個 node 搶控制權。
- 雷達按完整實際 TF 轉換（含反裝），檢查 0.44×0.32 m 底盤旋轉外接圓
  加 10 cm；有效波至少 75%，未知連續扇區不得超過 15°。Spin 本身仍有
  Nav2 碰撞檢查，且速度仍經平滑器、碰撞監控、語意融合、底盤 watchdog。
  玻璃／紙箱及遮蔽仍有限制，不能把未觀測範圍視為安全。
- 自旋途中繼續檢查雷達；至少約 333° 的實際 odom 旋轉與 75% 方向覆蓋
  才能採用結果。無人、資訊不足、失敗或中斷時不指定任意人物位置，回原流程。
  觀察後冷卻 600 秒，不無限自旋、不巡邏、不追人、不直接朝人群開車。

## 日後啟用順序（本次不執行）

1. 測量並確認新地圖、正常服務點、待機點、home 與開闊觀察區。
2. 在對應地圖載入後，用唯讀 `ros2 run smart_delivery_core optional_idle_map_inspect`
   取得 occupancy_fingerprint；另以 sha256sum 取得實際 map YAML 雜湊。
3. 填設定中的地圖雜湊、coordinates_verified、所需座標。換地圖即需重新確認。
4. 自動回家另需確認大電池來源、適配器訊息與觸發電壓。只有座標仍不夠。
5. **Nav launch、smart_delivery、API bridge 必須共用同一份 optional_idle_config**，
   建議先只使用安裝的預設設定檔，不單獨覆寫某一個程序。各自重新啟動。
   導航須經本專案 navigation.launch.py，才能條件式載入 home goal checker。
6. 先架高驗證停止接單與釋回，再空曠低風險區測觀察取消，最後有人監看測回家。

## 記錄與驗證

保留先前所有錄製項目，增加 optional_idle_state、task_admission 與 verified_status；
設定快照增加 optional_idle_features.yaml。關閉時這些 topic 無發布者屬正常。
功能只做靜態／替身與模擬回歸，**未啟用、未進行實車導航或電池低壓測試**。
現有服務點、電池設定、新地圖、核心與更新政策部署前後應做雜湊比對。

本次 ROS Jazzy / aarch64 環境回歸：296 項通過；包括原有配送、定位、電源、
視覺與 API bridge 測試。另核對新增 BT 的 ports 符合已安裝 Jazzy manifest。
此測試不包含既有 ament 版權／flake8／PEP257 格式檢查，也不替代實車驗證。
