# 2026-11-02 供電、定位突波與旋轉視野

基線：3303695。此版不啟動真實低電返航。INA 仍接充電槽小電池；
`main_battery_source_verified: false`、`home.enabled: false` 及
`home.battery_source_verified: false` 保持不變。改接底盤共用大電池後，
仍需先驗證量測來源再另行啟用返航。

## 充電槽供電

- I2C 可讀不等於充電槽仍有供電。任一啟用通道原始電壓低於 9.5 V，
  所有槽位立即轉 `unknown`，保留 `sensor_ok` 表示 I2C 是否可讀。
  低壓持續 2 秒標示 `supply_fault`；復電須所有啟用通道至少 9.8 V
  連續 5 秒，再經既有槽位狀態確認。停用通道不參與供電判定。
- 供電無效時 API 沿用既有 unknown 槽位保護，停止 claim 新租還任務。
  Delivery 每筆資料都更新有效性與序號，無效資料不能沿用前一筆正常槽位，
  也不能銜接取還確認次數。
- 取還中短暫異常先暫停確認並顯示供電訊息；持續低壓 2 秒即明確失敗，
  其他無效資料連續 5 秒或完全沒有資料 10 秒亦明確失敗。
  正常操作仍為 3 筆確認、30 秒提醒、60 秒逾時。結果可靠重送機制不變。
- `/power_status` 新增選用 `slot_supply`：`healthy`、`state`、
  `fault_confirmed`、`failed_channels`、門檻與 `source`。既有 ch1–ch3 欄位不變。
  `vehicle_battery` 增加 `source_verified` 與 `voltage_source`，終端改稱
  Charging bus，避免把未驗證的小電池誤稱底盤大電池。
- 不新增獨立小電池 SOC 或啟用底盤返航。空槽電流、5/10 W 功率分類不變。

## 定位瞬時 covariance 突波

僅已 LOCALIZED 且一秒內有健康參考位姿時，才可開啟最多 1.5 秒的停車觀察。
需地圖吻合度至少 0.60、足夠扇區、近障礙比例正常，雷達、里程計、
AMCL 原始時間戳及時間對齊的 odom TF 都新鮮；位姿相對里程計的
殘差不得超過 0.15 m / 0.12 rad。

觀察時立即透過 `/localization/speed_limit` 輸出 0，融合節點亦立即發零速。
保留現有 action goal，避免短暫突波直接取消停靠。至少經 0.3 秒、
兩筆獨立且健康的 AMCL 資料後才釋放；資料失效、位姿跳動或超時，
仍進原本 SUSPECT／重新定位。此觀察不是降低全域定位驗證門檻；
全域幾何檢查及六筆穩定確認不變。不新增線上粒子雲分析。

`/localization/state` 新增 `covariance_probe`（active、failed、motion_zero、
elapsed_sec、healthy_samples）。觀察期間 ready 保持 true 以保留 action，
實際運動由零速限制阻擋；分析錄製時應同看此欄位與 speed_limit。

## 二次定位與人流旋轉

- 重新定位時以 callback 接收頻率收集雷達，在 20 Hz 非阻塞計時器補查
  **掃描時間戳的 odom TF**；每路待處理及有效快取最多各 5 張。
  不再只能由 5 Hz 決策時刻填入快取。缺 TF 可短暫重試，逾 0.3 秒即丟棄。
- raw 與 filtered 獨立驗證，反裝 yaw=pi、自體遮罩維持。
  連續未知角度上限 15° 改 20°，不改成 30°。
- 二次旋轉當前有效視野通常仍需 75%；只有當前至少 73%、
  至少 3 張獨立近期對齊資料且合併視野至少 80% 時，才允許補足。
  當前已達 75% 時，原合併 75% 規則保留。
  任一近障礙、資料過期、位姿跳動均不授權旋轉。
- 人流旋轉只改未知角度為 20°，當前與合併 75% 不變。
  等候、停止、剩餘角度重試及任務優先規則不變。
- 橫移 95% / 4°、距離 0.30 m、速度上限 0.20 m/s、二次等候 8 秒均不變。
- 待機計時暫緩訊息須同一原因持續 0.3 秒才印出，仍限制 5 秒一次。
  即時安全結果與結構化診斷不延遲，短暫故障仍能從錄製資料查到。

## 驗證與下次測試

候選版本：659 項 pytest 通過（既有 copyright/flake8/PEP257 不在此回歸範圍），
並完成 localhost/domain 231 的 ROS 啟動、日誌、SIGINT/SIGTERM 結束及
零速限制 DDS 路徑驗證。沒有發出實車目標或非零速度，不代表已完成實車驗收。

下次維持完整錄製：`bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh`。
原有項目不減少，新欄位位於已錄製 topic。觀察：正常取還、供電斷開與復電、
座位區正常導航、搬動後第二輪旋轉及人流自旋；空間不足仍應拒絕移動。

網頁/API 不需 migration。既有 unknown 狀態與任務 progress/result 訊息沿用。
若後台要顯示供電原因或正確標示電池來源，可選擇顯示新增的 ROS 診斷資訊；
目前不新增雲端欄位。Nav2 關閉時的既有容器崩潰尚未修復，本版未改關閉流程，
也未更新 kernel、Hailo 或 ROS 套件。
