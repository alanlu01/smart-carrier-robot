# 二次定位證據刷新、人流安全續轉與簡潔狀態文字

基準 `c8c7b58`。本版不改地圖／座標、速度、電池模型、雷達遮罩、DDS 設定、AMCL covariance 與獨立樣本門檻。

## 二次等待持續刷新，但不延長原期限

- `second_guard_wait` 仍受原 8 秒 wall-clock 限制，保持零速；每個原觀測週期檢查新 scan，刷新地圖吻合度並要求 AMCL 靜止更新，不再直接跳過證據刷新。
- 地圖更新以獨立 scan stamp 去重；TF 尚未就緒的同 scan 可稍後重試，但成功更新後不能靠重複同幀累加觀測。
- 六筆確認仍由既有 AMCL callback 驗證，不由 timer 自行湊樣本。second_guard_wait 另外拒絕重複／倒退／不合法／過期 AMCL source stamp，以及有移動命令、odom 移動或 motion-inhibited 的樣本。
- 只有 covariance、地圖品質、新鮮感測、位姿一致性與原 6 筆健康樣本全部通過，才可在原地恢復，不必強制再轉一圈。
- 如仍不合格，就繼續原來有碰撞／視野檢查的橫移或第二圈；時限內皆不可用仍請人工介入。沒有把 NaN/Inf 轉為自由空間，也沒有放寬橫移 95%／4°或旋轉 75%／15°。
- `/localization/state.recovery_failure_checks` 區分 map unavailable/stale/update_failed/low/insufficient_samples/recovered、AMCL unavailable/stale/covariance_unhealthy/healthy 與穩定樣本數、感測 blockers。資料過期不再寫成新觀測的低吻合度。

## 人流短暫資料抖動：停止、排空、等待後續剩餘角度

- 僅在新鮮定位相位為 LOCALIZED/SUSPECT、地圖一致、無移動禁止／電池接管時，允許因 TF、odom 或 scan 短暫失效進既有有限等待。
- 當前 action 先取消，必須看到 action 終態與獨立連續停穩，再談恢復；不因舊視野曾清空而繼續輸出速度。
- 等待不代表授權移動。恢復仍需新鮮 LOCALIZED、原 Delivery dispatch gate、0.30 秒的 odom/scan/TF 新鮮度、停穩與新雷達視野證據。
- 每次最多 20 秒、每輪最多 2 次；原 90 秒分析總會話、續轉額度與原地圖修正重建額度保留。正常續轉只送尚未轉完的角度，不每次重轉 360°。
- 定位真正升級至全域／人工等狀態，結束本輪方向證據；顯著 map→odom 修正仍按既有規則丟棄舊統計並有限重建，不能把不一致方向資料繼續計分。
- 近障仍立即取消；新訂單／低電量仍優先接管。持續缺資料、等待過期或額度用盡仍退出。
- **沒有採用候選的 20°盲區**：分析仍 15°，有效率 75%，連續安全 0.75 秒、3 個獨立 scan、停穩 1 秒均不變。

## 終端：圖示＋簡短中文，診斷保留在狀態訊息

```
➡️ 前往人流分析點
⏳ 等待停妥與安全視野
🔄 人流分析中（180°／360°）
✅ 分析完成，選擇：座位
➡️ 前往待機點：座位
⏸️ 等待定位／感測恢復，保留目標：座位
💤 已停妥：座位
```

- 獨立純呈現模組 `delivery_console.py` 統一啟動、人流、暫停、待機與失敗文字，不改 API/state reason codes。
- 待機時短暫失去資料改為 INFO「⏳ 待機計時暫緩：…」；進行中的人流動作安全暫停仍 WARN。取消尚未證明完成時顯示「🛑 人流動作停止中」，不宣稱已停穩。
- 原本每次印出整串 JSON 的人流狀態、分數與啟動資料，不再全部塞進終端。完整診斷仍在 `/smart_carrier/optional_idle_state`、`/delivery/startup_state`、`/localization/state` 與既有任務/action topics 中。
- 狀態變更立即顯示；分析／安全等待約每 5 秒更新；資料錯誤訊息仍節流。Nav2 等第三方原始錯誤不攔截刪除。
- AMCL 靜止更新的每筆 INFO 改由 `stationary_update_requests` 計數保留，避免一秒一行洗版，不減少靜止更新請求本身。
- 終端字串不是機器介面；使用本倉庫狀態 topic 的工具不需更改。自行解析舊終端字串的外部腳本需改用 topic／新文字。

## 消費端耗時診斷

- 使用既有固定空間 DurationMetrics 測量定位 timer／state machine、map matching、AMCL request、second spin guard、lateral guard、state heartbeat 與主要感測 callbacks。
- Optional runtime 記錄 poll、pose lookup、spin guard 耗時。純診斷，不更換 executor 或另增工作執行緒。
- 方法裝飾器保留 callback 原簽名、回傳與例外；不捕捉輸入內容、不記錄秘密、不額外控制硬體。
- 定位 tick 另存 lifetime 接收間隔，避免原 5 秒 reset 的 max 欄位錯過一次延遲。可配合獨立 scan monitor 區分來源／傳輸與消費節點積壓，仍不能僅由 duration 指定唯一根因。

## 驗證與限制

- 候選靜態／mock 功能與安全回歸 602 項通過；不包含既有 copyright/flake8/PEP257 檢查。
- localhost/domain 231 使用實際 ROS 訂閱與 production 裝飾後 raw callback，20 筆假雷達回呼成功；完整診斷 JSON 與簡潔中文輸出均可用。沒有建構真正 LocalizationManager、沒有 Nav2、goal 或馬達指令。
- 仍需實車：二次安全等待中原地收斂／仍不合格的人工介入、人流自旋資料抖動後剩餘角度續轉、近障立即停止與新訂單接管。
- 所有既有錄製 topics 保留；錄製指令不變：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

本版不涉及前端/API migration，不需要網頁隊友修改。
