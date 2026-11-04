# 2026-10-28 唯讀分析：Delivery 退出、初次定位暫停與橫移中止

## 範圍

- Pi `robot@[redacted-ip]`，車載／本機／GitHub main 均為 `242ee2dc3ba4307f66dcc9268bf242be873c2ca6`。本機與車載工作樹乾淨，非部署版本不一致。
- run：`/home/robot/test_logs/20261028_194505_full_system_B2CPyL`。
- events 19:45:09–19:58:51；MCAP 約 19:45:29.567–19:58:49.896，800.329 秒、219,247 筆訊息。
- bag/info 結束碼為 0，console 管線碼 130；離線讀取成功。不將 console 中斷碼等同於 bag 損壞，也不保證 DDS 無遺失。
- 只讀 bag、日誌、程式與 Git 提交；僅另以不建立 ROS node、不初始化 ROS 的 logger 記憶體重現驗證，不改任何車載檔案、參數、節點、訂單或 GitHub。
- 日期時間皆為台灣時間。使用者口述約 7:57 的自救，記錄實際橫移約 19:56:42–48；19:57 已在等待人工。

## 使用者確認

- Delivery 退出的終端 Traceback 最後為：

```
optional_idle_runtime.py:566 in _safe
    (logger.warning if self.motion else logger.info)(message)
ValueError: Logger severity cannot be changed between calls.
[ros2run]: Process exited with failure 1
```

- 第一段導航後定位暫停前沒有搬車、碰 USB 或操作熱點，屬非預期暫停。
- 後段定位前有搬動／轉動車輛；橫移後沒有再碰、扶車或有人走近；19:58:07 有手動給位置。

## 1. P0：Delivery 退出為 242ee2d 引入的確定回歸

### 根因

`optional_idle_runtime.py:564–566` 將原本固定 WARN 改成同一個 callsite 根據 `self.motion` 動態切換 INFO/WARN。

本機安裝的 `rclpy/impl/rcutils_logger.py` 依 CallerId（函式、檔案與行號等）保存 logging context；同一 logger/context 的 severity 改變時，313 行直接拋出 ValueError。先在 idle 印 INFO，之後行動中檢查失敗印 WARN，即觸發未捕捉例外。例外從 `_safe` 穿過 `poll` 到 Delivery main，程序退出。

這是訊息呈現修改造成的程式錯誤，不是人流模型、相機、盲區門檻或硬體故障。

### 提交對照

- 前版 [c8c7b58](https://github.com/alanlu01/smart-carrier-robot/commit/c8c7b582d2c1dc856d505301462ea4c488746dfc)：固定 `get_logger().warning(...)`，沒有這個動態等級 callsite。
- [242ee2d](https://github.com/alanlu01/smart-carrier-robot/commit/242ee2dc3ba4307f66dcc9268bf242be873c2ca6)：新增同一行 INFO/WARN 動態呼叫，`git blame` 明確指向此提交。
- 全庫搜尋相同模式，本次找到的直接風險位置為上述一處；其他定位狀態的 info/warning 已分開不同程式行。

### 實際 rclpy 重現，不動車輛

使用實際 installed OptionalIdleRuntime `_safe`、假的感測欄位與實際 `rclpy.logging.get_logger`；沒有 `rclpy.init()`、ROS node 或動作。序列為 idle→moving→idle：

- 目前 242ee2d：False → **ValueError** → False。
- 從 Git 在記憶體載入前版 c8c7b58：False → False → False。

### 本次事件

- 第一個 Delivery：19:51:47 開始人流自旋，狀態心跳隨後消失；19:51:49 goal_guard 偵測導航心跳逾時，19:51:50 取消殘留目標。
- 重啟後第二個 Delivery：19:53:41 前往分析點，19:53:59 再出現心跳逾時；19:54:00 殘留導航被取消。
- 使用者提供的 Traceback 證明至少一筆為上述 exception；第二筆事件型態相同，未另外取得第二份 stderr，不能單憑消失的心跳證明每次都同一例外。
- 所以風險不只在「已開始看人」：任何 optional motion 中 `_safe()` 從 INFO 路徑切到 WARN 都可能觸發。
- goal_guard 有實際取消殘留 action；這不代表程序崩潰可接受或會瞬間停止，仍需優先修正。

### 漏測與建議修復

先前 602 項回歸的 logger 是簡化 fake，沒有 rcutils 的 callsite cache；隔離 ROS 驗證只走 idle 資料失效，未經過同一 logger 的 INFO→WARN 切換。通過測試不能代替這條真實狀態路徑的驗證。

建議最高優先：將 INFO/WARN 明確拆為不同程式行；用實際 rclpy logger 測 INFO→WARN→INFO 與反向序列，再驗證 moving 狀態、短暫失效、取消排空與續轉。不要以降低 logging level 或吞掉 ValueError 當作修復。另應評估未捕捉 optional 例外時的有界安全收尾，不能讓 owned action 等到失聯保護才取消，也不能未確認停妥就繼續服務。

## 2. 初次導航定位暫停：AMCL 回呼阻塞約 3.956 秒

### 證據

- 19:46:00 Delivery ready，19:46:03 開始前往電機工程學系。
- 19:46:10.922 定位管理器以 raw/filtered 皆約 3.99 秒未收到為由進 RECOVERING_LOCAL；tick gap 約 4.146 秒。
- 同一心跳的新增 `wall_durations.amcl_callback`：47 次中唯一一次 over_50ms，最近／最大耗時均約 **3.95616 秒**，峰值 age 約 0.013 秒；即剛剛發生。
- 此前各感測 callback 與 state heartbeat 最大為數十毫秒，當次長耗時不是地圖評分 timer 本體。
- 但整包 raw/filtered scan 最大接收間隔僅約 0.306/0.330 秒，public odom 最大約 0.198 秒；獨立資料來源持續更新。
- 因此是定位管理器消費端在 AMCL callback 中卡住，導致自身快取過期，不是 raw lidar、filtered publisher 或四輪回授真的停了 4 秒。
- 19:46:21 恢復 LOCALIZED，未自旋；19:46:24 繼續原任務。

### 更細根因：已縮小範圍，尚非完全定案

- AMCL callback 內有同步 `_persist_trusted_pose()`，目前每 2 秒可寫一次可信位置快取，包含 `write_text` 與 `os.replace`。
- 19:46:13 的系統取樣對應區間 iowait 約 6.65%，I/O pressure avg10 約 13.8%，與事件有時間相關性。
- 此同步快取路徑源自較早提交 `0b987cc`（9/15），不是 242ee2d 新加的功能；10/27 新診斷讓長 callback 更容易定位到。
- 目前沒有獨立持久化子步驟耗時，故**同步寫檔是優先嫌疑，不應寫成已證明唯一根因**。

建議：補 cache write/atomic replace 與 callback 內細分耗時；若證實磁碟阻塞，將運行中可信快取寫入移出感測 callback，採單一 writer、有界 latest-only 任務與原子替換。保留 boot marker、boot/map identity、原觀測時間與新鮮度檢查，不直接停用快取，也不放寬 scan watchdog。

## 3. 橫移後未旋轉：停止距離超出既有上限

### 記錄

- 19:56:20–33 第一圈全域 Spin 成功完成；之後 AMCL xy_std 仍約 1 m 以上，未通過位置可信度。
- 19:56:42.794 許可左移 0.30 m，速度上限 0.25 m/s。
- 約 19:56:46.1 chassis_cmd_vel 已為零；輪速里程計後續仍增加，19:56:46.54 附近到達最終估計。
- 最終沿目標橫移方向約 **0.333646 m**，交叉方向約 **0.020063 m**，偏航約 **-3.789°**。
- 既有停止門檻為距離≤0.33 m、交叉方向≤0.05 m、偏航≤5°。直接越界的是距離，超過 33 cm 上限約 **3.65 mm**；不是本次 yaw 或 cross-track 超限。
- 19:56:46.974 停止階段已記 `橫移煞停距離／角度偏差超限`；19:56:48 確認四輪停妥、解除租約後轉人工。程式明確禁止 abort 後再開始第二圈。
- 記錄中的移動距離是輪速里程計估計，不是外部尺量；可能包含實際停止尾段與四輪分組回授更新延遲。使用者已確認沒有外力干預。

### 版本對照與建議

- 此停止偏差保護源自 `69390a9`（10/7），並非 242ee2d 新增；最新提交只在相關函式增加耗時診斷／文字，未更改橫移速度、距離或此門檻。
- 不應為了強制第二圈而略過 abort 或直接放大允許過衝。
- 建議量測零指令到各輪實際零回授的時間、評估分組延遲；再提前減速／停止或降低短距離橫移速度，使目標與停止裕度匹配。是否改 0.25 m/s 應另行討論，這輪未改。

## 其他結果

- 3 筆服務均 done，且皆有 cloud_confirmed；本機 durable ACK 約 0.036/0.068/0.187 秒，雲端確認約 6.513/13.452/5.632 秒（以 bag `/task_result`→ACK 訊息到達差計）。含背景排程、重送與通訊，非單次 HTTP RTT 或前端顯示耗時。
- INA3221 1,600 筆 power_status 沒有 sensor_ok=false；有效電壓約 11.168–11.376 V。
- 語意半速 27 組可配對指令皆符合 ×0.5，但樣本少，不代表所有行人情境測試完成。
- 有 API read/TLS timeout，但 NetworkManager 本次沒有完整 link disconnect 紀錄，不能把首段暫停直接歸於熱點斷線。
- CPU 取樣平均約 81.0%、最高約 94.1%（四核心正規化、扣除 idle/iowait）；溫度約 49.1–57.3°C，可用 RAM 最低約 5.12 GiB，throttled 全為 0x0。仍不建議提高速度／控制頻率。
- 檢查時剩餘磁碟約 12 GiB，使用率 80%。未刪資料。

## 建議優先順序

1. **P0：修 logger 等級切換造成的 Delivery 崩潰，補真實 rclpy 狀態轉移回歸。** 在修復前不建議再跑人流實車展示。
2. **P1：追 AMCL callback 同步阻塞，優先分離／量測持久化 I/O。** 保留硬體與感測逾時保護。
3. **P2：調整橫移的減速與停止裕度／回授延遲補償。** 不是單純多轉一次即可解決。

本次僅分析，未部署、推送、停節點或取消任何訂單。
