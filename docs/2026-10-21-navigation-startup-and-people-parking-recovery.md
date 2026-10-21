# 導航啟動防重複、人流停靠續行與感測通訊防阻塞

基準：`64ec9bf`。本次不修改地圖、座標、速度、AMCL 信心門檻、雷達遮罩、電池設定或前後端 API。

## 導航啟動

- `navigation.launch.py` 在任何 Nav2 子程序啟動前，取得同主機／使用者／ROS domain 的 flock。鎖由 launch 程序持有，正常關閉或程序死亡後由系統釋放，鎖檔保留，不以刪檔作解鎖。
- 同時檢查同 domain 的既有 Nav2 容器與 standalone map_server/amcl/controller/bt_navigator，避免舊版未受鎖保護的啟動殘留。不殺程序、不重設生命週期。
- 第二次 `start_nav` 明確拒絕，顯示既有程序資訊。原本的 shell alias 不變。
- Delivery 啟動仍採 30 秒總上限，保留 unavailable service／RPC timeout／inactive／duplicate node 等原因及同名節點數。若發現目標 bt_navigator 有多個同名同 namespace 節點，即使其中一個回 active 也不派工。
- 主機鎖不能替另一台 VM 上的 launch 上鎖，也無法完全涵蓋外部直接執行 upstream bringup 的競態；VM 仍應只開 display，不另開 Nav2。Delivery 的圖查詢受 DDS discovery 完整性限制，不能當作跨機唯一性保證。

## 人流停靠選點續行

- `going_people_standby` 的可恢復安全中斷先取消 action、等待終態與獨立連續停穩證據，再進 `waiting_people_parking_recovery`。不能只憑取消 ACK 送新 goal。
- 保留已完成分析與選定的絕對 map 座標；不重算最近待機點、不重新做整圈人流自旋。完成分析的 600 秒冷卻時間不延長也不縮短。
- 每次最多等待 90 秒、最多 2 次，且受原停靠總 deadline 限制；停靠總發送上限仍為 3 個 goal，安全中斷也不能造成無限次重新發送。
- 全域定位期間允許原地有限等待，但不保留舊 action／旋轉權限。恢復後需新鮮 LOCALIZED、原 Delivery dispatch gate、地圖 fingerprint、有效 TF／odom／scan、電池與禁止移動狀態全部通過，連續停穩後才可對原選點發新 goal。
- 原選點必須仍是現配置的待機點，不能在換圖、座標被改動或未知位置狀態下任意續行。新訂單、電池接管、人工定位、失去有效地圖或等待逾時終止本次意圖。停止／取消尚未證明完成時，依舊不允許新 motion。
- 維持原本 0.10 m / 0.10 rad 的實際抵達確認及連續停穩，不放寬停靠精度，也不以 Nav2 SUCCEEDED 單獨回報抵達。

## 安全檢查與分析前等待

- `_safe()` 保留同樣條件，診斷分列地圖、Delivery ready、移動許可、電池、odom 來源／接收新鮮度及 map pose；TF 查詢失敗或來源過期保留原因。
- `/smart_carrier/optional_idle_state` 增加 `safety_check`、`last_unsafe_check`、停靠恢復次數與剩餘時間。診斷不反過來覆寫定位管理器 ready。
- 終端失敗詳情最多每 5 秒 1 筆；心跳保留最近失敗資料，避免為了減少訊息而丟失分析依據。
- 已到分析點、尚未起旋轉時，短暫 TF／接收抖動改用原 10 秒等待期限，期間不移動，也不延長 0.30 秒資料新鮮度標準。原有獨立 scan、0.75 秒連續安全、1 秒停穩與 15°盲區上限不變。
- 全域／人工等定位升級仍清除尚未完成的方向證據，不將另一輪定位旋轉當成人流掃描。

## 感測通訊

- 保留現有 SHM + UDP、ASYNCHRONOUS 及各 topic 原 QoS；沒有把 `/odom` 或所有系統 topic 全部改成 BEST_EFFORT。
- 只在雷達濾波、odom 整合與 odom relay 程序使用的 sensor XML 加入 UDP `non_blocking_send=true`，並明確設置 publication mode 環境值。
- [Fast DDS 官方說明](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/api_reference/transport/udp_transport/udp_transport_descriptor.html)：UDP socket buffer 滿時可立即返回並丟棄該 datagram，而非卡住 send_to；可能影響遠端吞吐量。既有可靠 DDS 的重傳與本機資料逾時停車保護仍保留。此設定不是所有 write 路徑的硬即時上限，也不能保證完全消除可靠 writer 資源競爭。
- `audit_sensor_transport.py` 只讀目標程序的允許環境鍵與 XML hash／policy，不輸出 API token、不啟動 ROS node。設定檔與環境是「配置證據」，不是 runtime publication mode 或弱網路改善已被證實。
- 錄製指令不變、舊 topics 全部保留；增加開始／結束 audit 檔，並在既有 system_health 每約 30 秒記錄一次，涵蓋先錄影後啟動節點的順序。

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

## 驗證與下次測試

- 候選功能／安全回歸 560 項通過，不包含既有 ament copyright/flake8/PEP257 檢查。
- localhost/domain 231 假里程計取得 20 筆 paired odom/TF、write_errors=0；假雷達經實際 laser_filters 收到 29 筆，新 XML 無 parser error。未發布馬達控制或導航目標。
- 實際 navigation launch 的 preflight／第二次拒絕也在 domain 231 驗證，只建構 action、不執行 Nav2 子 action。
- 仍需實車驗證：分析選座位→途中短暫 SUSPECT→恢復後仍前往座位；全域恢復後同目標續行；等待時新訂單接管；到分析點的短暫 TF 抖動不立刻退出；弱網路時 filtered/odom 是否仍有多秒延遲。
- 不需要前端／API migration，不需通知隊友修改網頁。
