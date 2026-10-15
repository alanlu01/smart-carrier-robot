# 2026-10-15 發布隔離、靜止定位與人流續轉

基準 `eef281d`。備份：`/home/kj0921/backups/transport_recovery_20261015_13VDk6/dev_ws_src.tar.gz`。

## 修改範圍

### 1. 避免公開發布卡住四輪回授

- `mecanum_odom_real` 的輪速解算、積分、校正係數、20Hz與0.8秒watchdog不變。
  改為只發布內部 `/chassis/odom_local`，Best Effort、KEEP_LAST 1。
- `start_all` 使用的 `vm_robot_model/smart_bringup.launch.py` 自動多啟動獨立程序
  `odom_transport`，轉送原有公開 `/odom` 與 `/tf`，兩者仍為RELIABLE且保留原時間戳。
  過期超過0.20秒、重複／倒序、無效姿態或錯誤frame的內部快照不轉送。
  TF呼叫若延遲，轉送odom前再驗一次年齡，不替舊資料蓋新時間。
- 分離程序讓公開DDS發布等待不再占住四輪回授／積分執行緒。
  即使公開管線壞掉，仍不代表可以繼續導航：原有感測新鮮度與停車保護保留。
- 僅積分、relay、laser filter程序套用 `fastdds_sensor_publish.xml`：
  SHM+UDP維持、非同步發布、顯式可重分配history memory，writer等待預算100ms。
  不修改 `.bashrc` 或全域RMW，不降低訂單／action／map／static TF可靠性。
  非同步不等於任何條件下絕對不阻塞；仍記錄write時間與錯誤，後續要做真實弱網對照。
- `/chassis/feedback_health` 是監看資料，改Best Effort，避免健康發布本身拖住回授。
  若手動echo此topic，需 `--qos-reliability best_effort`；錄製會自動適配。

Fast DDS Jazzy的同步／非同步與XML覆寫依據：
[官方rmw_fastrtps說明](https://github.com/ros2/rmw_fastrtps/blob/jazzy/README.md)。
這不是核心／ROS／Hailo升級，亦不是把全部topic改成Best Effort。

若手動只執行 `ros2 run smart_delivery_core mecanum_odom_real`，現在還需啟動
`ros2 run smart_delivery_core odom_transport` 才有公開odom／TF；正常 `start_all` 已包含，
VM仍用原 `display.launch.py`，不用改顯示topic。

### 2. 靜止搬車與派發前確認

- 當原有嚴重map-match門檻（≤0.20）持續出現，且AMCL健康、實車靜止，
  原始／過濾雷達來源新鮮、odom新鮮、驗證無blocker、有效方向≥4、
  近距離遮擋比例≤60%，才累計。
- 至少10秒且5次獨立map觀測；觀測中斷／逾時、視野不足、近人遮擋或恢復便歸零。
  不把一幀遮擋當搬車，也不把重複timer當新觀測。
- 觀察期間 `/localization/state.dispatch_allowed=false`，Delivery及人流入口暫緩派發，
  不先送goal再靠有速度才揭露問題；訂單仍接收保存。
- 確認後進既有SUSPECT→驗證→必要時全域，仍不跳過原有定位品質門檻。
  正常定位完成與重建map窗口時重設此觀察。

### 3. 人流安全許可與續轉

- 至少3筆獨立安全scan、停妥1秒之外，再要求安全許可連續0.75秒。
  任一拒絕重設連續安全時間，不放寬原15°未知區或75%有效比例。
- 未知區仍立即安全取消，停妥後等新證據，沿用90秒整場與10秒單次等待上限。
- 續轉請求與實際續轉分開：續轉後未取消且odom有≥0.05rad進展，才計一次實際續轉。
  最多2次實際續轉，同時最多4次續轉請求；任務優先與硬障礙／過期資料停止不變。
- 續轉只轉剩餘角度，不額外多轉360°。若持續盲區無法取得許可，仍安全退出；
  不保證所有位置都能完成分析。

### 4. 第二輪定位等待

- `second_guard_wait_sec: 8.0`，等待時維持零速。
- 左右側移每秒有限重查；安全拒絕不耗掉唯一側移，實際開始arming才算使用。
- 側移不可用但旋轉安全仍可第二輪原地全域；仍不安全到期就要求人工。
- 原有完整四輪停止證據、lease交接、側移30cm／0.25m/s、最多兩輪均保留。

### 5. 診斷、結束與錄製

- 底盤健康依當下回授age／timer延遲計算，不僅沿用上一個healthy callback。
- 新 `/chassis/odometry_transport_health` 提供來源年齡、成功／丟棄／write錯誤及發布耗時。
- 定位增加stationary_map_observation、dispatch_allowed與第二輪等待elapsed；
  人流增加續轉請求／pending、連續安全時間。
- ready先變false但state尚未更新的終端文字改成「ready=false（最近狀態…）」；
  靜止品質確認阻擋派發時，不再誤印成啟動穩定等待。
- 修正里程計Ctrl+C重複shutdown例外。
- 錄製原59項全部保留，新增 `/chassis/odom_local`、`/chassis/odometry_transport_health`；
  額外保存感測DDS設定，不錄raw image，不改API或前端協定。

## 驗證

- 回歸測試包含原功能、低分防抖、搬車持續觀察、派發predicate、relay新鮮度／配對、
  安全等待解除、續轉記帳、過期健康值。
- 隔離domain231／LOCALHOST：20份合成odom配對公開TF、0個write錯誤；公開odom仍RELIABLE。
- 分程序故障注入：relay每次TF刻意卡0.8秒；4秒內內部odom80份，最大間隔約0.0504秒，
  四輪healthy，0份過期公開odom；沒有goal或motor命令。
- 原恢復cancel／lease交接隔離測試通過。
- 安裝的C++ laser_filters用反裝TF與30份合成scan測試，收到29份filtered，無goal／motor命令。
- 直接讀10/15 bag做觀察反事實對照：13:55:34.785標記待確認，13:55:45.386達持續門檻，
  不必等13:57下單。不是把bag控制播放到實車，也不證明閉迴路必然成功。

## 下次實車驗證

沿用 `bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh`。
全套ROS重開後確認`odom_transport`存在、VM map/scan/TF正常；人流點周邊留空，
測完整自旋與未知區暫停／續轉；靜止搬車觀察是否先暫緩派發再恢復；
第二輪遇短暫人員後退開，確認有限重查而不繞過真障礙；最後做VM連／不連與弱網對照。
不在沒有空間／無法看守時做搬車或自旋測試。

本次不改地圖、服務／待機座標、雷達遮罩、正式停靠精度、電源／3筆取還模型、
MPPI／速度、視覺FPS、Hailo、1060核心、自動更新停用、真實低電返航停用狀態。
無需網頁或API migration。
