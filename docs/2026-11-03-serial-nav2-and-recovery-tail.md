# 2026-11-03 串口復原、導航程序隔離與橫移收尾

基線 d716b9e。本版不變更地圖、服務點、待機點、配送速度、Nav2 演算法參數、
footprint、雷達遮罩、定位驗證門檻或電源模型。kernel 1060、既有驅動與更新停用設定保持。
低電返航仍停用，等待 INA 改接並確認底盤大電池。

## 串口

`serial_to_ros`、`cmd_vel_to_serial` 使用可重連連線物件。讀取非阻塞且限制每輪資料量，
寫入 timeout 0.10 秒；失效關閉舊 FD，以 0.5、1、2、4、5 秒上限退避重新開啟。
檢查 `/dev/stm32` 指向的裝置身分，避免沿用 hotplug 前舊 FD。
同類重試錯誤每 5 秒最多一次；新的斷線邊緣立即提示，不再每 0.05 秒刷錯誤。

每次 TX 重開先送零速，不保存待重送速度。解除限制需 RX 的新 session、新鮮完整四輪組、
停車證明，以及至少一組在 TX 開啟之後收到的完整回授。
停車證明沿用三組零速、跨越一秒規則；正常行駛保留新鮮完整回授檢查。
兩組健康四輪組才標示 RX healthy，完整組最大年齡 0.65 秒；不改原 0.8 秒底盤 watchdog。
2 Hz 回授一旦組合完成，立即發布健康訊息，避免額外等待 0.2 秒心跳造成誤暫停。
速度命令佇列深度 1，檢查 DDS source timestamp，超過 0.25 秒不送出；無新命令時送零。
STM32 韌體、baud 115200 與 `$CMD,vx,vy,w#` 小數二位封包格式保持。

新增 `/chassis/serial_rx_health`、`/chassis/serial_tx_health`：包含 session、開啟次數、
錯誤次數、重試時間、回授完整時間戳、armed、拒絕與接受命令數、arming_count。
這些是車端診斷，不新增後端 API 欄位或 migration。

## Nav2 隔離與健康監看

`navigation.launch.py` 預設 `use_composition:=False nav_transport_profile:=async`。
Nav2 各元件使用獨立程序，採專用 SHM＋非阻塞 UDP／非同步發佈設定，
僅作用於 Nav2 啟動群組；保留 ROS 明確設定的 reliability/durability。
原本感測與里程計專用傳輸設定不變，未更換 RMW 或更新 ROS 套件。

`navigation_health` 以非阻塞 GetState 觀察 map_server、amcl、controller_server、bt_navigator、
behavior_server、collision_monitor；最多一個未完成請求／節點，請求逾 0.75 秒取消，
約 1 Hz 探測及發布。只觀察，不自行呼叫啟用、重設或重啟。
全數 active 且回覆 3 秒內才標示 ready；輸出 `/navigation/pipeline_health`。
此狀態提供給定位管理器判定資料管線；失聯不能靠舊 ready 允許續行。

啟動鎖亦辨識獨立 Nav2 殘留程序；避免只剩 collision_monitor 等程序時重複啟動。
stdout、stderr、exit code 持續存至 ROS launch log 資料夾，檔名改包含各元件名稱及 PID；
單一程序退出不關閉整組日誌。既有容器記錄功能仍保留，未收集 API credentials。

受控對照可明確指定（不要在原 start_nav 執行中重複啟動）：

```bash
# 本版預設
ros2 launch smart_delivery_core navigation.launch.py use_composition:=False nav_transport_profile:=async
# 舊模式對照
ros2 launch smart_delivery_core navigation.launch.py use_composition:=True nav_transport_profile:=current
```

不能據靜態測試宣稱已修復所有弱網問題。下次仍須測試遠端 RViz 開關、弱網、重連與實際導航。

## 橫移收尾

維持 0.30 m 任務、0.20 m/s 上限、25 mm 煞停預留及 8 秒硬上限。
增加 3 mm 終點容差，解決本次停在 0.273556 m 卻差 1.444 mm 才能進正常停止流程的情況。
末段速度不大於 0.02 m/s 且一秒無毫米級進展時：若已走至目標前 3 cm 內，主動送零並確認停妥；
否則仍判失敗。距離超限、cross-track、yaw、雷達、TF 與融合安全租約檢查均保留。
正常收尾仍需四輪停妥證明及安全租約解除，才進橫移後觀察與第二圈旋轉安檢，
不直接當作已定位成功。新增 target_tolerance_completion／settled_tail_completion 時序證據。

## 故障提示與續行

維持既有 ROS state 名稱，新增 `manual_intervention_kind`（pose/pipeline）及 navigation_health。
四輪回授失聯或導航服務失效，先安全停止與保留任務；資料持續不恢復時，
顯示「先恢復底盤／導航資料」，不持續要求在 RViz 重給位置。
資料完整恢復後重新進 VERIFYING，原六筆樣本、地圖及全域候選驗證仍須通過。
恢復觀察不會重新播送舊開機 seed；人工提供 pose 後也不以舊 seed 覆蓋。
真正位置不確定、幾何不通過等情況仍明確要求人工定位。

## 驗證

- 候選：689 項 pytest 通過；不包含既有 copyright/flake8/PEP257 lint。
- ROS localhost/domain 231，實際 pyserial＋PTY 假 STM32：斷開、換裝置、重連、等待回授／停車，
  全部實際發送封包為零速。正常 2 Hz 回授未反覆解除/重新啟用，兩個 session 各啟用一次。
- Nav2 原生隔離對照：組件現行與組件 async 均能啟動但關閉仍有子程序 SIGSEGV；
  獨立程序 async 啟動、關閉未出現 SIGSEGV。此為靜態對照，沒有遠端弱網壓力或真實導航目標。
- 完整候選入口確認 `/navigation/pipeline_health` ready，14 個 Nav2 子程序正常退出，
  各程序退出記錄均已保存。沒有操作實體串口或發送實車導航目標。
- 單元回歸包含 0.273556 m 真實案例，正常停妥後接回第二輪全域流程。

## 下次實車驗證

先架高測串口重插：零速、回授失聯、重連與新回授確認應順序發生，不重送舊速度。
恢復後以鍵盤短指令或低風險導航確認能接受新命令。
正常導航時分別開/關 VM RViz，觀察串口健康、Nav2 六個服務與 TF 接收；
弱網測試需確認停車與訂單保留，重連後依完整健康條件恢復。
搬車後觀察橫移能正常收尾，再依雷達安全條件開始第二圈；空間不足仍拒絕旋轉。

### 撞椅腳專項（本版未更改碰撞門檻）

1. 量測車體最外緣，包含輪、板材、線材／支架，確認 44×32 cm footprint 是否包住實體。
2. 用本次同類椅腳在前方、右前、側方定點觀察 raw/filtered 點數，保留精確事件秒數。
3. 低速接近並分開測試直行、原地轉向，確認 FootprintApproach 與實際零速／減速。
4. 檢查細椅腳能否滿足 min_points=6，及 range_min=0.15、costmap obstacle_min_range=0.25 的近距離影響。
5. 依實測再決定近距離停車區、窄障礙連續幀與近障礙轉速限制；不先放大遮罩或縮小危險區。

## 錄製及相容性

指令不變：`bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh`。
原 topic 全保留，增加兩個串口健康及導航健康 topic；補存 Nav2 XML 與各程序傳輸設定稽核。
前後端不需 migration；若網頁要區分設備中斷與人工定位，可另行對接細分診斷，
既有訂單進度／結果可靠重送保持相容。
