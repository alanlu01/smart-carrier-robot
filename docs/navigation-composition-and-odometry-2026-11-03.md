# 2026-11-03 導航組合模式與回授一致性

## 本次變更

- `navigation.launch.py` 預設恢復 `use_composition:=True`、`nav_transport_profile:=current`；保留啟動鎖、健康診斷、完整程序輸出與退出紀錄。所有 Nav2 服務保留。
- 原生 gdb 測試在完整組合模式退出時，分別發現 MPPI ParametersHandler 解構及 rclcpp Context 解構呼叫到已卸載函式庫。`nav2_composed_bringup.launch.py` 僅對導航容器設定 `LD_PRELOAD`，從已安裝 ament 元件索引取得元件庫並保留至程序退出，另保留 MPPI controller/critics。沒有修改 `/opt/ros`、全域環境或系統套件。單獨隔離 docking/控制器、改執行器、依序 lifecycle shutdown 並未解決全部問題，未採用。
- 串口 RX 以完整 ABCD 組發布 `/chassis/wheel_sample`：來源接收時間、連線 session、序號、原始四輪值。這不是 STM32 硬體取樣時間。odom、定位與 fusion 使用同一組資料，拒絕過期、重播、缺輪與非有限值。保留原始 `/stm32_data`、串口健康訊息及停車證據要求。
- 人流流程把短暫資料等待（最多 8 次）與真正定位暫停（最多 2 次）分開；不延長原有單次等待及整輪期限。快取同一輪 10 ms 內的有效 TF，仍核對原始時間戳；僅閒置業務決策限制為 10 Hz，不降低運動安檢頻率。
- costmap 的障礙與射線最小距離改為 0.15 m。Collision Monitor 增加 `NearFootprintStop`：base_footprint 下 x ±0.30 m、y ±0.24 m，至少 2 點；fusion 保持停止 0.75 s 避免狀態閃爍立刻再加速。保留原 Approach、車體 footprint、1 m inflation 與雷達遮罩。
- AMCL 健康、雷達與里程計新鮮，且只有地圖品質輕微下降時，既有導航最多繼續 6 s、累積 0.60 m、累積 1 rad，任一達限即退出；平移向量最高 0.20 m/s。未知/嚴重失配、可信度異常、資料過期、車體故障仍停止。只有真正恢復的地圖證據可重置額度；期間禁止派出新任務，服務到站亦須等待恢復才交接。仍使用 Nav2 現有規劃與動態避障，沒有盲行或新增 map→odom 發布者。
- 將初始位置重送移到觀測就緒檢查前，避免 AMCL 錯過第一筆種子後，與等待 map→odom 的導航啟動互相阻擋。
- URDF 四個輪子的顯示半徑改為 0.040 m（直徑 8 cm）；所有 joint origin、輪軸高度、雷達高度不變。

## 保留設定

- STM32 韌體、MPU6050、馬達控制、里程計 `.000389`、`yaw_ratio`、運動換算使用的既有半徑/幾何與補償均未調整。不能因 URDF 顯示修正就把目前已校過的換算半徑直接減半。
- 地圖、服務點、待機點、電源模型、API/前端契約不變；真實低電量返航觸發仍停用。
- 核心維持 `6.8.0-1060-raspi`；不更新 Hailo、相機、ROS 或系統相依套件。

## 驗證與後續實測

回歸測試涵蓋完整組資料、重播/時間戳拒絕、串口重連、停車證據、限速續行距離/轉角、短暫資料等待與近距離停止保持。
候選版完整回歸結果為 **704 passed**。實際單容器啟動、AMCL 冷啟動、定位就緒及乾淨退出均通過；假串口重連兩次連線、全零輸出與 odom/定位同組資料亦通過。
原生測試使用 ROS domain 231、localhost 與假串口，正式節點停止；驗證 RX→odom/定位接收到相同 session/序號、USB 假重連、全零輸出，以及實際 navigation launch、AMCL 自行建立 map→odom、定位管理器驗證就緒與容器乾淨退出。測試的 pose cache 使用獨立暫存路徑。

這些測試不能取代實車驗收。下次應觀察相同路線的 CPU、controller 超時、scan/feedback 延遲、VM/RViz 弱網、人流完整自旋，以及右前椅腳的低速停止距離。不得由隔離靜態測試宣稱實車 CPU 已下降特定百分比，亦不保證所有椅腳都能偵測。

錄製指令仍為：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

全部既有 topics 保留，新增 `/chassis/wheel_sample`；不必手動編寫另一份清單。
診斷比較仍可使用 `use_composition:=false nav_transport_profile:=async`。
