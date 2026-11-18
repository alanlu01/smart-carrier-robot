# 執行期優化與錄製模式

本版先修正四輪 I/O→ROS mailbox 在競爭時漏送完整群組的路徑。只保留最新群組，最多嘗試五次、等待不超過 0.20 秒；新資料取代舊 pending，失聯、換 session、時鐘回退或來源過期即丟棄。保持原始來源時間／序號與 0.65／0.8 秒保護門檻，不令硬體 I/O 等鎖。

`/chassis/serial_rx_health` 的 `wheel_ipc` 增加收到、成功交接、寫入失敗、重試、取代、失效及最後交接序號；既有 `/chassis/serial_io_diagnostics` 同時帶出此資料，可與下游 wheel_delivery 及輪樣本序號比對。

進出站幾何僅在一次同步 start／poll 內快取完全相同的 footprint 查詢。上限 512 筆，結束即釋放，不對位置取整、不跨週期沿用許可，不修改車身尺寸／避障門檻。新 costmap snapshot 有自己的快取，TF、資料年齡、停止與取消檢查照常。station_state 的 geometry_cache 提供累計查詢／命中數，並非 CPU 節省比例。

STM32 定期正常回授摘要減為約 30 秒，缺資料／恢復變化仍在原摘要週期提示；所有結構化健康與故障事件保留。導航速度、定位、取還／電源參數及座標不變，未加入 IMU。

## 錄製

在樹莓派終端輸入，依情境選一種：

```bash
# 日常測試：減少錄製負載
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh --daily

# 完整診斷：原有指令維持相同全部 topic
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

日常模式只省略 `/global_costmap/costmap`、`/global_costmap/costmap_updates`、`/particle_cloud`、`/bond` 的錄製，不停用系統發布。保留原始／濾後雷達、里程計、TF、地圖、local costmap、任務、取還、電源、定位／導航／通訊與效能指標。需要深入分析全域障礙格、粒子分布或生命周期時選完整模式。

兩模式都以前景執行；Ctrl+C 結束並等待關閉訊息，勿 Ctrl+Z。日常目錄為 `_daily_system_`，完整目錄仍為 `_full_system_`，各自存 recording_profile.txt。`--print-topics` 只顯示清單，不啟動 ROS、不建立錄製。

## CI

lint、無 ROS 的政策／持久化／IPC／幾何測試，以及 Jazzy 的 ROS 功能測試分開執行。ROS bridge 測試放在 API tests/ros，沒有刪掉或用 skip 掩蓋。ROS job 使用 Jazzy／Noble，套件安裝只在 CI container，不升級車載系統。既有 ament copyright／flake8／PEP257 全包規範尚未納入這次功能 job；原有 Ruff 範圍仍是獨立必跑檢查，不將功能通過表述為全部歷史 lint 均通過。

## 驗收界線

功能測試與隔離 PTY／ROS 測試不驅動實車。下次實車比較 wheel_ipc 計數／序號完整度、停頓原因、CPU、來源年齡 P95/P99；日常與完整錄製在相同路線做 A/B，不能以 bag 縮小比例推算 CPU 降幅。
