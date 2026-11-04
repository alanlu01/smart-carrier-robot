# 2026-11-03 下午 start_nav 唯讀分析

本次未更改車載檔案、未重啟節點、未發布初始位置或運動指令、未回放 rosbag。

## 資料範圍

- 車載版本：29de853d9c9a9eb7b2811888820858e73f8f183c；工作目錄乾淨。
- 本次錄製：`/home/robot/test_logs/20261103_151013_full_system_mFxsQA`。
- rosbag 約 429.13 秒，125,502 筆；正常完成（bag exit 0）。
- start_nav：15:09:51 與 15:16:20；此報告不混用中午測試及 domain 231 靜態驗證。
- 使用者確認本次未開 VM、未手動給初始位置。對複合問題的「是」不作為精確重開機時間證據；實際系統 boot time 為 12:26:16。

## 已確認的第一輪失敗鏈

1. 定位管理器 15:09:59 記錄 `same_boot_manual_required`，明確等待人工初始位置。
2. 開機識別與定位快取相同；快取保存時間 13:20:05，距啟動約 109.9 分鐘，超過 `pose_cache_max_age_sec=3600`。
3. 本次 `/initialpose=0`、`/amcl_pose=0`。TF 有 odom→base_footprint 8,481 筆，沒有 map→odom。
4. AMCL 已 active，但持續提示未給初始位置。planner_server 啟用全域代價地圖時等待 map TF。
5. 15:11:10 全域代價地圖啟用逾時；lifecycle_manager_navigation 回報 bringup aborted。
6. bt_navigator、behavior_server、collision_monitor 留在 inactive；Delivery 因 bt_navigator inactive 等待並逾時，保留訂單。

結論：第一輪不是 start_nav 完全沒啟動，而是位置初始化缺失，使 Nav2 啟用中斷。車輛擺回原點不會自動通知軟體。此同次開機保護由 0b987ccc（9/15）引入，並非本次更新新增；不可為便利直接取消保護或無條件播送原點。

## 第二輪另有服務傳輸問題

- 15:16:27 map_server 成功讀入 3F_NEW 地圖（827×1070，0.05 m/cell）。
- 15:16:28 `failed to send response to /map_server/change_state (timeout)`。
- 定位 lifecycle manager 停在 Configuring map_server；健康資料顯示 map_server inactive、AMCL unconfigured。
- 15:17:36 planner_server 再次因無 map TF 啟用逾時。
- 最後關閉時定位 lifecycle manager 未能正常結束，被 launch 升級到 SIGKILL（-9）。第一輪所有程序正常結束；未看到本次 SIGSEGV。

第二輪不能只歸因於初始位置。上一版新增獨立程序及 async DDS 設定，服務回覆傳輸回歸是待驗證假說，尚無對照實測足以斷言根因。地圖檔案存在且成功讀取，並非地圖路徑錯誤。

## 其他觀察

- 串口 RX/TX 全程同一 session、open_count=1、failures=0，未見中午那種 USB 斷線／錯誤洪流。
- 15:15:41 與重啟附近 15:16:24 仍有四輪回授約 0.82–0.83 秒逾時，自行恢復；不能據此宣稱回授完全正常。
- INA 三槽判定維持 empty/full/empty，供電狀態 healthy；無本次供電失效證據。
- 語意資訊持續有資料，但啟動及重啟附近有逾時、限制車速及輸出零速紀錄；不是第一輪 map TF 缺失的原因。
- CPU 83 個區間四核心總體平均忙碌約 61.0%，最高約 99.4%；最高溫 60.6°C、可用 RAM 最低約 5.28 GiB。啟停有調度壓力，未證明 CPU 是服務逾時唯一原因。
- 磁碟約使用 92%，僅餘 4.9 GB。建議匯出後整理舊錄製，本次未刪資料。
- 本次沒有實際導航、全域路徑或任務結果，因此不能驗證導航、人流分析與二次定位功能。

## 對近期更新的判斷與後續建議（尚未修改）

1. 將初始位置建立與整套 Nav2 ready 分成階段：AMCL/map/感測就緒即可建立定位，導航可用後才放行配送及運動。明確顯示是等待人工位置還是服務故障。
2. 為「已人工確認擺回原點但沒有 VM」提供明確的一次性初始化入口，保留同次開機不能擅自假定原點的保護；不可自動清快取或無條件延長信任。
3. 針對 map_server/change_state 回覆丟失，隔離域對照現行／async profile 與獨立程序；加入有界啟動失敗退出或安全重試，避免永久半啟動。不可未核對狀態就重複 change_state。
4. 補真正無 map→odom、由 AMCL 接收 seed 建立 TF 的整合測試，再測同次開機快取過期、服務回覆逾時、關閉後重開。上一版入口測試使用預先存在的靜態 map→odom，未覆蓋本次 bootstrap；先前通過不等於本次流程已驗證。

目前先維持唯讀，沒有部署、回退、修改或推送。
