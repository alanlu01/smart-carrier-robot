# 2026-10-08 定位自救與感測管線診斷

## 修改範圍

1. 橫移安全檢查啟動時讀取 `my_lidar.launch.py` 同一份已安裝的
   `config/my_laser_filter.yaml`，驗證為 base_footprint 非反向車體盒。
   遮罩不得超出既有橫移 footprint。無法驗證時保留未遮罩的保守原始檢查。
   原始與過濾雷達仍須同時通過；車體內回波視為「未知」，不是自由空間。
   外部障礙、有效比例至少 95%、連續未知缺口最多 4° 均保留。
   遮罩不因地圖旋轉而旋轉，依實際雷達 TF（目前 yaw=pi）投影。
   傾斜的掃描平面不以錯誤平面近似，直接拒絕橫移。
2. post_global 原本 30 秒上限後，只有新鮮資料、停妥、地圖吻合、
   位置穩定且健康樣本未足／covariance 持續改善時，可延至總共 40 秒。
   不是額外 40 秒。六筆獨立健康確認不變，不因寬限直接放行。
   寬限到期仍沿用單次安全橫移／第二輪全域／人工介入流程。
3. 新增低頻健康資訊內的耗時統計。未調整 watchdog、TF、DDS 設定或 QoS。
   這是根因追查工具，不能宣稱已解決網路不良時的管線停頓。

不更動地圖、服務點、充電模型、速度、旋轉回正參數；自動回家與人流待機仍停用。
修改既有遮罩檔後須重啟 start_all/start_nav，勿只在執行中改 filter 參數，
否則 manager 的啟動快照與 filter 不一致。使用其他 filter 設定時須同步指定
localization_manager 的 `self_mask_config_file`。

## 部署前驗證

- 樹莓派獨立副本：323 項靜態／模擬功能回歸通過（不含既有格式 lint）。
- 今日 11:13:35.672 的實際 bag 掃描離線驗證：raw 左側由拒絕改為通過，
  1 筆車內回波改列未知，有效比例 99.81%、缺口 0.20°。
  raw／filtered 右側仍拒絕：有效比例 85.36%、未知缺口 11.175°。
  使用錄製 TF，包含雷達高度 0.425 m 與反裝 yaw=pi。
- 模擬驗證 2/6 可在寬限內完成、40 秒不再寬限、移動／過期／關鍵品質不足
  不給額外寬限、外部障礙不會被遮罩、里程計 watchdog／積分不因診斷改變。
- 以上不是實車移動驗證；尚須按下列清單進行下一次實測。

## 下次錄製

樹莓派新終端執行（保留此前所有 topic、CPU／Wi-Fi／核心紀錄）：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

看到 recorder 已訂閱後開始測試；結束在同一終端按 Ctrl+C，等待關檔。
不要 Ctrl+Z，不要把錄製放背景。無需錄製原始影像或新增 topic。

## 新欄位解讀

- `/chassis/feedback_health.wall_durations`：`stm32_callback`、
  `odom_callback`、`tf_publish`、`odom_publish`、`health_publish` 的上次、
  最大、平均 wall-time、超過 50ms 次數、最大值距今多久。
  publish 耗時含排程／系統暫停，**不是 DDS 阻塞的單獨證明**。
  健康發布本身的耗時會在下一筆健康訊息顯示。
- `/localization/state.wall_durations.state_publish`：定位狀態發布耗時。
- `/localization/state.scan_pair_timing`：相同 header stamp 的
  filtered callback 時間減 raw callback 時間；可為負，因訂閱排程順序不同。
  包含 filter、傳輸、兩訂閱 callback 排程，**不是純 filter CPU 處理時間**。
  最多留 128 個未配對時間戳，記錄未配對淘汰數。
- 兩個健康訊息的 `sample_ros_stamp_ns` 是產生快照的 ROS 時間，
  可與 bag 接收時間比較，但須考量跨機時鐘誤差。
- `/localization/state.lateral_path_reports`：左右各 raw／filtered 的拒絕原因、
  遮罩回波數、有效比例、未知角度，或外部障礙座標。失敗時另寫一次日誌。

## 實車驗證（本次部署不啟動馬達）

1. 原地健康定位、正常租還與待機不受影響。
2. 開闊安全區搬車，觀察第一次全域結果；若仍不健康才需橫移。
   若已收斂，不應為測橫移而刻意放寬門檻。
3. 保留障礙／未知區域拒絕與橫移途中遇障輸出零速度。
4. 網路良好／訊號差、VM RViz 開／關分別記錄，操作時記下時間，
   優先採自然的訊號差；若要關熱點，車輛先停妥且有現場人員看顧。
5. 比較 raw bag 與節點 receipt gap、source age、publish wall-time、
   timer gap、scan pair skew、CPU／Wi-Fi，才決定下一階段 DDS／executor 改法。
