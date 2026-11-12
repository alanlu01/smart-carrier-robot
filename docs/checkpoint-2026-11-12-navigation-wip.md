# 2026-11-12 導航修正中途存檔（未部署）

此頁是關機前的歷史存檔；後續完成內容與驗證請見 [navigation-recovery-2026-11-12.md](navigation-recovery-2026-11-12.md)。

使用者要求先安全存檔、準備關機。此為 **WIP，不是可部署版本**。正式樹莓派與 origin 仍為 `ffe68492541e7d99d982c3e7067f774b25231fe0`，沒有修改正式 dev_ws、啟動正式節點或控制實車。關機前已確認 ROS domain 231 沒有殘留測試程序；正式 robot repo 乾淨。

## 已確認的修改範圍

1. 修正普通導航／進出站的 BT/action 交接及已產生 plan 卻 ACK timeout 的處理。
2. 導航 action 健康狀態、保留任務、有期限和次數上限的安全恢復。
3. 成功重新定位且導航鏈健康後，條件式重新武裝一次停靠回合；物理起轉停滯不盲目重試。
4. 短暫 TF 暫停不耗真正停靠失敗次數，保留人流選定待機點。
5. 人流待機計時對已停妥後 AMCL 緩慢修正的處理、暫停／清零／恢復提示；實際到站容許值維持。
6. 普通 MPPI 與 smoother 角加速度候選 1.2→0.8 rad/s²，Twirling 2→4；最高直行0.3、角速度0.8、5Hz不變。
7. A*僅離線／隔離比較，正式配送保留 NavFn Dijkstra；比較座位→左樓梯等已知困難路段，檢查狹縫、轉向及完整車身餘裕。未授權直接畫禁行區或移服務點。

## 已做但尚未驗收

- 新增 `navigation_policy.py`、`navigation_policy.cpp`、`smart_owned_navigate.xml`：嘗試讓一般與進出站使用同一個持續載入的 BT，使用與精確 goal 時間戳／pose 綁定的策略選擇控制器與 goal checker。未收到匹配策略時不得退回普通導航。
- `DeliveryNavigator.goToPose` 對套件自有普通／進站 BT 發布上述策略；人流Spin、回家、自訂BT保留原入口。
- 新增 `navigation_action_health.py`：草稿版有上限的導航層 lifecycle RESET／STARTUP；保留 AMCL。必須經新鮮獨立停車證據、配送導航租約false、沒有導航／Spin action執行中後才可恢復。**仍須審查與故障注入驗證，不能直接部署。**
- `NavigationTranscript` 草稿解析原生 ACK timeout／取消失敗，寫 launch 專用原子診斷檔，`NavigationHealth` 低頻讀取並附 action 狀態；新增 planner_server lifecycle觀察。
- `DeliveryNavigator` 草稿訂閱 action health 並非阻塞請求恢復；`StationNavigation` 對 native FAILED/error_code=0 加最多1.5秒等候診斷，避免把 ACK timeout直接當一般訂單失敗。
- `StandbyRetryPolicy.rearm_after_recovery` 草稿：每回合最多一次，物理起轉停滯不自動解鎖。
- 配送主迴圈草稿加 action health閘門、恢復文字及停靠重新武裝；需要整合測試。
- Python 變更語法檢查通過。**全套測試尚未執行，還有多項未實作。**

## 隔離測試結果與已知阻擋

- Pi `/home/kj0921/validation_20261112_navigation/`；只在此建置及測試。
- `baseline.log`：舊版真實 composed Nav2＋軟體底盤，8次普通／StationY／Align／Exit交接及一次取消後接進站成功，未重現當天弱網故障。不可宣稱已找出或驗證根因。
- `candidate_build.log`：最初候選 smart_station_controller（含新C++策略節點）及 smart_delivery_core 成功建置；**後續新增健康／恢復草稿尚未重新同步建置**。
- `owned.log`：候選交接驗證停在 Python `navigation_policy(...) is None`，尚未實際進行新 BT 動作。原因：symlink install 的每個 BT XML 分別連到 source，`Path(tree).resolve().parent` 與 `(share/'behavior_trees').resolve()` 仍不同。下一步應以允許的每個精確檔案 `resolve()` 比對，兼顧拒絕任意自訂路徑、保留正常非symlink安裝。接著重跑候選交接驗證。
- 尚未執行人工故障注入、原生 lifecycle恢復、弱網情境、A*比較或正式入口測試。
- 測試使用 ROS_DOMAIN_ID=231、ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST、ROS_STATIC_PEERS空值，使用軟體底盤；正式車載Domain30。沒有實車移動。

## 續作順序

1. 修正上述 XML symlink 精確允許清單，測試目標與策略不符時無速度、無普通導航fallback；驗證重複普通／進站／取消與人流Spin交接。
2. 完成並審查 action健康恢復：請求去重、RPC timeout後不再發相反命令、錯誤保持鎖定、恢復世代、獨立停車新鮮度、非配送導航不受影響。加純邏輯測試及真正Nav2故障注入測試。保留90秒任務恢復期限及手動取消可用。
3. 完成短暫 TF 暫停的補停次數處理（現在仍未改 `optional_idle_runtime.py`）。
4. 完成人流倒數對緩慢 AMCL 修正的處理與圖示提示（現在仍未改 `standby_idle_clock.py`）；保持實際停靠XY/yaw標準、真實搬車／過期感測器不得當正常計時。
5. 修改候選速度參數（現在仍未改 `my_nav2_params.yaml`）。
6. 以相同地圖/costmap比較Dijkstra與A*，保留正式Dijkstra，輸出各路徑的耗時／長度／狹縫及轉向餘裕。
7. 完成回歸、原生整合、入口bootstrap等檢查，審查diff，備份正式程式再部署、驗證安裝版本與hash，最後推GitHub。**勿直接把此WIP部署／推成完成版本。**

## 參考與本次分析

- 本機外層報告：`reports/2026-11-12-1251-readonly-review.md`，以及同名資料夾中的效能、Nav2 stderr。
- 本機外層 `tmp/review_20261112_1251.json`、`_geometry.json` 是離線解析；原始bag在Pi `~/test_logs/20261112_125133_full_system_TOmON3/rosbag`。
- 車載Jazzy Nav2 1.3.12，`/opt/ros/jazzy/include/nav2_behavior_tree/bt_action_node.hpp` 可核對原生ACK及halt處理；沒有更新任何ROS／核心／Hailo套件。
- 最新候選政策控制器／checker都需核對根據goal選到正確profile；原本正常BT不應被外部selector覆蓋成stationprofile。新ownedBT已移除其中的ControllerSelector／PlannerSelector。
