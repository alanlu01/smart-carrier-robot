# 2026-11-12 導航交接、恢復與待機計時修正

基準 `ffe6849`。本次12:51～13:11錄製中，左樓梯準備點到達後，原生BT雖已收到正確路徑，卻連續發生ComputePathToPose ACK timeout；之後新訂單和返回待機都失敗。另有已成功定位但停靠重試耗盡、已停妥後AMCL緩慢修正使人流計時清零的情況。

## 變更

- **減少BT重建。** 配送自有普通導航、進站、出站與小幅對正共用`smart_owned_navigate.xml`。C++ `DeliveryNavigationPolicy`只接受與該次目標時間戳、map frame、完整pose完全相符的策略，明確選擇FollowPath／StationX/Y／ExitX/Y／Align及對應goal checker；策略缺失或不符時等待最多2秒後停止，不退回普通控制器。普通分支保留原重規劃／有限恢復，station分支仍只有規劃與指定FollowPath，不自行加Spin／BackUp。
- **保留原生執行器。** BasicNavigator的goToPose、isTaskComplete、cancelTask完全保留；StationNavigation僅在送目標前發布策略。人流Spin、精準回家與自訂BT保持原入口；RViz一般Nav2 Goal仍可使用原普通BT。
- **辨識action故障。** 原生stderr的ACK timeout／取消失敗，以及策略匹配失敗，透過既有非同步transcript寫入launch專用原子診斷檔。NavigationHealth每秒讀取並在`/navigation/pipeline_health.action`附上狀態、故障序號、恢復世代及次數；也補上planner_server生命週期觀察。單純halt等不到結果的訊息不直接判定永久故障。
- **有限導航層恢復。** 僅對配送租約時間窗內的故障，經Delivery請求、導航租約false、沒有Navigate／Spin執行中、新鮮獨立四輪停車證據連續1秒後，才對`lifecycle_manager_navigation`執行RESET→重新確認停車→STARTUP→確認新鮮active回覆。AMCL、map及里程計不重置。最多2次／start_nav；各階段有期限，RPC逾時不重發相反命令，無法驗證或達上限則清楚要求檢查start_nav。自動恢復不適用未由配送持有的故障。
- **任務保留與停靠重新武裝。** Native FAILED但error_code=0時最多等1.5秒診斷，將握手故障分開處理，保留原90秒資料／導航恢復期限與人工取消。真正的定位／導航恢復後，每個停靠回合最多額外重新武裝一次；實際起轉停滯不盲目重試。終端區分定位等待、導航恢復與停靠次數耗盡，避免停留在過期的「等待定位穩定」。
- **短暫TF暫停不消耗停靠失敗次數。** 人流選定的待機點保持，資料恢復後仍需取消終態及停妥確認，沿用原總期限和安全暫停次數上限。
- **人流計時與微小map修正。** 已驗證停妥、odom未移動、LOCALIZED且dispatch_allowed、xy_std≤0.20m、yaw_std≤8°、map match≥0.60時，允許輔助計時最多一次接受35cm內的緩慢map修正：先暫停、等待至少3秒穩定，最多等10秒；大於8cm／0.08rad的單次跳變、真實移動、資料不健康或超出邊界皆不使用例外。只更新計時參考，不修改目標或實際停靠容許值。增加圖示式暫停／清零／恢復訊息。
- **普通導航候選調參。** MPPI和smoother角加速度1.2→0.8rad/s²，Twirling權重2→4；vx0.3m/s、vy0.15m/s、wz0.8rad/s及5Hz維持，減速度保留。
- **修正普通導航小角度尾段停滯。** 隔離測試發現兩種Twirling權重都可能在已到XY、剩約7°時輸出低於軟體底盤起轉門檻的角速度。對於已進入10cm、方向差0.10～0.35rad、新鮮odom連續停車0.5秒且完整轉向掃掠安全的普通停靠，取消並排空原action後接既有有限次數Align；不重走準備點、不放寬最終XY/yaw或宣告假到站。

## A*比較與決定

正式仍為NavFn Dijkstra（`use_astar: false`）。使用車載已安裝NavFn程式庫，對11/12錄製的3個global costmap快照，各比較「座位／座位燈／CYCU EE→左樓梯」共9對路徑。同一快照與起終點下A*約快數毫秒，但障礙餘裕沒有增加，最末段仍需進站幾何保護。以13:03:32快照為例：座位路徑約7.16m／7.18m；座位燈約14.31m／14.59m。不能以此宣稱A*可改善鑽窄路，故本次不啟用，也未畫禁行區。

這是量化OccupancyGrid反推cost值後的離線比較，有量化誤差；全身前行姿態掃掠屬保守檢查，不代表SE2規劃或實車可行性驗收。工具為`tools/navfn_compare.cpp`與`tools/compare_recorded_navfn.py`。

## 驗證

- 配送核心742項功能測試、電源／API橋接／語意108項，共850項通過；另C++速度包絡測試通過。未包含原有ament版權／flake8／PEP257格式檢查。
- 真正composed PlannerServer、ControllerServer、BTNavigator等，搭配軟體底盤，通過8段普通／進出站／對正交接、取消後進站、錯誤策略不產生路徑或位移、普通速度不被station限速污染，以及資料過期時拒絕恢復、新鮮停車後導航層RESET／STARTUP、AMCL保持active、恢復後再進站和普通導航。
- 軟體底盤含0.11rad/s角速度死區；使用真正StationNavigation的普通停靠通過小角度尾段交接。這是邊界模型，不是重新校準實車起轉門檻。
- `verify_station_actions_ros.py --loaded --remote-obstacle`驗證生產類似callback負載、遠端障礙、安全取消、資料缺失及真實待機呼叫流程。
- 正式安裝入口bootstrap在隔離domain驗證Nav2健康、AMCL初始位置與管理器就緒，0導航目標、0非零命令，子程序正常結束。
- 以本次12:55待機資料離線重算，舊計時約剩3秒被map漂移清零，新邏輯於12:56:09確認原地修正後續算，約12:56:14滿60秒；原待機目標未改。
- 舊版單純交接測試也能通過，並未在隔離環境完整重現當天弱網的底層ACK故障；此次減少交接暴露並加入有界安全恢復，不能宣稱已證明消除所有DDS／弱網問題。微S改善仍需實車rosbag驗證。

## 部署與下次測試

- 正式舊版備份：`/home/kj0921/backups/navigation_20261112_QVjWDQ`，含基準commit、原始碼及實際build/install內容。
- 隔離驗證：`/home/kj0921/validation_20261112_navigation`；測試皆localhost、Domain231、軟體底盤，未控制Domain30實車。
- 重開`start_nav`、`start_delivery`後生效。先測電機／左樓梯連續多筆、進出站後接新單、搬車恢復後回待機，以及CYCU EE完整人流倒數。普通直線比較微S及必要轉彎是否變慢。故障時觀察圖示與`navigation/pipeline_health.action`，不要只看lifecycle active。
- 原錄製topics全部保留，額外加入`/delivery/navigation_policy`；健康、station、optional狀態已包含新增診斷。
- 無網頁／API schema migration；槽位、電源模型、服務／待機座標、地圖、雷達遮罩、footprint／膨脹層、里程計係數、STM32、核心1060與Hailo套件皆維持。
