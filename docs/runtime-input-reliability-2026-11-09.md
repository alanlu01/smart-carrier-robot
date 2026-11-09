# 2026-11-09 樹莓派輸入與啟動可靠性修正

基準 91eeb9b。針對 11/5 定位啟動提示、RX→TX 舊快照停車，以及 11/9 控制試驗結果處理。
STM32 由使用者另行修改；本提交不調整速度尺度、0.000389、yaw_ratio、導航速度、定位／回授門檻、BT 或服務座標。

## 定位狀態與初始位置

- 定位管理器建立後立即以原 TRANSIENT_LOCAL QoS 發布 ready=false，即使從未定位成功，Delivery 或晚加入訂閱者也有明確初始狀態。
- /localization/state 摘要與詳細訊息新增 initial_pose_source、startup_initial_pose，清楚記錄實際選用的初始來源與座標。
- 同次開機沿用可信位置時顯示簡短提示。預設 startup_pose_mode=auto，原有快取、地圖身分、新開機原點與缺乏可信位置時等待人工定位的保護不變。
- 已確認實車回到設定位置時，可明確執行 `start_nav startup_pose_mode:=configured`。它使用設定檔 initial_pose.x/y/yaw（目前為原點），不是推測車輛位置。
- configured 會清除舊可信位置的可重用資格，之後仍需正常 AMCL／地圖驗證，只有定位成功才重新保存可信位置；auto_initialize=false 仍不自動發布初始位置。

## 控制回授與完整診斷分工

- 新增 /chassis/serial_feedback，BEST_EFFORT、VOLATILE、小型最新值，約 20 Hz。
- 由實際 RX I/O 程序每次 poll 後擷取 heartbeat、連線／session、完整四輪來源時間、healthy 與停車證據，經有界非阻塞 mailbox 傳遞。
- ROS relay 優先發布該快照，TX 改用此通道。原 /chassis/serial_rx_health 仍保留完整診斷，避免較大、較低頻快照作為 TX 的唯一最新證據。
- sample_ros_stamp_ns 代表 I/O 擷取時間；complete_ros_stamp_ns 始終是實際完整四輪組時間，不能因 heartbeat 更新把舊輪速變新。
- 不在 ROS relay 重新加蓋來源時間。不合 session、資料逾時、缺乏停車證據或沒有新命令時，維持歸零與禁止舊命令重播。
- 原 .35 秒回授快照、.65 秒四輪證據、.25 秒命令期限與 1 秒配送租約均未放寬。
- RX、TX 要使用同一套部署；舊 RX 不提供新通道時，新 TX 會維持禁止非零輸出。正式重開 start_all 會同時載入兩者。

## 導航等待診斷

/navigation/pipeline_health 保持 ready／unavailable 契約，增加 reason 與 inactive／missing／stale 節點清單。
已回覆 inactive、尚無回覆、先前 active 但回覆過期，分別顯示 lifecycle_inactive、awaiting_responses、response_stale。
定位等待文字區分等待啟用與等待回覆更新；不以舊 active 回覆當成仍可行駛，也不因此重啟 Nav2 或放寬三秒新鮮度。

## 閒置減負

人流未有待配對 scan 時取消 50 Hz TF 重試 timer；遇到新鮮 scan 的未來時間外插才 reset。
成功、逾時或退出觀測時停止重試。仍只查原始 scan stamp，沒有 latest-TF 放行或延長安全期限。
其他觀測、安全心跳、配送原生 BasicNavigator executor 處理維持。

## 驗證

- 四套件功能回歸：760 項通過，不含既有 ament 版權／flake8／PEP257 格式檢查。
- verify_runtime_inputs_ros：真實晚加入 ready 訂閱、configured 來源與實際 ROS timer 取消／恢復；自動定位及動作停用，快取使用臨時路徑。
- verify_serial_hotplug：假 PTY、零速、熱插拔、診斷程序暫停、控制發布阻塞、完整四輪原子傳遞；新增兩秒停止舊 health 發布，確認 TX 仍以獨立小型回授維持就緒。
- verify_delivery_client_ros：單執行緒客戶端、高頻輸入、服務／待機各 30 秒、訂單優先取消及真實 1 秒配送租約失聯取消。
- ROS 測試限定 localhost/domain231，無真實串口、馬達與導航目標；隔離測試不代表已量得整車 CPU 改善或實車服務成功率。

候選整合結果：兩秒停止舊 health 發布未造成 TX 失去就緒；正常控制回授接收最大來源年齡約 0.0524 秒。
單執行緒服務／待機各 30 秒通過，正常導航租約最大間隔約 0.2023 秒，無誤取消，刻意失聯仍可取消。

## 操作與錄製

一般仍使用 start_all、start_nav、start_delivery；未改一般啟動預設。
重新搬到原點但沒有重開 Pi 時，依實車位置選擇手動 initialpose，或上述明確 configured 選項。

錄製指令不變：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

原 topic 清單不減少，只新增 /chassis/serial_feedback 及相應 QoS。
網頁、API 契約與資料庫無修改，隊友無須 migration。
