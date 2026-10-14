# 2026-10-14 橫移停止確認、舊安全租約交接、第二輪拒絕診斷

針對10/14日誌發現的恢復流程問題。當日未錄bag，舊token造成實車無法移動
仍是最強程式推論，不宣稱已直接捕捉。修改可用隔離方法重現與測試；實車待驗證。

## 修改範圍

1. 橫移完整四輪新鮮度上限由0.30改0.65秒，仍小於底盤0.80秒watchdog。
   A/B/C/D組裝窗口仍0.25秒；雷達0.30秒、odom0.25秒、租約0.25秒不變。
2. 停車改用至少三組獨立四輪零速回授，而且首末完整組跨度至少1秒。
   raw RT允許絕對值≤1（原始回授單位，不是m/s）；單輪移動、非有限／無效RT、
   過期及odom失聯都使證據重置。不以計時器重複讀同一組凍結odom代替確認。
   定位與fusion各自直接讀STM32，fusion另檢查新鮮零速odom及上游速度安靜至少1秒。
3. 停車確認與雷達可否移動分開：看不清周圍仍須煞停，但在四輪／odom證實
   停妥後可以解除停止租約。感測失聯仍鎖定，不用逾時強制解鎖。
4. 每個manager增加owner UUID。只可接管已失聯且faulted的舊token；
   接管只允許STOP，faulted不清除，即使新owner送permit=true也不可復活移動。
   活躍舊owner、新token及舊owner延遲指令不能取代交接中的owner。
   已解除的最近64個token不允許重新arming，避免延遲舊心跳重新上鎖。
5. 舊租約交接會取消Nav2導航／Spin。fusion獨立每0.5秒確認三個action的
   cancel-all回應：成功且待取消清單已空，確認在2秒內且來自目前發布者代次。
   已知ACCEPTED／EXECUTING／CANCELING狀態仍須等終態。
   Nav2 SimpleActionServer對active goal接受取消；不能把此策略移植到任意拒絕
   取消的自訂action server。從未使用的action可能沒發status，因此不能強求
   三個topic都曾出現終態；必須補以新鮮取消確認，而不是直接當unknown閒置。
6. fusion回報解除token／owner，manager核對相同交接才接受。交接完成只進
   VERIFYING，重新驗證地圖、AMCL與既有樣本門檻，並非重啟即授權導航。
7. 正常關閉提供最多4秒的停止／解除握手；若ROS context已失效、斷電、缺
   回授或fusion沒確認，保持鎖定，待新manager完成安全交接。不保證Ctrl+C
   能在context關閉前完成握手，不把一次zero當成已停止證據。
8. 首輪失敗後安全檢查最多等2秒，避免一次回呼相位空隙就人工介入。
   橫移仍最多一次，全域仍最多兩轮。近障礙物、未知雷達區與TF異常不放行。
   二輪自旋前另保留最多2秒等待（原有1秒global settle之外）。

## 新增可觀測欄位（原有topics不減少）

- `/localization/state`：lateral_handoff、recovery_guard、second_guard_report。
- `/localization/recovery_motion_guard`：owner、faulted、producer_age_sec、
  zero_wheel_groups、disarm_blockers、handoff_action_confirmations、released_token/owner。
- `/semantic/pipeline_health`：recovery_guard_blocked；安全鎖擋住時有效速度乘數為0。
- 第二輪終止日誌區分四輪／scan接收年齡、來源時間戳、odom TF、雷達未知
  比例／缺口與自旋掃掠近障礙物。Jazzy MessageInfo實測為dict，沒有publisher_gid；
  使用圖譜代次與新鮮cancel回應，不假設不存在的欄位。

人流排程／安全掃描、導航critical預檢、速度、FPS、MPPI、地圖／座標／遮罩、
INA、電池門檻、返航啟用、1060核心與更新政策本次不改。網頁／API無migration。

## 驗證與部署

- 功能回歸398項（不包含既有copyright/flake8/PEP257檢查）。
- `tools/verify_recovery_protocol.py`：ROS domain231、僅本機、三個合成action
  server，沒有正式ROS節點、沒有導航goal、硬體或實車topic；只向記憶體記錄零速。
- 部署前備份：`/home/kj0921/backups/recovery_20261014_MQZC3F/dev_ws_src.tar.gz`。
- 隔離測試／部署紀錄：`/home/kj0921/test_logs/validation_20261014_recovery/`。

## 下次實車驗證

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

保持原全部topic。先正常導航／配送，再在靜止且有安全空間時做搬車定位測試。
觀察是否走到第二輪，以及不允許二輪的詳細理由。第二輪不是無條件必做。
不為測試製造碰撞、不在移動中拔USB或強制結束manager。
若再次卡住，保留bag並讀guard／state；不要手動清token或繞過fusion。
單獨重启導航後，如遇舊租約，須看到交接→解除→VERIFYING→LOCALIZED才應續行。
正式實車節點未由此次部署啟動。
