# 2026-10-07：任務暫態重試與定位救援

## 六項變更

1. 明確 TF_ERROR（Jazzy controller=102 / planner=202，或明確 transform
   錯誤文字）保留原任務；最多重送兩次，每次等新鮮 scan/odom 與當下 TF
   連續可用 1 秒，最長 15 秒。定位管理器正在恢復時仍走原有等待定位流程，
   不用資料逾時強行啟動。真正的無路、碰撞、取消不套用 TF 重試。
2. scan_filtered 接收／來源延遲診斷最多每 5 秒一次，保留原始 topic 與安全
   門檻。APT 下載改為 03:00–03:05、安裝改為 04:00–04:05。
3. 結果提示區分 done / failed / cancelled / released。ACK 增加 ack_stage、
   cloud_confirmed、terminal_status，不改現有 event_id/task_id 或結果狀態。
4. 配送啟動用 request_id 向橋接器確認任務；先對帳及完成一次 claim 查詢，
   有任務則收集封存批次並排程，不需湊滿三筆。只有明確 empty 才發待機目標。
   10 秒還無法確認會提示並原地等網路，期間繼續收單／取消，非阻塞等待。
5. 關閉 ±20° 小擺頭。停車並短暫 nomotion 更新 4 秒，再進全域粒子重置與
   360° 自旋；若已有健康樣本正在收斂，保留有限延長時間讓六筆確認完成。
   不降低 covariance／地圖吻合度門檻，也不更動既有自旋回正與停妥檢查。
6. 第一次全域定位失敗後至多一次左右橫移。距離 0.30 m、速度上限
   0.25 m/s，以初始 odom 座標系計量；車頭偏差 >5°、縱向偏差 >5 cm、
   反向位移 >3 cm、總距離超限、8 秒動作逾時均停止。停妥後第二次全域
   定位仍不可信才請人工介入，不進行第三次橫移或任意巡走。

## 橫移防護與限制

- 僅 post_global、Spin 已完成且實際車輪／底盤命令停止時允許。
- raw/filtered scan 來源及接收 <=0.30 秒，odom <=0.25 秒，完整四輪組
  <=0.30 秒；失效／實體取還 motion_inhibited 即停。用 local odom TF，
  不依賴當時不可信的 map 位姿。雷達透過真實 base_footprint TF 轉換，
  包含現有反裝雷達的 pi yaw 與偏移，不把左右直接對應雷達角度。
- 使用完整 0.44×0.32 m footprint，加 8 cm 周界餘裕；移動中再保留
  7 cm 煞停餘裕。檢查左右完整掃掠區域，NaN/Inf/大量缺波不算暢通。
  改底盤尺寸時必須同步 lateral_footprint_half_x/y；不能靠縮遮罩解鎖。
- 命令只送 `/cmd_vel_nav` → 平滑器 → 碰撞監控 → 語意融合 → 底盤，
  不直接繞過安全層。fusion 另外限制只可橫移、上限 0.25 m/s。
- fusion 短效 token 許可 0.25 秒，許可過期鎖住零速；延遲心跳不得重新
  開啟移動。需回傳 matching token 的 guard ACK 後才能動。停車期間許可
  permit=false，確認新鮮回授及實際底盤零速 1 秒後才解除，允許第二次自旋。
- 若管理器崩潰導致許可鎖住，請先確保車輛停妥，再重啟 start_ai/fusion
  與 start_nav；不要直接繞過許可或向 chassis_cmd_vel 發速度。
- 紙箱／玻璃等雷達無法可靠偵測的物體仍是限制，不能保證所有材質安全。
  嚴格防護拒絕橫移會請人工介入，不以空曠場地為由放寬未知區域。
- 本版只有靜態／模擬回歸驗證；實際橫移、停距、車頭角度需有人監看測試。

## ACK 與網頁隊友（無新增強制 migration）

既有 HTTP endpoints、結果 enum、進度 state 不變。failed 的 progress_message
改成「任務失敗，結果正在同步雲端」，不再錯誤顯示操作成功。
網頁請勿只看到 result_pending/result_acked 就顯示服務成功，仍以後端的
done/failed/cancelled 結果為準。此原則延續先前對接清單，無新 DB 欄位要求。

ROS `/smart_carrier/task_result_ack` 新欄位：

- `ack_stage=local_durable`：車端 outbox 已持久保存，可繼續下一任務，非雲端接受。
- `ack_stage=cloud_confirmed`：HTTP 完成結果真正已被後端接受。
- `ack_stage=cloud_reconciled`：任務已是不可重送的終態／404 對帳，本機結清，
  不表示本次完成結果 POST 成功。

新增 `/smart_carrier/result_sync_state` 同步以上階段供量化；時間差應稱為
「本機持久保存延遲」及「結果至雲端確認延遲」，不要混稱 ACK 回傳延遲。
原先錄製項目全部保留，另增 dispatch_sync_request/state、result_sync_state、
recovery_motion_lease/guard。仍用 `bash .../tools/record_full_system.sh` 錄製。

## 更新排程維運

`tools/systemd/*/robot-night-window.conf` 部署至 `/etc/systemd/system/` 下
對應 timer.d。只重新載入與啟動 timer，不停止正在安裝的 apt 服務。
Persistent=false 避免白天開機補跑；若機器從未凌晨開機，需於非測試時段
人工執行安全更新或定期留機通電過夜，不能把移排程視為永久不用更新。
原有 Hailo 手動安裝版本／相機驅動與保留設定不做套件升降級。

## 下一次監看驗證

部署前於樹莓派獨立測試目錄執行 `tools/verify_revision.sh`：201 項功能
回歸通過（不啟動 ROS 節點、不移動硬體）。不含專案既有的全域 lint/
PEP257 檢查；靜態測試通過不代表實體停距或重新定位成功率已實測驗收。

1. 預先下 1 筆及 3 筆，啟動配送：先收單／排程，不先送待機目標。
2. 開始導航後短暫斷網：任務保留、恢復後對帳；結果 ACK 可分辨兩階段。
3. 讓 TF 暫態出現：最多兩次重試，不把無效路徑當 TF；取消優先且可終止。
4. 開闊且遠離玻璃處搬動車輛，確認不再小擺頭；觀察 360° 回正角與恢復時間。
5. 首次全域失敗時監看安全橫移：距離 <=30 cm、峰值 <=0.25 m/s；途中
   人員進入側邊要停止。雷達／輪速／fusion 中斷時應零速，不可自行復走。
6. 取還中不可橫移；第二次失敗必須停車要求人工。一般訂單／停靠仍需原有精度。
