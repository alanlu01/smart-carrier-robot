# 2026-10-30 待機定位、二次安檢與語意配對修正

## 本次交付

1. 待機地圖低分：涵蓋 critical/degraded（低於 .30），10 秒且至少 5 個獨立觀測；分數在 .20 附近波動不再清掉累計。已啟動的派發暫停須有新鮮、足夠方位、低近距遮擋、健康 AMCL 及恢復分數 .35 才解除。缺資料清掉時間證據，但保留暫停鎖定，不拿缺資料當恢復。
2. 二次定位旋轉：維持原始／過濾雷達分別驗證，單張可直接證明安全時保留原判斷；單張盲區則可用各自最多 5 張短時 odom 對齊證據補充。沿用 `spin_evidence` 的 .30 秒最新資料、.50 秒來源／.45 秒接收歷史、3 cm 位移限制、動態障礙物餘裕、至少 3 個不同 stamp、有效比例 .75、最大盲區 15°。目前或近期近障礙優先；車內遮罩回波是未知，並不當作自由空間。新增盲區起始方位及兩路報告。
3. 語意距離歷史：由 class ID 共用改為同類別中角度／框寬相近且唯一匹配的短期目標；同幀不共用同一軌跡，歧義則建立新軌跡，最多 64 個、閒置 .5 秒過期。每個軌跡仍保留最近 5 筆最小距離與既有 .5 秒清空保持、.6 m 停止／1.5 m 半速門檻。這是輕量關聯，不是人物身分追蹤或保證辨識正確。
4. `/semantic/pipeline_health` 原有 1 Hz 訊息附加 `association` 與 `last_stop_association`：來源 sequence／stamp、scan stamp、時間差、類別／信心值／角度／框寬、選定雷達角度及距離、近距群支持 beam 數、軌跡號與保持距離。每次診斷至多 16 個目標，最後煞停保留可追查快照。沿用同一錄製 topic，沒有新增高頻診斷 topic。
5. 人流停靠前往訊息等嘗試次數／結果更新後統一印一次；Nav2 transcript 的背景 writer 重用檔案 handle，仍逐批 flush，保留 stdout/stderr/exit 記錄及 2 MiB 有界 queue。

## 明確保留與限制

- 地圖、座標、導航速度／頻率、車內遮罩、15° 盲區、側移距離／速度／停止保護、6 筆定位驗證、電源模型及停用的真實返航設定均不更動。
- 語意仍採原本角窗中第 2 近雷達距離。新增的近距群支持數先用於診斷，尚未改成群集距離授權。沒有本次相機影像，不能宣稱已解決模型誤判或遠人配到近桌椅；需要補影像的受控測試。
- 持續無回波不是自由空間證據。10/30 兩段固定盲區／有效比例不足的場景，這次不承諾必定能啟動第二圈；新增機制主要針對短暫、互補且有實測回波的缺口。
- 派發暫停對短暫缺資料採安全鎖定；若持續無法取得可信分數，仍需資料恢復或進入既有定位恢復流程。
- 本次沒有提高速度、FPS，或宣稱 CPU 釋放多少百分比；後續須同路線 A/B。

## Nav2 關閉崩潰：已重現，尚未修復

在 localhost、ROS domain 231 使用假 TF、無導航目標／無底盤命令，直接啟動現有 Nav2 套件：

- 訊號關閉重現子容器 `exit code -6`／`Magick ... SIGSEGV` 與交錯 lifecycle 清理。
- 「先呼叫 manage_nodes SHUTDOWN」試驗等待回覆逾時，之後訊號關閉仍會重現。已測試短／較長等待與服務 QoS 對齊，沒有證據可稱其修復。
- 官方 [Nav2 1.3.12 lifecycle_manager.cpp](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_lifecycle_manager/src/lifecycle_manager.cpp) 定義 manage_nodes 服務、關閉與 pre-shutdown 流程，僅作介面核對；不是目前已取得崩潰 stack 的替代品。
- **不部署未通過驗證的自動關閉工具，不更換 Nav2／核心／驅動，不改動現有啟停介面。** 保留 `tools/diagnose_navigation_shutdown.py` 供隔離重現；此工具輸出 `child_abort=true` 表示故障仍存在，程式完成並不等於修復。不可用此結果聲稱正常關閉測試通過。
- 本次觀察的故障在人工結束階段；尚未證明與任務運行中定位問題有同一根因。

## 驗證

- `tools/verify_revision.sh`：639 項回歸通過。包括原始及過濾兩路真實 manager 安檢方法、短時互補盲區、固定盲區、近障礙、資料過期、odom 跳動、持續低分與暫時缺資料派發鎖定、目標距離分離。
- 延用既有 lint 排除：copyright、flake8、PEP257，不能說完整 ament lint 通過。
- localhost/domain231 真實 ROS：manager 建構與狀態序列化、原生 logger 分支、SIGINT／SIGTERM live-context 收尾、launch 原始輸出擷取、fusion 近距煞停與新增診斷均通過。未驅動實車。
- `tools/verify_20261030_stationary.py` 離線讀本次 bag：首次異常 11:18:19.090、候選規則確認 11:18:29.682，約 **10.592 秒**、10 個獨立觀測；相較原實測狀態 11:19:25 才進 SUSPECT。此為**離線反事實比較**，不是新版本實車偵測時間。
- Nav2 關閉比較為**已知失敗實驗**，不列入通過項目；部署紀錄保存完整輸出。

## 部署與測試

- 備份：`/home/kj0921/backups/stationary_fusion_20261030_nPLYo2/dev_ws_src.tar.gz`。
- 驗證紀錄：`/home/kj0921/test_logs/validation_20261030_stationary_fusion_gAHb55`。
- 下次依原本方式啟動即可；錄製指令不變：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

優先測：待機抬車後的觸發時間、有人短暫經過的誤觸發、座椅區第二輪是否受阻且原因正確、無近人時的起步與真近人煞停。若語意誤停重現，需同步保留相機影像才能分辨模型及深度關聯；本次沒有偷偷加入相機影像錄製或改動原錄製清單。

無網頁或 API migration。
