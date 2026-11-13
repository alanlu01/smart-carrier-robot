# 2026-11-13：停靠對正、橫移與短暫資料中斷

## 對應測試問題

11/13 09:49～10:07 記錄顯示：原地對正過修正、AMCL 座標修正使 Native 與配送層 XY 判定不同、橫移只走約 19 cm 卻標成完成、低頻四輪完整群組漏收，以及單一 map_server 狀態回覆過期引發額外定位恢復。

## 行為變更

- `StationAlign` 使用 C++ `StationAlignment`。只輸出旋轉，不因 map XY 修正轉而平移；沿用 Nav2 原生 footprint 碰撞模擬與既有下游速度/碰撞限制。以回報角速度預留煞停尾段，反向修正前等待低角速度 0.75 秒，一次 action 最多三段旋轉。一般導航與平移 controller 不變。
- 對正 Native checker 改為 stateful `StoppedGoalChecker`：起始 XY 5 cm、yaw 0.06 rad、停止速度平移 0.02 m/s／旋轉 0.03 rad/s。配送層仍另外檢查原始 odom 錨點 5 cm、旋轉掃掠空間與停妥後最終目標，不把 AMCL map 平移當成車身真的平移，也不只靠 Native success 宣告停靠完成。
- 單次對正期限按角差給 12～22 秒（角度/0.2 + 6 秒）；既有最多兩次對正及整段總期限不變。低頻回授可能使 Native 完成時仍有實體尾段，外層停妥與姿態重驗仍是必要條件。
- 橫移改連續減速，消除「動量門檻一到就直接發零」的早停回歸。目標仍 0.30 m，停止後至少 0.27 m 才視為有效；不足時在同一原始錨點、期限與租約內，重新檢查通道及感測、確認停穩後最多續行一次。保留 0.33 m、橫偏 0.05 m 與原角度上限，未達距離不能回報完成。
- 四輪群組首次送出後，約 0.1 秒間隔最多重送兩次，且原始來源年齡不得超過 0.65 秒。timestamp、sequence、session 完全不變；消費端重複資料不刷新年齡、不積分、不累積獨立停車證據。0.8 秒 watchdog 不變。
- 既有 serial I/O、底盤及 localization 診斷增加來源/送出/接收群組序號、去重與跳號資訊，供下次區分 UART 停止、relay 漏送和訂閱端漏收，不新增高頻錄影/雷達訂閱。
- 僅當 map_server 最後回覆 active、其他 lifecycle 回覆正常，且 AMCL、odom、獨立串口回授均新鮮時，允許 map_server 查詢年齡 3～4.5 秒的有限寬限。明確 inactive、其他節點 stale、缺資料或超過4.5秒仍撤銷 ready；action RESET 驗證仍使用嚴格3秒條件。每端最多一個查詢，單次回覆等待由0.75改1.25秒。
- 保存最近2秒內、健康定位與地圖吻合證據下的 map→odom 參考。缺參考與缺TF分別顯示 reference_missing／transform_missing，不誤報 position_changed，也不消耗8秒姿態驗證預算；總等待上限仍30秒，缺參考不能只靠高地圖分數恢復。
- 人流流程對明確的 pipeline_recheck 暫停保留有限續行機會，仍須恢復 LOCALIZED、健康資料與停妥才可重送。全域/人工定位仍清除舊旋轉證據，既有任務優先與等待期限不變。
- 終端顯示「🧭 車頭對正：±角度」，分清「橫移完成後第二輪」與「橫移不可用」，沿用既有節流。

## 驗證與實測限制

候選版已完成：

- 四套功能回歸 892 項通過；不包含原有 copyright／flake8／PEP257 格式檢查。
- C++ speed/alignment envelope 2 項通過，涵蓋 2 Hz 回授、200 ms 延遲、1～1.5 倍軟體反應及正反向修正。
- 原生 Nav2：8 條普通/進出站路徑、取消後接單、錯誤策略禁止動作、RESET/STARTUP 後再導航；AMCL lifecycle 保持。
- 原生對正：±1.92 rad 與 ±0.20 rad，動態 map→odom 平移修正每次14cm，停止後角差≤0.06 rad，無實體平移。Native 完成時仍可能有尾段，測試保留外層停妥重驗。
- 真正 ROS + PTY：診斷程序停頓3秒、control relay停頓2秒、虛擬USB重連，odom/localization接收同來源群組，無實車指令。
- 真正 DeliveryNavigator callback負載與遠端障礙，缺資料期限、取消排空、重啟與待機呼叫通過。
- 正式啟動入口軟體測試：不提供外部map TF仍可AMCL seed及驗證就緒；0 goals、0 nonzero commands，所有子程序正常退出。

驗證資料位於 `/home/kj0921/validation_20261113_navigation`，所有 ROS 測試 Domain231/LOCALHOST，沒有對實車發布速度。軟體模型不能取代實車的輪胎滑動、指南針量測或負載變化，對正及有效橫移距離仍需下一次場地測試驗收。

下次重啟 `start_all`、`start_nav`、`start_delivery` 後觀察：對正停妥角度、橫移至少27cm且不超33cm、第二圈定位觸發、人流資料暫停後續行、wheel_delivery 序號/跳號與 map_server response_grace 診斷。既有錄製主題不減少，無須前端/API migration。

普通導航速度、Twirling、地圖、站點、雷達遮罩、STM32、里程計係數、INA 模型及 kernel 均未修改。
