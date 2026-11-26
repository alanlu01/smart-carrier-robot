# 給網頁／API 隊友：2026-11-26 追加調整

車端已修正滿電弱電流與實體取還基準。請協助以下部分；既有下單、ACK 與結果重送介面不變。

1. **抵達不等於可取還**：`progress_state=arrived` 顯示「已抵達，準備槽位中，請暫勿取還」，不要先顯示拿取／放入指示。等車端 `waiting_action` 才顯示指定槽位與操作期限；有 `action_ready` 時以它為準。`wrong_slot` 應提示恢復錯槽，`verifying_action` 顯示確認中，不能提前顯示訂單完成。
2. **新增進度欄位需穿透後端**：車端進度新增 `action_ready: boolean`、`unverified_slots: integer[]`。bridge 會原樣保存、重送；目前本機看到的後端 `TaskProgress` schema / SQL 尚未持久化這兩欄，請補上接收、儲存、查詢與推播。若選擇新增資料庫欄位，migration 需另由你處理；本次沒有自動執行雲端 migration。
3. **感測暫停**：`action_ready=false` 時停用完成／操作引導，顯示車端 `progress_message`；恢復為 true 才開放。`arrived`、失敗、取消與終態不可沿用上一個 true。請將這兩個新增欄位當成每次狀態更新的完整當前值，不跨任務沿用。
4. **其他槽不確定的限制**：目標槽可靠時允許取還；其他槽 presence unknown 會列在 `unverified_slots`。顯示「部分槽位狀態待確認」，不要把 unknown 當 empty／可租，也不要宣稱這些槽的錯槽操作已完整驗證。目標槽不確定或電氣供電異常仍不允許完成。
5. **庫存與完成**：行充 full/ready 是充電功率推估，不是精確百分比；unknown 不可出租。完成必須以機器人實體確認後的終態為準；首次 ACK 可能只是本機持久化，不能視作雲端接受終態。

請先用相容方式加入 optional 欄位，舊車端缺欄位時保留原本依 progress_state 的流程。新增欄位上線前，至少先完成第 1 點，避免使用者在車端建立實體操作基準之前取走行充。機器人端導航七項修正不需額外導航 API。
