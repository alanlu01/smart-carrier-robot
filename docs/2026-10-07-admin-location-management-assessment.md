# 管理後台服務點／新地圖座標管理：唯讀評估

本次只修改車載預設停用功能，**未修改 web、雲端 API 或正式資料庫座標**。

## 權限與現況

- 已透過 GitHub API 核對三個 repo：pull=true、push=true、admin=false。
  有程式修改／推送權限；不是 GitHub repo 管理員不妨礙新增後台功能。
  GitHub 權限不等於網站管理員 session 或正式雲端 SSH／資料庫權限。
- 前端 `src/routes/admin.tsx` 的 LocationsPanel：正式模式只有 listLocations，
  add/remove 正式分支直接 return；新增、刪除按鈕限 demoMode。現在不能改
  不是多給一個前端按鈕權限就會生效。
- 後端公開 GET /api/v1/locations；admin router 有登入、訂單查詢／取消等，
  尚無 locations 的管理員 CRUD endpoints。
- 後端已有 `python -m app.location_calibration CODE X Y YAW --apply` 工具，
  未加 --apply 是預覽。repository.update_location_coordinates 可更新既有
  服務點；有 pending/in_progress 訂單時拒絕修改。執行需正式服務環境的
  資料庫帳號／設定，不應將帳密放進前端或直接在未量測前套用。
- 前端 main 推送由現有 GitHub Pages workflow 建置部署；後端需要另外走
  雲端部署流程，repo push 本身不能證明雲端 SSH／DB 寫入權限已驗證。

## 建議下一階段（需另行確認後實作）

1. 增加受既有 require_admin_session 保護的 locations GET/POST/PATCH，及
   啟用／停用操作。只有真正的後端驗證能阻擋未授權修改，不能只隱藏按鈕。
2. 編輯 code、名稱、說明、樓層、x/y（公尺）、yaw（弧度），拒絕 NaN／Inf、
   非法代碼、重複代碼與未確認的導航座標；已有 QR／歷史訂單的 code 不宜更名。
3. 刪除採 enabled=false 的停用方式，保留歷史訂單／QR 關聯。存在待處理或
   執行中訂單時禁止座標修改／停用，回傳清楚的 409 衝突原因。
4. 新地圖需 map_id／revision 或類似版本契約；建議草稿→量測確認→發布，
   讓一批座標一起對應同一地圖，避免只改一個點就讓其他舊點繼續接單。
5. 增加審計紀錄與舊值備份、修改摘要及二次確認。修改與建單／claim 的交易
   需考慮資料庫鎖定，不能只在 UI 先檢查有無訂單後直接覆寫。
6. UI 顯示正式地點的座標／地圖版本，提供編輯、新增、停用與錯誤回饋；
   修改後重新載入，不能只改瀏覽器 demo 資料。
7. 機器人 order_payload_to_order 會優先用 API 訂單的 location.x/y/yaw，故
   未來正式後台改好後，後續新訂單可直接使用新座標。已接入車端的任務
   不應中途改目標；本機 LOCATION_DB／離線測試座標仍需另外同步。
8. 現在 locations 是「服務點」，待機點、home 與人流觀察區在車端設定。
   若也希望後台管理，需新增 point_type／專用設定接口與車端版本同步機制；
   單純加服務點 CRUD 不會自動改到這些點。

結論：可以做正式管理員座標管理；建議另開一輪前後端修改與測試，與網頁
隊友協調後再部署。本次不因「是否能改」的詢問直接變更正式資料。
