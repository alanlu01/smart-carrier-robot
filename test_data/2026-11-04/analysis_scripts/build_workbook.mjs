import fs from 'node:fs/promises';
import path from 'node:path';
import { Workbook, SpreadsheetFile } from '@oai/artifact-tool';

const outputDir = path.resolve('outputs/rosbag_quantitative_20261003');
const previewDir = path.resolve('tmp/quantitative_20261003/previews');
await fs.mkdir(previewDir, {recursive:true});
const d = JSON.parse(await fs.readFile(path.join(outputDir, 'analysis_data.json'), 'utf8'));
const wb = Workbook.create();
const names = ['量化摘要','測試批次','訂單明細','定位事件','感測效能','行充感測','視覺語意','來源與方法'];
const sheets = Object.fromEntries(names.map(n => [n,wb.worksheets.add(n)]));
const navy = '#203B5C', blue = '#EAF0F6', amber = '#FFF1D6';
const font = 'Arial';
const col = n => {let s=''; for(let x=n;x>0;x=Math.floor((x-1)/26)) s=String.fromCharCode(65+(x-1)%26)+s; return s;};
// Excel dates are local wall clock in UTC+08:00, not an actual UTC timestamp.
const date = s => s ? new Date(s.replace(/[+-]08:00$/, '')+'Z') : null;
const epochLocal = t => date(new Date((t+8*3600)*1000).toISOString().replace(/Z$/, '+08:00'));
function base(s, title, width=12) {
  s.showGridLines=false;
  s.getRange(`A1:${col(width)}120`).format.font={name:font,size:11,color:'#263445'};
  s.getRange(`A1:${col(width)}120`).format.verticalAlignment='center';
  s.getRange('A2').values=[[title]];
  s.getRange('A2').format.font={name:font,size:16,bold:true,color:navy};
  s.getRange(`A3:${col(Math.min(width,10))}3`).format.borders={bottom:{style:'thin',color:'#8B9AA9'}};
  s.getRange(`A1:${col(width)}120`).format.rowHeight=24;
}
function table(s, headers, rows, row=6, name='DataTable', widths=[]) {
  const end=row+rows.length;
  s.getRange(`A${row}:${col(headers.length)}${end}`).values=[headers,...rows];
  const t=s.tables.add(`A${row}:${col(headers.length)}${end}`,true,name);
  t.style='TableStyleMedium2';
  s.getRange(`A${row}:${col(headers.length)}${row}`).format={fill:navy,font:{name:font,size:11,bold:true,color:'#FFFFFF'},horizontalAlignment:'center',verticalAlignment:'center',wrapText:true,rowHeight:40};
  for(let i=0;i<headers.length;i++) s.getRange(`${col(i+1)}${row}:${col(i+1)}${end}`).format.columnWidth=widths[i]??17;
  s.getRange(`A${row+1}:${col(headers.length)}${end}`).format.rowHeight=26;
  return {first:row+1,last:end};
}
function numeric(s, range, format='0.00') {s.getRange(range).setNumberFormat(format);s.getRange(range).format.horizontalAlignment='right';}
function dtfmt(s, range) {numeric(s,range,'mm-dd hh:mm:ss.000');}
function flag(s,range,operator,formula) {s.getRange(range).conditionalFormats.add('cellIs',{operator,formula,format:{fill:amber,font:{color:'#8A4A00'}}});}
function titleNote(s, text) {s.getRange('A4').values=[[text]];s.getRange('A4').format.font={name:font,size:11,italic:true,color:'#596A7C'};}

// Detailed task records: derived durations live here, summary references them.
const orders=sheets['訂單明細'];
base(orders,'訂單與結果確認',18);
titleNote(orders,'以任務ID去重；時間為臺灣時間。取消不等同配送失敗。');
const ot=table(orders,['來源','任務短ID','任務','服務點代碼','終態','開始導航','到達服務點','取還確認開始','車端結果','確認尾段(s)','雲端確認','雲端延遲(s)','定位暫停次數','錯槽事件數','完整任務ID','首ACK時間','首ACK延遲(s)','範圍說明'],
 d.orders.map(o=>[o.source_ids,o.task_id.slice(0,8),o.type??'未錄',o.location_code??'未錄',o.result_status,date(o.first_navigating_local),date(o.arrived_local),date(o.confirm_start_local),date(o.result_local),null,date(o.cloud_ack_local),null,o.paused_localization_events,o.wrong_slot_events,o.task_id,date(o.first_ack_local),null,o.note]),6,'OrderRecords',[15,14,12,15,19,23,23,23,23,16,23,16,18,15,44,23,18,50]);
for(let r=ot.first;r<=ot.last;r++) {
 orders.getRange(`J${r}`).formulas=[[`=IF(AND(ISNUMBER(H${r}),ISNUMBER(I${r})),(I${r}-H${r})*86400,"n.a.")`]];
 orders.getRange(`L${r}`).formulas=[[`=IF(AND(ISNUMBER(I${r}),ISNUMBER(K${r})),(K${r}-I${r})*86400,"n.a.")`]];
 orders.getRange(`Q${r}`).formulas=[[`=IF(AND(ISNUMBER(I${r}),ISNUMBER(P${r})),(P${r}-I${r})*86400,"n.a.")`]];
}
for(const c of ['F','G','H','I','K','P']) dtfmt(orders,`${c}${ot.first}:${c}${ot.last}`);
for(const c of ['J','L','Q']) numeric(orders,`${c}${ot.first}:${c}${ot.last}`);
orders.freezePanes.freezeRows(6);orders.freezePanes.freezeColumns(2);
flag(orders,`L${ot.first}:L${ot.last}`,'greaterThan',10);

const sess=sheets['測試批次'];
base(sess,'逐場測試彙整',16);titleNote(sess,'9份完整bag。場次數據反映不同軟體版本與操作情境，不作單一版本驗證。');
const st=table(sess,['來源','錄製開始','錄製結束','秒數','時數','訊息數','容量(GiB)','done任務','取消任務','done雲端確認','定位區段數','曾要求人工','CPU平均(%)','CPU P95(%)','紀錄器丟訊','版本短SHA'],d.sessions.map(s=>[s.source_id,date(s.start_local),date(s.end_local),s.duration_s,null,s.messages,s.bytes/1024**3,null,null,null,s.localization_episodes,s.manual_required_episodes,s.cpu_mean_percent,s.cpu_p95_percent,s.lost_reported,s.commit?.slice(0,7)??'未隨bag保存']),6,'SessionRecords',[11,24,24,15,12,17,16,15,15,18,16,16,17,17,17,24]);
for(let r=st.first;r<=st.last;r++) {
 sess.getRange(`E${r}`).formulas=[[`=D${r}/3600`]];
 sess.getRange(`H${r}`).formulas=[[`=COUNTIFS('訂單明細'!$E$${ot.first}:$E$${ot.last},"done",'訂單明細'!$S$${ot.first}:$S$${ot.last},A${r})`]];
 sess.getRange(`I${r}`).formulas=[[`=COUNTIFS('訂單明細'!$E$${ot.first}:$E$${ot.last},"cancelled",'訂單明細'!$S$${ot.first}:$S$${ot.last},A${r})`]];
 sess.getRange(`J${r}`).formulas=[[`=COUNTIFS('訂單明細'!$E$${ot.first}:$E$${ot.last},"done",'訂單明細'!$S$${ot.first}:$S$${ot.last},A${r},'訂單明細'!$T$${ot.first}:$T$${ot.last},1)`]];
}
orders.getRange('S6:T6').values=[['結果來源','雲端確認旗標']];
orders.getRange(`S${ot.first}:T${ot.last}`).values=d.orders.map(o=>[o.result_source_id??'未錄',o.cloud_ack_local?1:0]);
orders.getRange(`S6:T${ot.last}`).format.columnWidth=18;
orders.getRange('S6:T6').format={fill:navy,font:{color:'#FFFFFF',bold:true,name:font,size:11}};
// Explicit complete-service subset keeps non-applicable cancelled rows out of
// statistical inputs and exposes the exact 69-row population in the task sheet.
const doneRows=d.orders.map((o,i)=>[o,i+ot.first]).filter(([o])=>o.result_status==='done');
orders.getRange('V6:X6').values=[['done任務短ID','確認尾段樣本(s)','雲端延遲樣本(s)']];
orders.getRange('V6:X6').format={fill:navy,font:{color:'#FFFFFF',bold:true,name:font,size:11},rowHeight:40,wrapText:true};
for(let i=0;i<doneRows.length;i++) {
 const [o,src]=doneRows[i],r=7+i;
 orders.getRange(`V${r}`).values=[[o.task_id.slice(0,8)]];
 orders.getRange(`W${r}:X${r}`).formulas=[[`=J${src}`,`=L${src}`]];
}
orders.getRange('V6:X75').format.columnWidth=24;numeric(orders,'W7:X75');
dtfmt(sess,`B${st.first}:C${st.last}`);
numeric(sess,`D${st.first}:E${st.last}`);numeric(sess,`G${st.first}:G${st.last}`);numeric(sess,`M${st.first}:N${st.last}`,'0.0');
numeric(sess,`F${st.first}:F${st.last}`,'#,##0');
sess.freezePanes.freezeRows(6);sess.freezePanes.freezeColumns(1);
sess.getRange('A19:F21').values=[['未納入資料',null,null,null,null,null],['20260923_172438_full_system：有前置資料，無metadata／db3／mcap',null,null,null,null,null],['validation_20260929_*：程式驗證與建置資料，不是實車rosbag',null,null,null,null,null]];
sess.getRange('A19').format.font={bold:true,name:font,size:11};

const loc=sheets['定位事件'];base(loc,'定位暫停與恢復區段',14);
titleNote(loc,'區段由非LOCALIZED開始，到LOCALIZED結束；程式放行不代表實體位置正確。');
const lt=table(loc,['事件ID','來源','開始','恢復LOCALIZED','區段秒數','局部恢復','全域恢復','曾要求人工','結局','左截斷／啟動','操作註記','起始吻合分數','起始xy標準差(m)','觸發原因'],d.localization_episodes.map(e=>[e.episode_id,e.source_id,date(e.start_local),date(e.end_local),null,e.local_recovery?1:0,e.global_recovery?1:0,e.manual_required?1:0,e.outcome,e.startup_or_left_censored?1:0,e.operation_note,e.map_score_at_start,e.xy_std_m,e.reason]),6,'LocalizationRecords',[16,11,24,24,16,13,13,15,29,18,58,20,24,75]);
for(let r=lt.first;r<=lt.last;r++) loc.getRange(`E${r}`).formulas=[[`=IF(AND(ISNUMBER(C${r}),ISNUMBER(D${r})),(D${r}-C${r})*86400,"n.a.")`]];
dtfmt(loc,`C${lt.first}:D${lt.last}`);numeric(loc,`E${lt.first}:E${lt.last}`);numeric(loc,`L${lt.first}:M${lt.last}`,'0.000');
flag(loc,`H${lt.first}:H${lt.last}`,'equal',1);
loc.freezePanes.freezeRows(6);loc.freezePanes.freezeColumns(2);
const parkrow=lt.last+5;
loc.getRange(`A${parkrow-2}`).values=[['待機到達時的AMCL估計差距']];
loc.getRange(`A${parkrow-1}`).values=[['僅為軟體自洽檢查，不用作實體停靠精度。']];
const pt=table(loc,['來源','回報到達','待機點','AMCL距離差(m)','AMCL角度差(°)','位姿資料年齡(s)'],d.parked.map(p=>[p.source_id,date(p.time_local),p.point,p.estimated_distance_m,p.estimated_yaw_error_deg,p.amcl_age_s]),parkrow,'ParkingEstimates',[16,24,24,23,23,23]);
dtfmt(loc,`B${pt.first}:B${pt.last}`);numeric(loc,`D${pt.first}:F${pt.last}`,'0.000');

const telemetry=sheets['感測效能'];base(telemetry,'關鍵topic接收效能',11);
titleNote(telemetry,'Hz=(n−1)/(最後−首筆時間)。接收間隔不是發布節點耗時，也不是全車可用率。');
const keep=new Set(['/scan','/scan_filtered','/odom','/stm32_data','/tf','/amcl_pose','/vision/semantic_info','/power_status']);
const tm=d.topic_metrics.filter(t=>keep.has(t.topic)).sort((a,b)=>a.source_id.localeCompare(b.source_id)||a.topic.localeCompare(b.topic));
const tt=table(telemetry,['來源','Topic','樣本數','平均Hz','最大間隔(s)','P95間隔(s)','>1秒間隔數','時間戳年齡中位(s)','時間戳年齡P95(s)','說明'],tm.map(t=>[t.source_id,t.topic,t.samples,t.hz,t.max_gap_s,t.p95_gap_s,t.gaps_gt_1s,t.age_median_s,t.age_p95_s,t.topic==='/amcl_pose'?'靜止時低頻更新，長間隔不直接判故障':t.topic==='/stm32_data'?'四輪分行回授，topic約8Hz不代表完整輪速8Hz':t.topic==='/tf'?'多種TF合併頻率，非單一變換更新率':'錄製器接收端量測']),6,'TopicMetrics',[11,29,17,14,20,20,18,24,24,58]);
numeric(telemetry,`C${tt.first}:C${tt.last}`,'#,##0');numeric(telemetry,`D${tt.first}:F${tt.last}`,'0.000');numeric(telemetry,`H${tt.first}:I${tt.last}`,'0.000');
flag(telemetry,`E${tt.first}:E${tt.last}`,'greaterThan',1);
telemetry.freezePanes.freezeRows(6);telemetry.freezePanes.freezeColumns(2);

const power=sheets['行充感測'];base(power,'INA3221讀取旗標與量程',10);
titleNote(power,'近量程上限定義：current ≥ 1.637 A；不由此推算USB端精確瓦數或電量百分比。');
const pw=table(power,['來源','通道','power樣本數','已知sensor_ok旗標','無效旗標筆數','旗標無效率','近量程上限筆數','近量程佔比','最高電流(A)','操作／範圍註記'],d.power.map(p=>[p.source_id,p.channel,p.samples,p.known_sensor_flags,p.invalid_samples,null,p.saturated_samples,null,p.current_max_a,p.note]),6,'PowerRecords',[11,12,19,25,20,18,23,19,21,75]);
for(let r=pw.first;r<=pw.last;r++) {
 power.getRange(`F${r}`).formulas=[[`=IF(D${r}=0,"n.a.",E${r}/D${r})`]];
 power.getRange(`H${r}`).formulas=[[`=IF(C${r}=0,"n.a.",G${r}/C${r})`]];
}
numeric(power,`F${pw.first}:F${pw.last}`,'0.0%');numeric(power,`H${pw.first}:H${pw.last}`,'0.0%');numeric(power,`I${pw.first}:I${pw.last}`,'0.000');
flag(power,`G${pw.first}:G${pw.last}`,'greaterThan',0);
power.freezePanes.freezeRows(6);power.freezePanes.freezeColumns(2);

const vision=sheets['視覺語意'];base(vision,'視覺輸出與語意速度命令',11);
titleNote(vision,'命令驗證僅檢查軟體速度輸出。無標註影像與實體制動量測，不能計算辨識準確率。');
const vt=table(vision,['來源','語意topic平均Hz','推論FPS中位','倍率0.5觀察秒數','倍率0觀察秒數','倍率1觀察秒數','未觀察缺口(s)','半速非零輸入配對','半速匹配輸出','零速非零輸入配對','零速匹配輸出'],d.semantic.map(x=>[x.source_id,d.sessions.find(s=>s.source_id===x.source_id).vision_message_hz,x.vision_fps?.median??null,x.duration_s['0.5']??0,x.duration_s['0.0']??0,x.duration_s['1.0']??0,x.duration_s.unobserved_gap_s??0,x.commands.half_nonzero_input??0,x.commands.half_matching_output??0,x.commands.zero_nonzero_input??0,x.commands.zero_matching_output??0]),6,'SemanticRecords',[11,24,21,24,24,24,23,27,24,27,24]);
numeric(vision,`B${vt.first}:G${vt.last}`,'0.00');
vision.getRange('A20').values=[['量測方式']];vision.getRange('A21').values=[['每段倍率最多延續0.5秒；較長的接收缺口另列，不把舊倍率無限延伸。']];
vision.getRange('A22').values=[['速度配對：最近/cmd_vel與/chassis_cmd_vel相差≤0.10秒、輸入線速度>0.02m/s，各分量誤差<0.001。']];
vision.getRange('A23').values=[['同一時刻可能有定位／防撞鎖定，不能把零速輸出全部歸因於辨識到人。']];
vision.freezePanes.freezeRows(6);

const sources=sheets['來源與方法'];base(sources,'資料來源與量化定義',7);
titleNote(sources,'分析截止2026-10-03。原始rosbag保留在樹莓派，僅作唯讀存取。');
const sr=table(sources,['來源','資料夾','格式','車載版本快照','metadata SHA256','原始bag位置','已知操作／缺漏'],d.sessions.map(s=>[s.source_id,s.run,s.storage,s.commit??'未保存；不可用目前HEAD代替',s.metadata_sha256,s.source_uri,s.initialpose_qos_warning?'人工initialpose有QoS衝突，人工位置未保證完整錄製':'未見initialpose QoS衝突；未錄hidden action status']),6,'SourceRegistry',[11,55,14,48,74,100,74]);
const methodRows=[
 ['分析範圍','2026-09-22至2026-10-03現存9份bag；錄製時段總和，不是連續耐久測試。'],
 ['任務分母','至少一筆狀態或結果的事件內時間戳位於錄製視窗內；依task_id跨bag去重。'],
 ['終態事件','依event_id去重，使用created_at；同一任務在不同bag延續只算一次。'],
 ['歷史重播排除','S02一筆12:38:54取消事件，在12:39:41開始錄製前已產生，因此不納入本次終態。'],
 ['服務done','只代表車端結果done；含人工介入、重啟與錯槽操作修正，不是全自主成功率。'],
 ['取消任務','3筆cancelled另列。取消不直接當失敗，也沒有資料可驗證所有未claim的網頁訂單。'],
 ['取還確認尾段','最後一輪verifying_action事件時間到done.created_at。不包含INA前級去抖與使用者插拔時間。'],
 ['雲端延遲','done.created_at到API bridge文字日誌「雲端已確認任務結果」；不是前端畫面延遲。'],
 ['首ACK','首個對應事件ACK可能只是本機outbox持久化，不能冒充雲端確認。'],
 ['定位區段','連續非LOCALIZED狀態合為一區段，到首次LOCALIZED結束。啟動左截斷與未結束區段另標。'],
 ['物理正確性','程式恢復LOCALIZED不證明位置正確；S06搬車後已由使用者觀察到錯誤局部解。'],
 ['人工介入','MANUAL_REQUIRED以區段數計，不以重複訊息數計；6個區段含啟動與資料管線問題。'],
 ['恢復耗時','僅對起迄皆有觀察、非啟動左截斷區段計算；未完成區段不當0秒。'],
 ['地圖吻合與covariance','map_match_score為程式雷達撞圖分數；xy_std為估計分布不確定度，不是真實定位誤差。'],
 ['訊息Hz與間隔','使用bag接收時間。回呼阻塞、DDS、網路或錄製器丟訊都可能影響，不直接推論硬體故障。'],
 ['時間戳年齡','bag接收時間−ROS header.stamp；含感測器掃描週期與通訊，不等同scan filter純處理延遲。'],
 ['CPU','只有S01至S06保存system_top；整體四核心忙碌率=100−idle，含Nav2、桌面、錄製等。'],
 ['FPS','語意topic平均Hz與health推論FPS中位是不同量；S01至S06缺health樣本不填0。'],
 ['感測旗標','71,556筆通道樣本均有sensor_ok欄位，false=0；不代表功率或槽位分類準確率100%。'],
 ['量程上限','current≥1.637 A標為近上限；使用者確認1.638 A量程與USB實機約18W，不作精確能量估測。'],
 ['停靠差距','由AMCL與目標計算，未有外部位置真值；不能用來宣稱實體厘米級停靠。'],
 ['多筆批次','batch_id跨bag去重，共52批，其中17批≥2任務；不計算缺乏對照的最短路線或0.7效益。'],
 ['百分位','對逐筆數據合併計算，P95用線性插值；不平均每場中位數或P95。'],
 ['資料完整性','9場exit code皆0，但紀錄器累積告警合計275筆transport loss，無逐topic完整丟包率。'],
 ['版本比較','S01至S06有git_commit快照；S07至S09未隨bag保存，不推定與現在版本相同。'],
 ['時間格式','全表以臺灣UTC+08:00壁鐘呈現，Excel儲存為可排序的日期數值；底層JSON另有原始epoch。'],
 ['實驗限制','服務點、租還種類、電池配置、網路、人員與版本不固定，不能宣稱因果改善或跨環境可靠度。'],
];
const mr=table(sources,['項目','定義／限制'],methodRows,20,'Definitions',[29,132]);
sources.getRange(`B${mr.first}:B${mr.last}`).format.wrapText=true;sources.getRange(`A${mr.first}:B${mr.last}`).format.rowHeight=38;
sources.freezePanes.freezeRows(6);sources.freezePanes.freezeColumns(1);

// Summary owns no duplicated hardcoded computed totals.
const summary=sheets['量化摘要'];base(summary,'FlowCharge 實車測試量化結果',18);
summary.tabColor=navy;
titleNote(summary,'2026-09-22至2026-10-03，臺灣時間；包含人工協助及版本迭代。');
const sumRows=[
 ['完整rosbag',null,'份','9份現存資料，另1個測試資料夾缺bag'],
 ['錄製時段總和',null,'小時','不是連續耐久運行'],
 ['紀錄訊息總數',null,'筆','所有錄製topic訊息，非獨立實驗樣本'],
 ['記錄內活動任務',null,'筆','task_id跨bag去重，歷史重播另列'],
 ['服務完成done',null,'筆','包含人工協助與重啟，不是自主成功率'],
 ['取消任務',null,'筆','另列cancelled，不直接視作配送失敗'],
 ['完成結果雲端確認率',null,'比例','分母只含done任務'],
 ['取還確認尾段中位',null,'秒','verifying_action至車端done，n=69'],
 ['取還確認尾段P95',null,'秒','不包含INA前級去抖與實體插拔'],
 ['雲端確認延遲中位',null,'秒','車端done至API日誌確認，n=69'],
 ['雲端確認延遲P95',null,'秒','不是網頁顯示延遲'],
 ['雲端確認延遲最大',null,'秒','長尾受網路與重送影響'],
 ['定位非LOCALIZED區段',null,'個','含3個啟動／左截斷區段、1個未結束'],
 ['曾要求人工區段',null,'個','不直接當自主重定位失敗率'],
 ['無效INA旗標',null,'筆','71,556通道樣本均有欄位；不等同量測精度'],
];
const qt=table(summary,['指標','結果','單位','範圍'],sumRows,6,'SummaryMetrics',[33,19,13,77]);
const forms=[`=ROWS('測試批次'!A${st.first}:A${st.last})`,`=SUM('測試批次'!E${st.first}:E${st.last})`,`=SUM('測試批次'!F${st.first}:F${st.last})`,
 `=ROWS('訂單明細'!A${ot.first}:A${ot.last})`,`=COUNTIFS('訂單明細'!E${ot.first}:E${ot.last},"done")`,`=COUNTIFS('訂單明細'!E${ot.first}:E${ot.last},"cancelled")`,
 `=IF(B11=0,"n.a.",COUNTIFS('訂單明細'!E${ot.first}:E${ot.last},"done",'訂單明細'!T${ot.first}:T${ot.last},1)/B11)`,
 `=MEDIAN('訂單明細'!W7:W75)`,`=PERCENTILE.INC('訂單明細'!W7:W75,0.95)`,`=MEDIAN('訂單明細'!X7:X75)`,
 `=PERCENTILE.INC('訂單明細'!X7:X75,0.95)`,`=MAX('訂單明細'!X7:X75)`,
 `=ROWS('定位事件'!A${lt.first}:A${lt.last})`,`=SUM('定位事件'!H${lt.first}:H${lt.last})`,`=SUM('行充感測'!E${pw.first}:E${pw.last})`];
summary.getRange(`B${qt.first}:B${qt.last}`).formulas=forms.map(f=>[f]);
numeric(summary,`B${qt.first}:B${qt.last}`,'0.00');numeric(summary,'B7','0');numeric(summary,'B9:B12','#,##0');numeric(summary,'B13','0.0%');numeric(summary,'B19:B21','#,##0');
summary.getRange('A24').values=[['逐場服務結果']];summary.getRange('A24').format.font={name:font,size:12,bold:true};
const dates=['09/22','09/23','09/24','09/29下午①','09/29下午②','09/29晚間','09/30','10/01','10/03'];
summary.getRange('A26:D26').values=[['場次','done','取消','曾要求人工區段']];
for(let i=0;i<d.sessions.length;i++) {
 const r=27+i, sr=st.first+i;
 summary.getRange(`A${r}`).values=[[dates[i]]];
 summary.getRange(`B${r}:D${r}`).formulas=[[`='測試批次'!H${sr}`,`='測試批次'!I${sr}`,`='測試批次'!L${sr}`]];
}
summary.getRange('A26:D26').format={fill:navy,font:{name:font,size:11,color:'#FFFFFF',bold:true},horizontalAlignment:'center'};
const chart=summary.charts.add('bar',[summary.getRange('A26:A35'),summary.getRange('B26:B35'),summary.getRange('C26:C35')]);
chart.title='逐場完成與取消任務';chart.titleTextStyle.typeface=font;chart.titleTextStyle.fontSize=14;
chart.legend={position:'top',textStyle:{typeface:font,fontSize:11}};
chart.xAxis={axisType:'textAxis',textStyle:{typeface:font,fontSize:10}};
chart.yAxis={numberFormatCode:'0',numberFormatSourceLinked:false,textStyle:{typeface:font,fontSize:11}};
chart.setPosition('F6','N22');
chart.series.items[0].fill='#355C85';chart.series.items[1].fill='#C78736';
summary.getRange('F25').values=[['近期三場：9/30、10/1、10/3']];summary.getRange('F25').format.font={name:font,size:12,bold:true};
summary.getRange('F27:H30').values=[['指標','結果','單位'],['服務done',null,'筆'],['取消',null,'筆'],['曾要求人工區段',null,'個']];
summary.getRange('G28').formulas=[[`=SUM('測試批次'!H13:H15)`]];
summary.getRange('G29').formulas=[[`=SUM('測試批次'!I13:I15)`]];
summary.getRange('G30').formulas=[[`=SUM('測試批次'!L13:L15)`]];
summary.getRange('F27:H27').format={fill:navy,font:{name:font,size:11,bold:true,color:'#FFFFFF'}};
summary.getRange('F32').values=[['35筆done都有雲端確認；仍有定位暫停與人工介入。']];
summary.getRange('F34').values=[['不可將這35筆宣稱為35次全自主成功配送。']];
summary.getRange('A38').values=[['可用於專題：服務完成數、確認尾段延遲、結果重送、topic效能、定位事件數。']];
summary.getRange('A39').values=[['尚缺：前端顯示時間、物理位置真值、逐影像標註、每次插拔時間及完整人工搬車標記。']];
summary.getRange('F6:N39').format.columnWidth=13;
summary.getRange('F27:F30').format.columnWidth=28;

wb.recalculate();
// Independent values from Python analysis must agree within millisecond display precision.
const checks=[['量化摘要','B7',9,0],['量化摘要','B8',d.summary.duration_h,.00001],['量化摘要','B9',d.summary.messages,0],
 ['量化摘要','B10',72,0],['量化摘要','B11',69,0],['量化摘要','B12',3,0],['量化摘要','B13',1,0],
 ['量化摘要','B14',d.summary.confirm_stats.median,.002],['量化摘要','B15',d.summary.confirm_stats.p95,.002],
 ['量化摘要','B16',d.summary.cloud_stats.median,.002],['量化摘要','B17',d.summary.cloud_stats.p95,.002],
 ['量化摘要','B18',d.summary.cloud_stats.max,.002],['量化摘要','B19',34,0],['量化摘要','B20',6,0],['量化摘要','B21',0,0],['量化摘要','G28',35,0]];
for(const [sn,cell,expected,tol] of checks) {
 const actual=sheets[sn].getRange(cell).values[0][0];
 if(typeof actual!=='number'||Math.abs(actual-expected)>tol) throw new Error(`${sn}!${cell}: ${actual} expected ${expected}`);
}
console.log((await wb.inspect({kind:'table',range:'量化摘要!A6:D21',include:'values,formulas',tableMaxRows:16,tableMaxCols:4,maxChars:5000})).ndjson);
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:40},summary:'final formula error scan',maxChars:2000})).ndjson);
// Recalculation probe: add 1 second to an input, verify dependent duration, restore.
const testR=ot.first;
const original=orders.getRange(`I${testR}`).values[0][0];
const before=orders.getRange(`J${testR}`).values[0][0];
orders.getRange(`I${testR}`).values=[[original+1/86400]];
wb.recalculate();
if(Math.abs(orders.getRange(`J${testR}`).values[0][0]-before-1)>.002) throw new Error('Duration recalculation probe failed');
orders.getRange(`I${testR}`).values=[[original]];wb.recalculate();
const views=[['量化摘要','A1:N40'],['測試批次','A1:J16'],['訂單明細','A1:L16'],['定位事件','A1:J17'],['感測效能','A1:J18'],['行充感測','A1:I18'],['視覺語意','A1:K24'],['來源與方法','A19:B34']];
for(const [sn,range] of views) {
 const blob=await wb.render({sheetName:sn,range,scale:1.4,format:'png'});
 await fs.writeFile(path.join(previewDir,`${sn}.png`),new Uint8Array(await blob.arrayBuffer()));
}
const xlsx=await SpreadsheetFile.exportXlsx(wb);
await xlsx.save(path.join(outputDir,'FlowCharge_rosbag量化結果_20260922_20261003.xlsx'));
await fs.writeFile(path.join(previewDir,'verification.json'),JSON.stringify({checks:checks.length,sheets:names,orders:d.orders.length,sources:d.sessions.length},null,2));
console.log('Export complete');
