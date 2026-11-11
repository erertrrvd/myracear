# 拖拽入口：把 CSV 文件拖到「拖拽CSV到这里.bat」上，自动在 rviz2 里和固定的地图融合显示。
# 也可手动运行： .\drop_view.ps1 "D:\a.csv" "D:\b.csv"
#
# 做了什么：
#   - 地图固定用本文件夹里的 run_20261005_1552_filtered\run1\map.yaml（以后都用这张）
#   - 拖进来的每个 CSV 各画成一种颜色（起点绿球、终点红球、标文件名和点数，改文件会自动刷新）
#   - get_local_path 的跟踪日志（path_tracking_*.csv，表头 px,py,... cx,cy,...）会拆成“目标”“实际”两条线
#   - 只画拖进来的 CSV，不画项目里的 path.csv；一个 CSV 都没拖就显示地图 + path.csv
#   - 障碍格加粗 1 格（只影响显示，地图文件不变）
# 要求：CSV 的坐标和这张地图在同一个坐标系（建图起点为原点的 map 系）。
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Files)

$ErrorActionPreference = 'Stop'
$view = 'D:\ubuntu\slam\view_csv.ps1'                       # 查看工具（项目外，pixi 环境里的 rviz2）
$mapDirName = 'run_20261005_1552_filtered'                  # 固定使用的地图文件夹（与本脚本同目录）

function Fail($msg) { Write-Host "!! $msg" -ForegroundColor Red; Write-Host ''; exit 1 }

if (-not (Test-Path $view)) { Fail "找不到查看工具：$view（需要 D:\ubuntu\slam 里的 pixi 环境）" }

# 找地图：优先正式保存的 run1，其次自动保存的 autosave
$root = Join-Path $PSScriptRoot $mapDirName
$map = @("$root\run1\map.yaml", "$root\autosave\map.yaml") | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $map) { Fail "找不到地图：$root\run1\map.yaml`n   请确认地图文件夹 $mapDirName 和本脚本放在同一目录。" }

# 整理拖进来的文件：只收 .csv
$csvs = @()
foreach ($f in $Files) {
    if (-not $f) { continue }
    if (-not (Test-Path -LiteralPath $f)) { Write-Host "跳过（找不到）：$f" -ForegroundColor Yellow; continue }
    if ([IO.Path]::GetExtension($f).ToLower() -ne '.csv') { Write-Host "跳过（不是 CSV）：$f" -ForegroundColor Yellow; continue }
    $csvs += (Resolve-Path -LiteralPath $f).Path
}

Write-Host "地图 : $map"
if ($csvs.Count) {
    Write-Host 'CSV  :'; $csvs | ForEach-Object { Write-Host "   $_" }
    Write-Host '正在打开 rviz2（关掉 rviz2 窗口即退出）...'
    & $view -Map $map -Extra $csvs -NoPath -Inflate 1
} else {
    Write-Host '没有拖入 CSV：只显示地图 + 项目里的 path.csv。' -ForegroundColor Yellow
    Write-Host '正在打开 rviz2（关掉 rviz2 窗口即退出）...'
    & $view -Map $map -Inflate 1
}