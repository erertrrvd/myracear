# 一键：在 rviz2 里显示 path.csv + 任意调试 CSV（+ 可选已保存的地图），不需要小车和 SLAM。
# 用法（PowerShell）：
#   .\view_csv.ps1                                   只看 path.csv
#   .\view_csv.ps1 -Extra "D:\a.csv","D:\b.csv"      叠加调试段（改文件自动刷新）
#   .\view_csv.ps1 -Map "D:\ubuntu\slam\maps\run1\map.yaml"   再加地图
#   .\view_csv.ps1 -Dx 0.2 -Dy -0.1 -Dyaw 0.05       手动平移/旋转路径
param(
    [string[]]$Extra = @(),
    [string]$Map = '',
    [string]$PathCsv = '',
    [string]$AbA = '',
    [string]$AbB = '',
    [int]$ForkIndex = -1,
    [int]$Inflate = 0,
    [switch]$NoPath,
    [double]$Dx = 0, [double]$Dy = 0, [double]$Dyaw = 0
)
$ErrorActionPreference = 'Stop'
$proj  = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$slam  = 'D:\ubuntu\slam'
$node  = Join-Path $proj 'mapping\scripts\view_path_on_map'
$rviz  = Join-Path $proj 'mapping\rviz\path_on_map.rviz'
if (-not $PathCsv) { $PathCsv = Join-Path $proj 'include\path.csv' }

$rosArgs = @('--ros-args', '-p', "path_csv:=$PathCsv", '-p', "ab_fork_index:=$ForkIndex",
             '-p', ('dx:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dx)),
             '-p', ('dy:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dy)),
             '-p', ('dyaw:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dyaw)))
if ($Extra.Count) { $rosArgs += @('-p', ('extra_csv:=' + ($Extra -join ','))) }
if ($Map)  { $rosArgs += @('-p', "map_yaml:=$Map") }
if ($NoPath) { $rosArgs += @('-p', 'show_path:=false') }   # 只显示 -Extra 的 CSV，不画 path.csv
if ($Inflate -gt 0) { $rosArgs += @('-p', "inflate_cells:=$Inflate") }   # 仅显示：障碍格加粗，锥桶更醒目
if ($AbA)  { $rosArgs += @('-p', "ab_a_csv:=$AbA") }
if ($AbB)  { $rosArgs += @('-p', "ab_b_csv:=$AbB") }

$env:ROS_LOCALHOST_ONLY = '1'        # 只在本机通信，不干扰小车网络
Set-Location $slam
$nodeProc = Start-Process pixi -ArgumentList (@('run','--frozen','python',"`"$node`"") + $rosArgs) -PassThru -WindowStyle Hidden -RedirectStandardError "$env:TEMP\view_csv_node.err" -RedirectStandardOutput "$env:TEMP\view_csv_node.out"
Start-Sleep -Seconds 3
if ($nodeProc.HasExited) { Write-Host 'Node failed to start:'; Get-Content "$env:TEMP\view_csv_node.err"; exit 1 }
try {
    pixi run --frozen rviz2 -d $rviz
} finally {
    if ($nodeProc -and -not $nodeProc.HasExited) { taskkill /PID $nodeProc.Id /T /F | Out-Null }
}