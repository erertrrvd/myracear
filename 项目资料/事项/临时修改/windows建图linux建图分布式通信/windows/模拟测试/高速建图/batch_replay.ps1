# 批量离线重建 11-03 的车上包（新流程：/key1_odom 只取一条里程计轨迹、去畸变、地面线过滤、存图后描黑 + 赛道拟合 + 报告）
param([string]$Variant = 'default')
$replay = 'D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\replay.ps1'
$B = 'C:\Users\lzc2\Desktop\小车原始包'
$names = @('map_20270127_104547','map_20270127_103427','map_20270127_101953','map_20270128_110741','map_20270128_111228',
           'map_20270126_095524','map_20270127_102454','map_20270127_105228','map_20270126_100700',
           'map_20270126_101008_修复','map_20270127_101941_修复')
$names = $names | ForEach-Object { if ($_ -like '*_修复') { $_ } elseif (Test-Path "$B\$($_)_修复") { "$($_)_修复" } else { $_ } }
foreach ($n in $names) {
    $short = $n -replace '_修复', ''
    $out = "D:\ubuntu\slam\maps_replay\1103_${short}_$Variant"
    if (Test-Path "$out\run1\report.html") { Write-Host "跳过（已有）：$out"; continue }
    $extra = @()
    if ($Variant -eq 'fast') { $extra = @('-Mapper', 'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\config\mapper_fast.yaml') }
    if ($Variant -eq 'nodeskew') { $extra = @('-NoDeskew') }
    Write-Host "===== $(Get-Date -Format HH:mm:ss) $n -> $out"
    & powershell -NoProfile -ExecutionPolicy Bypass -File $replay -Bag "$B\$n" -Out $out @extra *>&1 | Select-Object -Last 8
}
Write-Host "===== 全部完成 $(Get-Date -Format HH:mm:ss)"
