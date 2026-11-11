# 离线重建一个车上录包：起建图 launch（无 rviz、本机隔离的 DDS 域）→ player.py 播放 → save_mapping 保存 → map_report 出报告。
#   powershell -ExecutionPolicy Bypass -File replay.ps1 -Bag D:\ubuntu\slam\bags_car\map_xxx -Out D:\ubuntu\slam\maps_replay\xxx_new [-NoDeskew] [-Mapper path] [-Track N] [-Rate 1.0]
param(
    [Parameter(Mandatory)][string]$Bag,
    [Parameter(Mandatory)][string]$Out,
    [switch]$NoDeskew,
    [string]$Mapper = 'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\config\mapper.yaml',
    [int]$Track = -1,
    [double]$Rate = 1.0,
    [int]$Domain = 77,
    [double]$OdomLag = 0.0,    # 模拟 WiFi 里程计成批晚到（player.py --odom-lag），0 = 不模拟
    [double]$OdomWait = 1.0    # 建图桥等里程计最多几秒（scan_tf_bridge odom_wait），0 = 旧做法不等
)
$ErrorActionPreference = 'Stop'
$proj = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location 'D:\ubuntu\slam'
$env:ROS_DOMAIN_ID = "$Domain"; $env:ROS_LOCALHOST_ONLY = '1'; $env:PYTHONIOENCODING = 'utf-8'
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
$null = New-Item -ItemType Directory -Force -Path $Out
$outFs = $Out.Replace([string][char]92, '/')
$deskew = if ($NoDeskew) { 'false' } else { 'true' }
$inv = [Globalization.CultureInfo]::InvariantCulture
$rateArg = [string]::Format($inv, '{0:0.0##}', $Rate)
Write-Host "重建：$Bag -> $Out  deskew=$deskew  mapper=$Mapper  rate=$rateArg"
$launchArgs = @('run','--frozen','ros2','launch',"`"$proj\mapping\launch\distributed_mapping_pc_windows.launch.py`"",
                "map_dir:=$outFs","run_index:=1","rviz:=false","autosave_period:=0","deskew:=$deskew","mapper_config:=`"$Mapper`"",
                "odom_wait:=$([string]::Format($inv, '{0:0.0##}', $OdomWait))","vote:=false","odom_source:=imu","ground_filter:=off")
$log = "$Out\launch.log"
$launch = Start-Process pixi -ArgumentList $launchArgs -PassThru -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError "$Out\launch.err"
try {
    Start-Sleep -Seconds 10
    pixi run --frozen python -I "$here\imu_player.py" "$Bag" --rate $rateArg --track $Track --odom-lag $([string]::Format($inv, '{0:0.0##}', $OdomLag))
    Start-Sleep -Seconds 5
    pixi run --frozen python "$proj\mapping\scripts\save_mapping" --run-index 1 --map-dir $outFs --raw-only
} finally {
    # 结束整棵进程树（pixi -> ros2 launch -> 各节点）
    & taskkill /PID $launch.Id /T /F *> $null
}
