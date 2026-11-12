param(
    [Parameter(Mandatory=$true)][string]$Bag,
    [Parameter(Mandatory=$true)][string]$Out,
    [int]$Domain = 77,
    [double]$Rate = 1.0,
    [switch]$Baseline,
    [switch]$PauseLap
)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $Out) { throw '输出目录已存在，请指定新的目录' }
$project = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$player = Join-Path $PSScriptRoot 'player.py'
$env:ROS_DOMAIN_ID = "$Domain"
$env:ROS_LOCALHOST_ONLY = '1'
$env:PYTHONIOENCODING = 'utf-8'
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $Out | Out-Null
$outFs = $Out.Replace([char]92,[char]47)
$launch = "$project\mapping\launch\distributed_mapping_pc_windows.launch.py"
$config = "$project\mapping\config\mapper_fast.yaml"
$args = @('run','--frozen','ros2','launch',"`"$launch`"", "map_dir:=$outFs", 'rviz:=false',
          'autosave_period:=0', 'vote:=false',
          "mapper_config:=`"$config`"")
if (-not $Baseline -and -not $PauseLap) {
    $args += @('odom_only:=true', 'anchor_graph:=true', 'field_enabled:=true',
               'field_front_m:=15.0', 'field_back_m:=15.0', 'field_left_m:=10.0', 'field_right_m:=10.0')
}
if ($PauseLap) { $args += @('odom_only:=true', 'pause_after_lap:=true', 'vote:=true') }
$proc = Start-Process pixi -WorkingDirectory 'D:\ubuntu\slam' -ArgumentList $args -PassThru -WindowStyle Hidden `
    -RedirectStandardOutput "$Out\launch.log" -RedirectStandardError "$Out\launch.err"
$recorder = $null
if ($Baseline) {
    $recordScript = Join-Path $PSScriptRoot 'record_slam_poses.py'
    $recordArgs = @('run','--frozen','python',"`"$recordScript`"", "`"$Out\slam_poses.csv`"")
    $recorder = Start-Process pixi -WorkingDirectory 'D:\ubuntu\slam' -ArgumentList $recordArgs -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput "$Out\recorder.log" -RedirectStandardError "$Out\recorder.err"
}
try {
    Start-Sleep -Seconds 12
    Push-Location 'D:\ubuntu\slam'
    try {
        pixi run --frozen python -I $player $Bag --rate "$Rate" --track -1 --odom-lag 0
        if ($LASTEXITCODE -ne 0) { throw "录包播放失败，退出码 $LASTEXITCODE" }
        Start-Sleep -Seconds 15
        if ($Baseline -or $PauseLap) {
            pixi run --frozen python "$project\mapping\scripts\save_mapping" --run-index 1 --map-dir $outFs --raw-only
            if ($LASTEXITCODE -ne 0) { throw "保存原始地图失败，退出码 $LASTEXITCODE" }
        } else {
            $export = "$project\mapping\scripts\export_cone_factor_map.py"
            foreach ($item in @(@('/cone_factor_graph/map','graph'),@('/cone_factor_graph/odom_map','odom'))) {
                pixi run --frozen python $export --topic $item[0] --output "$outFs/$($item[1])" --timeout 10
                if ($LASTEXITCODE -ne 0) { throw "保存因子图地图失败，退出码 $LASTEXITCODE" }
            }
        }
    } finally { Pop-Location }
} finally {
    # 只结束本脚本创建的 launch 进程树。
    if (-not $proc.HasExited) { & taskkill /PID $proc.Id /T /F *> $null }
    if ($recorder -and -not $recorder.HasExited) { & taskkill /PID $recorder.Id /T /F *> $null }
}
