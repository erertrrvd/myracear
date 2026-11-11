param([string]$Variant = 'relay', [int]$Dom = 61, [double]$Outage = 12, [int]$Dur = 30)
# relay：fake_car -> scan_relay -> 电脑（auto）；direct：fake_car -> 电脑（source:=direct，不开 relay）
$S = $PSScriptRoot   # 输出写在本目录
$R = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
Set-Location D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY = '1'; $env:ROS_DOMAIN_ID = "$Dom"
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
$env:PYTHONUNBUFFERED = '1'; $env:PYTHONIOENCODING = 'utf-8'
$mapDir = "D:/ubuntu/slam/maps_test_relay_$Variant"
Remove-Item -Recurse -Force "D:\ubuntu\slam\maps_test_relay_$Variant" -ErrorAction SilentlyContinue
$src = if ($Variant -in 'relay','norelay') { 'auto' } else { 'direct' }
$slamMode = if ($Variant -eq 'async') { 'async' } else { 'sync' }
$slam = Start-Process pixi -ArgumentList 'run','--frozen','ros2','launch',"`"$R\mapping\launch\distributed_mapping_pc_windows.launch.py`"","map_dir:=$mapDir",'rviz:=false','solid:=false','autosave_period:=5',"source:=$src","slam_mode:=$slamMode" -PassThru -WindowStyle Hidden -RedirectStandardOutput "$S\t_$Variant.out" -RedirectStandardError "$S\t_$Variant.err"
if ($Variant -eq 'relay') {
    $relay = Start-Process pixi -ArgumentList 'run','--frozen','python',"`"$R\mapping\scripts\scan_relay`"" -PassThru -WindowStyle Hidden -RedirectStandardOutput "$S\t_relay_node.out" -RedirectStandardError "$S\t_relay_node.err"
}
Start-Sleep 14
$car = Start-Process pixi -ArgumentList 'run','--frozen','python',"`"$S\fake_car3.py`"",'1.0',"$Dur" -PassThru -WindowStyle Hidden
Start-Sleep 8
$br = Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -match 'scan_tf_bridge' } | Select-Object -First 1
"bridge pid $($br.ProcessId)" | Out-File "$S\t_$Variant.log" -Encoding utf8
if ($Outage -gt 0) { & python "$S\suspend.py" $br.ProcessId $Outage | Out-File "$S\t_$Variant.log" -Append -Encoding utf8 }
Start-Sleep ($Dur - 8 - $Outage + 30)
$procs = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'maps_test_relay_|fake_car3|scan_relay' -and $_.Name -ne 'powershell.exe' }
foreach ($p in $procs) { & taskkill.exe /PID $p.ProcessId /T /F 2>$null | Out-Null }
Start-Sleep 2
& python "$S\eval_map3.py" "D:\ubuntu\slam\maps_test_relay_$Variant\autosave\map.yaml" 2>&1 | Out-File "$S\t_$Variant.log" -Append -Encoding utf8
"DONE" | Out-File "$S\t_$Variant.log" -Append -Encoding utf8
