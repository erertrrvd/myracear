$S = 'C:\Users\lzc2\AppData\Local\Temp\claude\C--Users-lzc2-Desktop-my-racecar----\5ca29077-7719-4ef5-9bb7-8ac44bf526ad\scratchpad'
$R = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$result = "$S\experiment_result2.txt"
[IO.File]::WriteAllText($result, "", (New-Object System.Text.UTF8Encoding($false)))
$base = [IO.File]::ReadAllText("$R\mapping\config\mapper.yaml")
# 名称, occupancy_threshold, min_pass_through, beams
$variants = @(
    @('1_当前(0.35,4) 不去畸变', '0.35', '4', 720, 'false'),
    @('2_当前(0.35,4) 去畸变',   '0.35', '4', 720, 'true'),
    @('3_降阈值(0.20,3) 不去畸变', '0.20', '3', 720, 'false'),
    @('4_降阈值(0.20,3) 去畸变',   '0.20', '3', 720, 'true'),
    @('5_更松(0.10,2) 不去畸变',  '0.10', '2', 720, 'false')
)
$i = 0
foreach ($v in $variants) {
    $i++
    $name = $v[0]
    $cfg = $base
    $cfg = [regex]::Replace($cfg, '(?m)^(\s*occupancy_threshold:\s*)\S+', ('${1}' + $v[1]))
    $cfg = [regex]::Replace($cfg, '(?m)^(\s*min_pass_through:\s*)\S+', ('${1}' + $v[2]))
    $cfgPath = "$S\mapper2_v$i.yaml"
    [IO.File]::WriteAllText($cfgPath, $cfg, (New-Object System.Text.UTF8Encoding($false)))
    $mapDir = "D:/ubuntu/slam/maps_exp2_v$i"
    $mapDirWin = "D:\ubuntu\slam\maps_exp2_v$i"
    Set-Location D:\ubuntu\slam
    $env:ROS_LOCALHOST_ONLY = '1'; $env:ROS_DOMAIN_ID = "$(90 + $i)"
    $slam = Start-Process pixi -ArgumentList 'run','--frozen','ros2','launch',"`"$R\mapping\launch\distributed_mapping_pc_windows.launch.py`"","map_dir:=$mapDir",'rviz:=false','autosave_period:=5',"mapper_config:=$cfgPath","beams:=$($v[3])","deskew:=$($v[4])" -PassThru -WindowStyle Hidden -RedirectStandardOutput "$S\exp2_v$i.out" -RedirectStandardError "$S\exp2_v$i.err"
    Start-Sleep 12
    $car = Start-Process pixi -ArgumentList 'run','--frozen','python',"`"$S\fake_car3.py`"",'1.5','24' -PassThru -WindowStyle Hidden
    Start-Sleep 44
    $procs = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'maps_exp2_v|fake_car3' }
    foreach ($p in $procs) { & taskkill.exe /PID $p.ProcessId /T /F 2>$null | Out-Null }
    Start-Sleep 2
    $line = & python "$S\eval_map3.py" "$mapDirWin\autosave\map.yaml" 2>&1 | Select-Object -Last 1
    Add-Content -Path $result -Value "$name : $line" -Encoding UTF8
}
Add-Content -Path $result -Value "DONE" -Encoding UTF8
