$S = $PSScriptRoot   # 输出写在本目录
Set-Location D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY = '1'; $env:ROS_DOMAIN_ID = '67'
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force "$S\bag_test" -ErrorAction SilentlyContinue
$rec = Start-Process pixi -ArgumentList 'run','--frozen','ros2','bag','record','-o',"`"$S\bag_test`"",'/scan','/encoder_imu_odom' -PassThru -WindowStyle Hidden -RedirectStandardOutput "$S\rec.out" -RedirectStandardError "$S\rec.err"
Start-Sleep 6
$car = Start-Process pixi -ArgumentList 'run','--frozen','python',"`"$S\fake_car3.py`"",'1.0','30' -PassThru -WindowStyle Hidden -Wait
Start-Sleep 3
# 像 Ctrl+C 一样让 ros2 bag record 收尾：用 taskkill（无 /F）发关闭，不行再强杀
$p = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'bag record' -and $_.Name -ne 'powershell.exe' }
foreach ($x in $p) { & taskkill.exe /PID $x.ProcessId /T 2>$null | Out-Null }
Start-Sleep 5
$p = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'bag record' -and $_.Name -ne 'powershell.exe' }
foreach ($x in $p) { & taskkill.exe /PID $x.ProcessId /T /F 2>$null | Out-Null }
Get-ChildItem "$S\bag_test" | Select-Object Name, Length | Format-Table -AutoSize | Out-String
