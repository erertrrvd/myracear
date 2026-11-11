$proj = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location 'D:\ubuntu\slam'
$env:ROS_DOMAIN_ID = '79'; $env:ROS_LOCALHOST_ONLY = '1'; $env:PYTHONIOENCODING = 'utf-8'
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
$out = 'D:/ubuntu/slam/maps_replay/test_ekf_launch'
$l = Start-Process pixi -ArgumentList @('run','--frozen','ros2','launch',"`"$proj\mapping\launch\distributed_mapping_pc_windows.launch.py`"","map_dir:=$out",'rviz:=false','autosave_period:=0','odom_source:=ekf') -PassThru -WindowStyle Hidden -RedirectStandardOutput "$here\launch_ekf.log" -RedirectStandardError "$here\launch_ekf.err"
Start-Sleep -Seconds 10
pixi run --frozen ros2 node list
pixi run --frozen python -I "$here\fake_ekf.py"
& taskkill /PID $l.Id /T /F *> $null
