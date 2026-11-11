$proj = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location 'D:\ubuntu\slam'
$env:ROS_DOMAIN_ID = '78'; $env:ROS_LOCALHOST_ONLY = '1'; $env:PYTHONIOENCODING = 'utf-8'
$a = Start-Process pixi -ArgumentList @('run','--frozen','python',"`"$proj\mapping\scripts\ekf_input`"") -PassThru -WindowStyle Hidden -RedirectStandardOutput "$here\ekf_input.log" -RedirectStandardError "$here\ekf_input.err"
$b = Start-Process pixi -ArgumentList @('run','--frozen','ros2','run','robot_localization','ekf_node','--ros-args','-r','__node:=ekf_filter_node','--params-file',"`"$proj\mapping\config\ekf.yaml`"",'-r','odometry/filtered:=/ekf_odom','-r','/clock:=/ekf_clock','-p','use_sim_time:=true') -PassThru -WindowStyle Hidden -RedirectStandardOutput "$here\ekf_node.log" -RedirectStandardError "$here\ekf_node.err"
Start-Sleep -Seconds 6
pixi run --frozen python -I "$here\fake_ekf.py"
& taskkill /PID $a.Id /T /F *> $null; & taskkill /PID $b.Id /T /F *> $null
