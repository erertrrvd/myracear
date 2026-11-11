# 电脑端远程视觉入口：连小车 -> 收摄像头画面 -> 本机识别 -> 把结果传回小车。
# 用法（Windows PowerShell）：
#   powershell -ExecutionPolicy Bypass -File .\start_pc_vision.ps1
#   powershell -ExecutionPolicy Bypass -File .\start_pc_vision.ps1 -Car 192.168.5.100 -Port 9000
#   加 -NoShow 不弹窗。窗口里按 q 退出。
# 小车上要先运行：bash start_remote_vision.sh
param(
  [string]$Car = "192.168.5.100",
  [int]$Port = 9000,
  [switch]$NoShow
)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$py = "D:\light_ai\venv\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Host "找不到 $py（装了 onnxruntime 和 torch 的 Python）"; exit 1 }

$t = Test-NetConnection -ComputerName $Car -Port $Port -WarningAction SilentlyContinue
if (-not $t.TcpTestSucceeded) {
  Write-Host "连不上 ${Car}:${Port}。请确认："
  Write-Host "  1. 小车上已运行 bash start_remote_vision.sh（并且没有报错退出）"
  Write-Host "  2. 电脑连着小车的 WiFi，IP 在 192.168.5.x 网段，没有开 VPN"
  Write-Host "  3. 端口号一致"
  exit 1
}
$extra = @()
if ($NoShow) { $extra += "--no-show" }
& $py (Join-Path $here "pc_vision_client.py") --car $Car --port $Port @extra
