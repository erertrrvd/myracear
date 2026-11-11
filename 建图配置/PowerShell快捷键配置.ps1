# Shortcuts: a line that is just "2" / "3" / "4" runs the mapping command below
#   2 = high-speed mapping with wheel + IMU odometry poses; laser only draws the map; 3 = save;
#   4 = high-speed mapping (~3 m/s, two laps): -Fast = mapper_fast.yaml, loop search 8 m (live solid: type -LiveSolid by hand)
#   a / b / c / d = save + fit to track A / B / C / D (3 alone = pick the closest track automatically)
if (Get-Module -ListAvailable PSReadLine) {
    Set-PSReadLineKeyHandler -Key Enter -ScriptBlock {
        $shortcuts = @{
            '2' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Mode filtered -Fast -OdomOnly'
            '3' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Save'
            '4' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Mode filtered -Fast'
            'a' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Save -Track A'
            'b' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Save -Track B'
            'c' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Save -Track C'
            'd' = 'Set-Location D:\ubuntu\slam; .\run_slam_windows.bat -Save -Track D'
        }
        $line = $null; $cursor = $null
        [Microsoft.PowerShell.PSConsoleReadLine]::GetBufferState([ref]$line, [ref]$cursor)
        $key = $line.Trim()
        if ($shortcuts.ContainsKey($key)) {
            [Microsoft.PowerShell.PSConsoleReadLine]::Replace(0, $line.Length, $shortcuts[$key])
        }
        [Microsoft.PowerShell.PSConsoleReadLine]::AcceptLine()
    }
}
