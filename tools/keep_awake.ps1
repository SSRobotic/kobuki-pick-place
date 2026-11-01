# Keep the screen and system awake while this window is open (like a video player does).
# Windows Modern Standby otherwise cuts the robot's USB devices when the laptop idles. Ctrl+C to stop.
Add-Type -Namespace Win32 -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint esFlags);'
$ES_CONTINUOUS = [uint32]2147483648; $ES_SYSTEM_REQUIRED = [uint32]1; $ES_DISPLAY_REQUIRED = [uint32]2
Write-Host "Keeping the laptop awake (screen on) while this tab runs. Ctrl+C to stop." -ForegroundColor Yellow
try {
    while ($true) {
        [void][Win32.Power]::SetThreadExecutionState($ES_CONTINUOUS -bor $ES_SYSTEM_REQUIRED -bor $ES_DISPLAY_REQUIRED)
        Start-Sleep -Seconds 30
    }
} finally {
    [void][Win32.Power]::SetThreadExecutionState($ES_CONTINUOUS)
    Write-Host "Released."
}
