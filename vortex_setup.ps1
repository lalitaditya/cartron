# vortex_setup.ps1
# Pre-flight check and environment setup for Aria -> Piper teleoperation.

$V_ID = "1D50"
$CAN_PID = "606F" # candleLight / GS_USB (Renamed from $PID to avoid PS conflict)
$UDP_PORT = 5010

Write-Host "=== Vortex Teleop Pre-flight Setup ===" -ForegroundColor Cyan

# 1. Check for CAN Adapter
Write-Host "[1/4] Checking for GS_USB CAN Adapter (VID:$V_ID PID:$CAN_PID)..."
$dev = Get-PnpDevice | Where-Object { $_.InstanceId -like "*VID_$V_ID&PID_$CAN_PID*" }

if ($null -eq $dev) {
    Write-Host "  FAILED: No candleLight/GS_USB adapter found. Please plug it in." -ForegroundColor Red
} else {
    $driver = Get-PnpDeviceProperty -InstanceId $dev.InstanceId -KeyName "DEVPKEY_Device_Service" | Select-Object -ExpandProperty Data
    Write-Host "  FOUND: $($dev.FriendlyName)" -ForegroundColor Green
    Write-Host "  Driver: $driver"
    
    if ($driver -ne "WinUSB") {
        Write-Host "  WARNING: Adapter is NOT using WinUSB driver. Use Zadig to change it now!" -ForegroundColor Yellow
    }
}

# 2. Firewall Rule for Hand Tracking (UDP 5010)
Write-Host "[2/4] Checking Firewall rule for Aria Hand Tracking (UDP $UDP_PORT)..."
$rule = Get-NetFirewallRule -DisplayName "Aria Hand Tracker" -ErrorAction SilentlyContinue
if ($null -eq $rule) {
    Write-Host "  Creating Firewall rule to allow hand data from WSL..." -ForegroundColor Yellow
    New-NetFirewallRule -DisplayName "Aria Hand Tracker" -Direction Inbound -Action Allow -Protocol UDP -LocalPort $UDP_PORT -Profile Any
    Write-Host "  DONE." -ForegroundColor Green
} else {
    Write-Host "  Rule already exists." -ForegroundColor Green
}

# 3. Check WSL IP address
Write-Host "[3/4] Checking WSL connectivity..."
$wsl_ip = wsl hostname -I
if ($null -eq $wsl_ip) {
    Write-Host "  FAILED: Could not find WSL IP. Make sure WSL is running." -ForegroundColor Red
} else {
    Write-Host "  WSL IP: $wsl_ip" -ForegroundColor Green
}

# 4. Check for FastDDS Unicast Profile
Write-Host "[4/4] Checking FastDDS configuration..."
$xml_path = Join-Path (Get-Location) "fastdds_aria_unicast.xml"
if (Test-Path $xml_path) {
    $env:FASTRTPS_DEFAULT_PROFILES_FILE = $xml_path
    Write-Host "  FastDDS Unicast profile verified at $xml_path" -ForegroundColor Green
} else {
    Write-Host "  WARNING: fastdds_aria_unicast.xml not found in current directory." -ForegroundColor Yellow
}

Write-Host "`nReady for teleoperation!" -ForegroundColor Cyan
Write-Host "Run the bridge with: python aria_teleop/aria_piper_bridge.py --udp --can --mirror"
