# watch_aria_usb.ps1
# Monitors for Aria UsbNcm device (VID_2833&PID_0300) and auto-attaches to WSL
# Run this in an ADMIN PowerShell BEFORE starting streaming from WSL
#
# The flow:
# 1. WSL calls start_streaming() -> glasses re-enumerate from 0083 -> 0300
# 2. This script detects the 0300 device on Windows
# 3. Binds + attaches it to WSL
# 4. WSL gets the cdc_ncm network interface -> DDS data flows

Write-Host "=== Aria USB Watcher ===" -ForegroundColor Cyan
Write-Host "Monitoring for UsbNcm device (VID_2833&PID_0300)..."
Write-Host "Press Ctrl+C to stop."
Write-Host ""

# First, make sure the normal ADB device is attached
try {
    $state = usbipd state 2>$null | ConvertFrom-Json
    $adb = $state.Devices | Where-Object { $_.InstanceId -like "*VID_2833*PID_0083*" }
    if ($adb -and $null -eq $adb.ClientIPAddress) {
        Write-Host "[Info] ADB device found at busid $($adb.BusId), attaching to WSL..." -ForegroundColor Yellow
        usbipd bind --busid $adb.BusId --force 2>$null
        usbipd attach --busid $adb.BusId --wsl 2>$null
        Write-Host "[OK] ADB device attached." -ForegroundColor Green
    } elseif ($adb) {
        Write-Host "[OK] ADB device already attached at busid $($adb.BusId)." -ForegroundColor Green
    } else {
        Write-Host "[Warn] ADB device (0083) not found. Is the Aria plugged in?" -ForegroundColor Yellow
    }
} catch {
    Write-Host "[Error] Could not check ADB device: $_" -ForegroundColor Red
}

$ncmAttached = $false
$lastBusId = ""

while ($true) {
    try {
        $state = usbipd state 2>$null | ConvertFrom-Json
        if (-not $state) {
            Start-Sleep -Seconds 1
            continue
        }

        # Look for UsbNcm device by InstanceId (contains VID_2833&PID_0300)
        $ncm = $state.Devices | Where-Object { $_.InstanceId -like "*VID_2833*PID_0300*" }
        
        if ($ncm -and $null -ne $ncm.BusId) {
            # Device is physically connected (has a BusId)
            if ($ncm.BusId -ne $lastBusId) {
                Write-Host "[$(Get-Date -Format 'HH:mm:ss')] UsbNcm device appeared at busid $($ncm.BusId)" -ForegroundColor Yellow
                $lastBusId = $ncm.BusId
            }

            if ($null -eq $ncm.ClientIPAddress) {
                # Not attached to WSL yet - bind and attach
                Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Binding and attaching UsbNcm to WSL..." -ForegroundColor Yellow
                usbipd bind --busid $ncm.BusId --force 2>$null
                Start-Sleep -Milliseconds 300
                $result = usbipd attach --busid $ncm.BusId --wsl 2>&1
                if ($LASTEXITCODE -eq 0) {
                    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] UsbNcm ATTACHED to WSL! (busid $($ncm.BusId))" -ForegroundColor Green
                    $ncmAttached = $true
                } else {
                    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Attach failed: $result" -ForegroundColor Red
                }
            } elseif (-not $ncmAttached) {
                Write-Host "[$(Get-Date -Format 'HH:mm:ss')] UsbNcm already attached to WSL." -ForegroundColor Green
                $ncmAttached = $true
            }
        } else {
            # Device not physically connected (BusId is null)
            if ($ncmAttached) {
                Write-Host "[$(Get-Date -Format 'HH:mm:ss')] UsbNcm device disconnected." -ForegroundColor Yellow
                $ncmAttached = $false
                $lastBusId = ""
            }
        }
    } catch {
        Write-Host "[Error] $($_.Exception.Message)" -ForegroundColor Red
    }
    
    Start-Sleep -Milliseconds 500
}
