# Aria Glasses USB Setup (Run as Admin in PowerShell)
# Finds the Aria glasses and attaches them to WSL.

Write-Host "Searching for Aria glasses..." -ForegroundColor Cyan
$usbList = usbipd list
$ariaLine = $usbList | Select-String "Aria"

if ($ariaLine) {
    $busid = ($ariaLine.ToString() -split "\s+")[0]
    Write-Host "Found Aria glasses on BUSID: $busid" -ForegroundColor Green
    
    Write-Host "Binding device (needs Admin)..."
    usbipd bind --busid $busid
    
    Write-Host "Attaching to WSL..."
    usbipd attach --wsl --busid $busid
    
    Write-Host "Successfully attached! Now check 'adb devices' in WSL." -ForegroundColor Green
} else {
    Write-Host "Aria glasses not found in 'usbipd list'. Check USB cable." -ForegroundColor Red
}
