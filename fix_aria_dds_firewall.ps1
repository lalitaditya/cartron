# fix_aria_dds_firewall.ps1
# ============================================================
# Run this script as Administrator (right-click → Run as Admin)
# It adds Hyper-V firewall rules so DDS/UDP traffic from the
# Aria glasses can reach WSL2.
# ============================================================

#Requires -RunAsAdministrator

$VMCreatorId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
$RuleName    = 'Aria DDS UDP Inbound Allow'

Write-Host "=== Aria DDS Firewall Fix for WSL2 ===" -ForegroundColor Cyan

# ── 1. Check existing rules ──────────────────────────────────────────────────
$existing = Get-NetFirewallHyperVRule -VMCreatorId $VMCreatorId -ErrorAction SilentlyContinue |
    Where-Object { $_.DisplayName -eq $RuleName }

if ($existing) {
    Write-Host "[OK] Rule '$RuleName' already exists. Skipping creation." -ForegroundColor Green
} else {
    # Allow ALL inbound UDP to WSL2 (DDS uses multicast + ephemeral ports)
    New-NetFirewallHyperVRule `
        -Name        'AriaStreamDDS_UDP_In' `
        -DisplayName $RuleName `
        -Direction   Inbound `
        -Action      Allow `
        -Protocol    UDP `
        -VMCreatorId $VMCreatorId

    Write-Host "[ADDED] Hyper-V firewall rule: allow all inbound UDP to WSL2" -ForegroundColor Green
}

# ── 2. Also add a TCP rule for DDS discovery (some implementations use TCP) ──
$TcpRuleName = 'Aria DDS TCP Inbound Allow'
$existingTcp = Get-NetFirewallHyperVRule -VMCreatorId $VMCreatorId -ErrorAction SilentlyContinue |
    Where-Object { $_.DisplayName -eq $TcpRuleName }

if ($existingTcp) {
    Write-Host "[OK] Rule '$TcpRuleName' already exists." -ForegroundColor Green
} else {
    New-NetFirewallHyperVRule `
        -Name        'AriaStreamDDS_TCP_In' `
        -DisplayName $TcpRuleName `
        -Direction   Inbound `
        -Action      Allow `
        -Protocol    TCP `
        -VMCreatorId $VMCreatorId

    Write-Host "[ADDED] Hyper-V firewall rule: allow all inbound TCP to WSL2" -ForegroundColor Green
}

# ── 3. Also ensure Windows Firewall allows UDP on DDS ports ──────────────────
$WinRuleName = 'Aria DDS Streaming (UDP 7000-9000)'
$existingWin = Get-NetFirewallRule -DisplayName $WinRuleName -ErrorAction SilentlyContinue
if (-not $existingWin) {
    New-NetFirewallRule `
        -DisplayName $WinRuleName `
        -Direction   Inbound `
        -Action      Allow `
        -Protocol    UDP `
        -LocalPort   '7000-9000' `
        -Profile     Any
    Write-Host "[ADDED] Windows Firewall rule: allow UDP 7000-9000 inbound" -ForegroundColor Green
} else {
    Write-Host "[OK] Windows Firewall rule '$WinRuleName' already exists." -ForegroundColor Green
}

# ── 4. Show final state ──────────────────────────────────────────────────────
Write-Host "`n=== Current Hyper-V Firewall Rules for WSL ==="  -ForegroundColor Cyan
Get-NetFirewallHyperVRule -VMCreatorId $VMCreatorId |
    Format-Table -Property DisplayName, Direction, Action, Protocol -AutoSize

Write-Host @"

=== Next Steps ===
1. Restart WSL:  wsl --shutdown
2. Reopen your WSL terminal
3. Run:  python3 aria_debug_stream.py
"@ -ForegroundColor Yellow
