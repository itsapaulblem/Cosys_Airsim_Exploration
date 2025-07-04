# Download AirSim Blocks Environment Binary for Windows (Simple Version)

param(
    [string]$BlocksDir = "blocks_env",
    [string]$BlocksUrl = "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/Blocks.zip"
)

Write-Host "Downloading AirSim Blocks Environment Binary" -ForegroundColor Green
Write-Host "==============================================" -ForegroundColor Green

# Create directory if it doesn't exist
if (-not (Test-Path $BlocksDir)) {
    New-Item -ItemType Directory -Path $BlocksDir -Force | Out-Null
    Write-Host "Created directory: $BlocksDir" -ForegroundColor Cyan
}

# Check if already downloaded
if (Test-Path "$BlocksDir\LinuxBlocks") {
    Write-Host "OK: Blocks environment already exists at $BlocksDir" -ForegroundColor Green
    exit 0
}

Write-Host "Downloading Blocks environment..." -ForegroundColor Cyan
$zipPath = "$BlocksDir\Blocks.zip"

try {
    # Download using PowerShell
    $progressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $BlocksUrl -OutFile $zipPath -UseBasicParsing
    $progressPreference = 'Continue'
    
    Write-Host "OK: Download completed successfully" -ForegroundColor Green
    
    Write-Host "Extracting archive..." -ForegroundColor Cyan
    
    # Use PowerShell Expand-Archive for ZIP files
    Expand-Archive -Path $zipPath -DestinationPath $BlocksDir -Force
    
    # Check for the extracted directory structure
    $extractedPath = "$BlocksDir\Linux"
    if (Test-Path $extractedPath) {
        Write-Host "OK: Archive extracted to $extractedPath" -ForegroundColor Green
        
        # Move to expected location
        Move-Item $extractedPath "$BlocksDir\LinuxBlocks" -Force
        
        # Clean up
        Remove-Item $zipPath -Force
        
        Write-Host "SUCCESS: Blocks environment ready!" -ForegroundColor Green
        Write-Host "Location: $BlocksDir\LinuxBlocks" -ForegroundColor Cyan
        
        Write-Host ""
        Write-Host "Next steps:" -ForegroundColor Cyan
        Write-Host "1. Ensure VcXsrv X11 server is running" -ForegroundColor White
        Write-Host "2. Run: .\run_full_stack_windows_simple.ps1" -ForegroundColor White
    } else {
        Write-Host "ERROR: Failed to extract archive - expected directory not found" -ForegroundColor Red
        Write-Host "Looking for: $extractedPath" -ForegroundColor Red
        exit 1
    }
}
catch {
    Write-Host "ERROR: Failed to download Blocks environment" -ForegroundColor Red
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host ""
    Write-Host "Manual download instructions:" -ForegroundColor Yellow
    Write-Host "1. Download: $BlocksUrl" -ForegroundColor Yellow
    Write-Host "2. Extract to: $BlocksDir\LinuxBlocks" -ForegroundColor Yellow
    exit 1
} 