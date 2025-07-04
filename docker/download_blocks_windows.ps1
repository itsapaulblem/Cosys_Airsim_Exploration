# Download AirSim Blocks Environment Binary for Windows
# This script downloads and extracts the Blocks environment without requiring WSL

param(
    [string]$BlocksDir = "blocks_env",
    [string]$BlocksUrl = "https://github.com/Cosys-Lab/Cosys-AirSim/releases/download/v1.8.1/LinuxBlocks.tar.gz"
)

Write-Host "🚀 Downloading AirSim Blocks Environment Binary (Windows)" -ForegroundColor Green
Write-Host "=======================================================" -ForegroundColor Green

# Create directory if it doesn't exist
if (-not (Test-Path $BlocksDir)) {
    New-Item -ItemType Directory -Path $BlocksDir -Force | Out-Null
    Write-Host "📁 Created directory: $BlocksDir" -ForegroundColor Cyan
}

# Check if already downloaded
if (Test-Path "$BlocksDir\LinuxBlocks") {
    Write-Host "✅ Blocks environment already exists at $BlocksDir" -ForegroundColor Green
    Write-Host "💡 To re-download, remove the directory: Remove-Item -Recurse -Force $BlocksDir" -ForegroundColor Yellow
    exit 0
}

Write-Host "📥 Downloading Blocks environment..." -ForegroundColor Cyan
$tarPath = "$BlocksDir\LinuxBlocks.tar.gz"

try {
    # Download using PowerShell
    $progressPreference = 'SilentlyContinue'  # Hide progress bar for cleaner output
    Invoke-WebRequest -Uri $BlocksUrl -OutFile $tarPath -UseBasicParsing
    $progressPreference = 'Continue'
    
    Write-Host "✅ Download completed successfully" -ForegroundColor Green
    
    # Check if we have 7-Zip, tar, or WSL available for extraction
    $extractionMethod = $null
    
    if (Get-Command "7z" -ErrorAction SilentlyContinue) {
        $extractionMethod = "7zip"
        Write-Host "🔧 Using 7-Zip for extraction" -ForegroundColor Cyan
    }
    elseif (Get-Command "tar" -ErrorAction SilentlyContinue) {
        $extractionMethod = "tar"
        Write-Host "🔧 Using tar for extraction" -ForegroundColor Cyan
    }
    elseif (Get-Command "wsl" -ErrorAction SilentlyContinue) {
        $extractionMethod = "wsl"
        Write-Host "🔧 Using WSL for extraction" -ForegroundColor Cyan
    }
    else {
        Write-Host "❌ No suitable extraction tool found" -ForegroundColor Red
        Write-Host "💡 Please install one of the following:" -ForegroundColor Yellow
        Write-Host "   - 7-Zip: https://www.7-zip.org/" -ForegroundColor Yellow
        Write-Host "   - Windows 10+ (has built-in tar)" -ForegroundColor Yellow
        Write-Host "   - WSL: wsl --install" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "🔗 Alternatively, manually download and extract:" -ForegroundColor Yellow
        Write-Host "   URL: $BlocksUrl" -ForegroundColor Yellow
        Write-Host "   Extract to: $BlocksDir\LinuxBlocks" -ForegroundColor Yellow
        exit 1
    }
    
    Write-Host "📦 Extracting archive..." -ForegroundColor Cyan
    
    switch ($extractionMethod) {
        "7zip" {
            # Extract with 7-Zip
            & 7z x $tarPath -o"$BlocksDir" -y | Out-Null
            if ($LASTEXITCODE -eq 0) {
                # 7-Zip might create a .tar file first, extract that too
                $tarFile = "$BlocksDir\LinuxBlocks.tar"
                if (Test-Path $tarFile) {
                    & 7z x $tarFile -o"$BlocksDir" -y | Out-Null
                    Remove-Item $tarFile -Force
                }
            }
        }
        "tar" {
            # Extract with built-in tar (Windows 10+)
            Set-Location $BlocksDir
            tar -xzf "LinuxBlocks.tar.gz"
            Set-Location ..
        }
        "wsl" {
            # Extract with WSL
            wsl bash -c "cd '$BlocksDir' && tar -xzf LinuxBlocks.tar.gz"
        }
    }
    
    if ($LASTEXITCODE -eq 0 -or (Test-Path "$BlocksDir\LinuxBlocks")) {
        Write-Host "✅ Extraction completed" -ForegroundColor Green
        
        # Clean up
        Remove-Item $tarPath -Force
        
        # Make the binary executable (if WSL is available)
        if (Get-Command "wsl" -ErrorAction SilentlyContinue) {
            wsl bash -c "chmod +x '$BlocksDir/LinuxBlocks/Linux/Blocks.sh'" 2>$null
        }
        
        Write-Host "🎉 Blocks environment ready!" -ForegroundColor Green
        Write-Host "📁 Location: $BlocksDir\LinuxBlocks" -ForegroundColor Cyan
        Write-Host "🚀 Run script: $BlocksDir\LinuxBlocks\Linux\Blocks.sh" -ForegroundColor Cyan
        
        Write-Host ""
        Write-Host "🐳 Next steps:" -ForegroundColor Cyan
        Write-Host "1. Start VcXsrv X11 server" -ForegroundColor White
        Write-Host "2. Run: .\run_full_stack_windows.ps1" -ForegroundColor White
    }
    else {
        Write-Host "❌ Failed to extract archive" -ForegroundColor Red
        Write-Host "💡 Please try manual extraction or check the extraction tool" -ForegroundColor Yellow
        exit 1
    }
}
catch {
    Write-Host "❌ Failed to download Blocks environment" -ForegroundColor Red
    Write-Host "🔗 URL: $BlocksUrl" -ForegroundColor Yellow
    Write-Host "❌ Error: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
} 