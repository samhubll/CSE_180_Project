[CmdletBinding()]
param()

. (Join-Path $PSScriptRoot 'ROS-Container.ps1')

try {
    Initialize-ProjectContainers
    Write-Host 'ROS 2 Jazzy project terminal. Type exit to close it.'
    & $script:ProjectDocker exec -it --workdir /MRTP/CSE180 mrtp-jazzy /bin/bash --init-file /MRTP/CSE180/src/CSE_180_Project/scripts/interactive.bash
    exit $LASTEXITCODE
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
