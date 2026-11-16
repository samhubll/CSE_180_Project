[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateSet('build', 'warehouse', 'detector', 'search', 'topics', 'teleop')]
    [string]$Action
)

. (Join-Path $PSScriptRoot 'ROS-Container.ps1')

try {
    Initialize-ProjectContainers
    & $script:ProjectDocker exec -it --workdir /MRTP/CSE180 mrtp-jazzy /bin/bash /MRTP/CSE180/src/CSE_180_Project/scripts/project.sh $Action
    exit $LASTEXITCODE
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
