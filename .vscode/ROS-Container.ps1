$ErrorActionPreference = 'Stop'

function Resolve-ProjectDocker {
    $dockerCommand = Get-Command docker -CommandType Application -ErrorAction SilentlyContinue
    if ($dockerCommand) { return $dockerCommand.Source }

    foreach ($dockerPath in @(
        (Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\resources\bin\docker.exe'),
        'C:\Program Files\Docker\Docker\resources\bin\docker.exe'
    )) {
        if (Test-Path -LiteralPath $dockerPath) { return $dockerPath }
    }

    throw 'Docker Desktop was not found. Install and open Docker Desktop, then open this terminal again.'
}

function Get-ProjectDockerValue([string[]]$DockerArguments) {
    $savedPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $dockerValue = & $script:ProjectDocker @DockerArguments 2>$null
        $dockerExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $savedPreference
    }
    return @{
        ExitCode = $dockerExitCode
        Text = ($dockerValue | Out-String).Trim()
    }
}

function Initialize-ProjectContainers {
    $script:ProjectDocker = Resolve-ProjectDocker
    $engineType = Get-ProjectDockerValue -DockerArguments @('info', '--format', '{{.OSType}}')
    if ($engineType.ExitCode -ne 0) {
        throw 'Docker is unavailable. Open Docker Desktop and wait until its engine is running, then retry this terminal or task.'
    }
    if ($engineType.Text -ne 'linux') {
        throw 'ROS needs Linux containers. In Docker Desktop switch to Linux containers, then retry.'
    }

    foreach ($containerName in @('mrtp-jazzy', 'novnc')) {
        $containerState = Get-ProjectDockerValue -DockerArguments @('container', 'inspect', '--format', '{{.State.Status}}', $containerName)
        if ($containerState.ExitCode -ne 0) {
            throw "The saved '$containerName' container is missing. Run the course setup at outputs\ROS-Class.ps1 -Action Start -NoBrowser first."
        }
        if ($containerState.Text -ne 'running') {
            & $script:ProjectDocker start $containerName
            if ($LASTEXITCODE -ne 0) {
                throw "Could not start '$containerName'. Check Docker Desktop, then retry."
            }
        }
    }
}
