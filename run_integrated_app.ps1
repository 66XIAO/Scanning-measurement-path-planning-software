param(
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$Bootstrap = Join-Path $PSScriptRoot 'app_bootstrap.py'
$LocalConfig = Join-Path $PSScriptRoot 'config\local_environment.ps1'
if (Test-Path -LiteralPath $LocalConfig) {
    . $LocalConfig
}
$CondaEnvironment = if ($env:SCANNING_APP_CONDA_ENV) {
    $env:SCANNING_APP_CONDA_ENV
} else {
    'base'
}

$CondaCommand = Get-Command conda -ErrorAction SilentlyContinue
if (-not $CondaCommand) {
    throw 'Conda was not found on PATH. Install/activate Conda or add its Scripts directory to PATH.'
}
if (-not (Test-Path -LiteralPath $Bootstrap)) {
    throw "Application bootstrap not found: $Bootstrap"
}

Write-Host "Using Conda environment: $CondaEnvironment"
$BaseArguments = @('run', '--no-capture-output', '-n', $CondaEnvironment,
                   'python', '-X', 'faulthandler', $Bootstrap)
$PreflightArguments = $BaseArguments + @('--check')

& $CondaCommand.Source @PreflightArguments
$PreflightExitCode = $LASTEXITCODE
if ($PreflightExitCode -eq -1) {
    Write-Warning 'Native CAD/GUI dependency preflight exited unexpectedly; retrying once.'
    & $CondaCommand.Source @PreflightArguments
    $PreflightExitCode = $LASTEXITCODE
}
if ($PreflightExitCode -ne 0) {
    throw "Application environment preflight failed with exit code $PreflightExitCode."
}
if ($CheckOnly) {
    exit 0
}

& $CondaCommand.Source @($BaseArguments + @('--skip-check'))
$AppExitCode = $LASTEXITCODE
if ($AppExitCode -ne 0) {
    throw "Application start/runtime failed with exit code $AppExitCode."
}
