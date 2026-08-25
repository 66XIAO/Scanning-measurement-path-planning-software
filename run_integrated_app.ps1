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
$Arguments = @('run', '--no-capture-output', '-n', $CondaEnvironment, 'python', $Bootstrap)
if ($CheckOnly) {
    $Arguments += '--check'
}

& $CondaCommand.Source @Arguments
$AppExitCode = $LASTEXITCODE
if ($AppExitCode -ne 0) {
    throw "Application environment check/start failed with exit code $AppExitCode."
}
