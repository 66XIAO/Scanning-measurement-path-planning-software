param(
    [switch]$CheckOnly,
    [string]$ValidateModel,
    [int]$GridU = 5,
    [int]$GridV = 7
)

$ErrorActionPreference = 'Stop'
$Bootstrap = Join-Path $PSScriptRoot 'app_bootstrap.py'
$LocalConfig = Join-Path $PSScriptRoot 'config\local_environment.ps1'
if (Test-Path -LiteralPath $LocalConfig) {
    . $LocalConfig
}
if (-not (Test-Path -LiteralPath $Bootstrap)) {
    throw "Application bootstrap not found: $Bootstrap"
}

if ($CheckOnly -and $ValidateModel) {
    throw 'CheckOnly and ValidateModel cannot be used together.'
}
if ($env:SCANNING_APP_PYTHON_EXE) {
    $PythonPath = $env:SCANNING_APP_PYTHON_EXE
    if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) {
        throw "Configured Python interpreter not found: $PythonPath"
    }
    $PythonPath = (Resolve-Path -LiteralPath $PythonPath).Path
    $PythonPrefix = Split-Path -Parent $PythonPath
    $env:PATH = "$PythonPrefix;$(Join-Path $PythonPrefix 'Library\bin');$(Join-Path $PythonPrefix 'Scripts');$env:PATH"
    $env:CONDA_PREFIX = $PythonPrefix
    $RuntimeCommand = $PythonPath
    $BaseArguments = @('-X', 'faulthandler', $Bootstrap)
    Write-Host "Using Python interpreter: $PythonPath"
} else {
    $CondaEnvironment = if ($env:SCANNING_APP_CONDA_ENV) {
        $env:SCANNING_APP_CONDA_ENV
    } else { 'base' }
    $CondaCommand = Get-Command conda -ErrorAction SilentlyContinue
    if (-not $CondaCommand) {
        throw 'Conda was not found on PATH. Configure SCANNING_APP_PYTHON_EXE or activate Conda.'
    }
    $RuntimeCommand = $CondaCommand.Source
    $BaseArguments = @('run', '--no-capture-output', '-n', $CondaEnvironment,
                       'python', '-X', 'faulthandler', $Bootstrap)
    Write-Host "Using Conda environment: $CondaEnvironment"
}
$PreflightArguments = $BaseArguments + @('--check')

& $RuntimeCommand @PreflightArguments
$PreflightExitCode = $LASTEXITCODE
if ($PreflightExitCode -eq -1) {
    Write-Warning 'Native CAD/GUI dependency preflight exited unexpectedly; retrying once.'
    & $RuntimeCommand @PreflightArguments
    $PreflightExitCode = $LASTEXITCODE
}
if ($PreflightExitCode -ne 0) {
    throw "Application environment preflight failed with exit code $PreflightExitCode."
}
if ($CheckOnly) {
    exit 0
}

if ($ValidateModel) {
    & $RuntimeCommand @($BaseArguments + @('--skip-check', '--check-model', $ValidateModel,
                                          '--u', $GridU, '--v', $GridV))
} else {
    & $RuntimeCommand @($BaseArguments + @('--skip-check'))
}
$AppExitCode = $LASTEXITCODE
if ($AppExitCode -ne 0) {
    throw "Application start/runtime failed with exit code $AppExitCode."
}
