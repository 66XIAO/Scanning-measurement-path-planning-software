$ErrorActionPreference = 'Stop'

$TestPrefix = 'D:\Env\conda\2024\envs\test'
$PythonOccPrefix = 'D:\Env\conda\2024\envs\Pythonocc'
$Bootstrap = Join-Path $PSScriptRoot 'app_bootstrap.py'

if (-not (Test-Path -LiteralPath (Join-Path $TestPrefix 'python.exe'))) {
    throw "Working Conda interpreter not found: $TestPrefix"
}
if (-not (Test-Path -LiteralPath (Join-Path $PythonOccPrefix 'Lib\site-packages\OCC'))) {
    throw "Existing Pythonocc packages not found: $PythonOccPrefix"
}

$env:CONDA_PREFIX = $TestPrefix
$env:PYTHONOCC_PREFIX = $PythonOccPrefix
$env:ROBODK_API_PATH = 'D:\RoboDK\Python37\lib\site-packages'
$env:QT_PLUGIN_PATH = Join-Path $PythonOccPrefix 'Library\lib\qt6\plugins'
$env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $env:QT_PLUGIN_PATH 'platforms'
$env:PATH = "$TestPrefix;$TestPrefix\Library\bin;$TestPrefix\Scripts;$env:PATH"

& (Join-Path $TestPrefix 'python.exe') $Bootstrap
