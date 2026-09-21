param([string]$SourceProgram = 'A')
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
$env:SPEED_V1_SOURCE_PROGRAM = $SourceProgram
$env:SPEED_V1_OUTPUT = 'artifacts/speed_v1/capture_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff')
& 'D:\Env\conda\2024\envs\test\python.exe' -B -c "import os,sys,runpy; dll=os.add_dll_directory(r'D:\Env\conda\2024\envs\Pythonocc\Library\bin'); sys.path.insert(0,r'D:\Env\conda\2024\envs\Pythonocc\Lib\site-packages'); sys.argv=['capture_speed_case.py','--source-program',os.environ['SPEED_V1_SOURCE_PROGRAM'],'--output',os.environ['SPEED_V1_OUTPUT'],'--import-test']; runpy.run_path('scripts/capture_speed_case.py',run_name='__main__')"
if ($LASTEXITCODE -ne 0) { throw 'Case capture/import failed; see the diagnostic above.' }
