$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
py -3.10 scripts\run_all_strict_cases.py
py -3.10 scripts\summarize_strict_results.py
