$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$main = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\user\main.cpp")
$vsSource = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\vs_inference.cpp")
$vsHeader = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\vs_inference.hpp")
$tcp = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\tcp.cpp")

function Assert-True([bool]$condition, [string]$message)
{
    if (-not $condition)
    {
        throw $message
    }
}

Assert-True (-not $main.Contains("VSInference g_vs;")) `
    "main.cpp must not define the global VS inference object"
Assert-True ($vsSource.Contains("VSInference g_vs;")) `
    "vs_inference.cpp must own the global VS inference object"
Assert-True ($vsHeader.Contains("extern VSInference g_vs;")) `
    "vs_inference.hpp must declare the shared VS inference object"
Assert-True (-not $tcp.Contains("extern VSInference g_vs;")) `
    "tcp.cpp must use the declaration from vs_inference.hpp"

Assert-True ($vsHeader -match "bool\s+consume_new_result\(std::string& result\);") `
    "VSInference must expose a one-shot result consumption API"
Assert-True ($main.Contains("if (g_vs.consume_new_result(result))")) `
    "main.cpp must only consume each result once before mapping it to an action"
Assert-True (-not $main.Contains("g_vs.get_result()")) `
    "main.cpp must not manually read the VS result"
Assert-True (-not $main.Contains("g_vs.clear_result()")) `
    "main.cpp must not manually clear the VS result"
Assert-True (-not $main.Contains("[TIMING]")) `
    "the inaccurate result-to-trigger timing output must be removed"
Assert-True ($vsSource -match "if\s*\(lost_cnt\s*==\s*0\)\s*\{\s*lost_since\s*=\s*std::chrono::steady_clock::now\(\);") `
    "lost-object timing must begin on the first missing frame"

Assert-True ($vsHeader -match "int\s+take_fps_count\(\)\s*;") `
    "VSInference must expose a one-second FPS counter read-and-reset API"
Assert-True ($main.Contains("g_vs.take_fps_count()")) `
    "main.cpp must print VS FPS from the existing one-second scheduler branch"
Assert-True (-not $main.Contains("vs_fps_last")) `
    "main.cpp must not keep a separate VS wall-clock timer"

Write-Host "VS runtime structure checks passed"
