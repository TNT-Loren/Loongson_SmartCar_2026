$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$main = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\user\main.cpp")
$tcp = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\tcp.cpp")
$tcpHeader = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\tcp.hpp")
$scheduler = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\scheduler.cpp")
$modeHeader = Join-Path $root "project\code\tcp_camera_mode.hpp"

function Assert-True([bool]$condition, [string]$message)
{
    if (-not $condition)
    {
        throw $message
    }
}

Assert-True ($main.Contains('tcp_image_transmission_init("192.168.31.20", 8086)')) `
    "main.cpp must keep the TCP address as a directly editable literal"
Assert-True (-not $main.Contains("SMARTCAR_CAMERA_TRANSPORT_MODE")) `
    "main.cpp must not contain camera transport preprocessor branches"
Assert-True (-not $main.Contains("tcp_camera_publish_frame")) `
    "main.cpp must not publish frames through a double buffer"
Assert-True ($main.Contains("tcp_camera_update_vs_image();")) `
    "main.cpp must delegate VS image preparation to tcp.cpp"

Assert-True ($tcp.Contains("g_tcp_camera_image[0]")) `
    "tcp.cpp must retain the original track debug image buffer"
Assert-True ($tcp.Contains("g_vs.image_copy[0]")) `
    "tcp.cpp must bind the teammate VS RGB565 matrix"
Assert-True ($tcp.Contains("seekfree_assistant_camera_information_config")) `
    "tcp.cpp must configure the selected matrix directly"
Assert-True ($tcp -match "#define\s+TCP_IMAGE_SOURCE\s+(0|1|TCP_IMAGE_SOURCE_TRACK|TCP_IMAGE_SOURCE_VS)(\s|$)") `
    "tcp.cpp must expose one directly editable matrix-source macro"
Assert-True (-not $tcp.Contains("TcpCameraFrameBuffer")) `
    "tcp.cpp must not use the double-buffer implementation"
Assert-True (-not (Test-Path $modeHeader)) `
    "the old transport-mode helper header must be removed"
Assert-True (-not $tcpHeader.Contains("tcp_camera_publish_frame")) `
    "tcp.hpp must not expose the double-buffer publish API"
Assert-True ($scheduler.Contains("seekfree_assistant_camera_send();")) `
    "the original background sender must remain in scheduler.cpp"

Write-Host "TCP camera structure checks passed"
