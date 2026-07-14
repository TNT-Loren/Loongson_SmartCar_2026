$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$main = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\user\main.cpp")
$scheduler = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\scheduler.cpp")
$pidHeader = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\Control\Control_Header\pid.hpp")
$speedHeader = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\speed_strategy.hpp")
$menuSource = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\code\Control\Menu\common_MYmenu.cpp")
$cmake = Get-Content -Raw -Encoding UTF8 (Join-Path $root "project\user\CMakeLists.txt")

function Assert-True([bool]$condition, [string]$message)
{
    if (-not $condition)
    {
        throw $message
    }
}

Assert-True ($main.Contains('#include "common_MYmenu.hpp"')) `
    "main.cpp must include the menu runtime API"
Assert-True ($main.Contains("Menu_Init();")) `
    "main.cpp must initialize the menu before starting the scheduler"
Assert-True ($main.Contains("Menu_Task();")) `
    "main.cpp must poll the menu from its main loop"
Assert-True (([regex]::Matches($main, "Menu_Force_Stop\(\);")).Count -ge 2) `
    "main.cpp must force STOP from both signal and cleanup paths"

Assert-True ($scheduler.Contains('#include "common_MYmenu.hpp"')) `
    "scheduler.cpp must include the menu runtime API"
Assert-True ($scheduler.Contains("if (!Menu_Car_Enabled())")) `
    "scheduler.cpp must gate the 10ms control loop on the menu RUN state"
Assert-True ($scheduler.Contains("base_start_speed = 0.0f;")) `
    "the STOP branch must reset the speed ramp"
Assert-True ($scheduler.Contains("pid_left.clear();") -and
             $scheduler.Contains("pid_right.clear();") -and
             $scheduler.Contains("pid_angle.clear();")) `
    "the STOP branch must clear all PID state"
Assert-True ($scheduler.Contains("motor_set_speed(0, 0);")) `
    "the STOP branch must command zero motor output"

foreach ($accessor in @("kp_ptr", "ki_ptr", "kd_ptr", "output_limit_ptr", "integral_limit_ptr", "deadband_ptr"))
{
    Assert-True ($pidHeader.Contains($accessor)) "pid.hpp is missing $accessor"
}
foreach ($parameter in @("k_speed_up_step", "k_speed_down_step", "k_large_alpha_slowdown_deg"))
{
    Assert-True ($speedHeader.Contains("extern float $parameter;")) `
        "speed_strategy.hpp is missing $parameter"
}

Assert-True ($menuSource.Contains("std::atomic<bool> g_car_enabled{false};")) `
    "the car must default to an atomic STOP state"
Assert-True ($menuSource.Contains("g_menu_disabled_until_restart = true;")) `
    "starting the car must permanently disable the menu until restart"
Assert-True ($cmake.Contains("../code/Control/Menu")) `
    "CMake must include and compile the Control/Menu directory"

Write-Host "Menu structure checks passed"
