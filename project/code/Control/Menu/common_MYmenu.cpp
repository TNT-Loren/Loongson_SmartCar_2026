#include "common_MYmenu.hpp"

#include "beep.hpp"
#include "common_menu.hpp"
#include "pid.hpp"
#include "speed_strategy.hpp"
#include "zf_common_headfile.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>

namespace
{
zf_device_ips200 g_ips200;
zf_driver_gpio g_key1(ZF_GPIO_KEY_1);
zf_driver_gpio g_key2(ZF_GPIO_KEY_2);
zf_driver_gpio g_key3(ZF_GPIO_KEY_3);
zf_driver_gpio g_key4(ZF_GPIO_KEY_4);
zf_driver_gpio g_key5_hall(ZF_GPIO_HALL_DETECTION);

Menu_Item g_head;
Menu_Item *g_page = nullptr;
Menu_Item *g_first_item = nullptr;
Menu_Item *g_selected_item = nullptr;

std::atomic<bool> g_car_enabled{false};
bool g_start_display = false;
bool g_menu_disabled_until_restart = false;

std::array<std::uint8_t, 5> g_last_key_level{{1, 1, 1, 1, 1}};
std::array<std::chrono::steady_clock::time_point, 5> g_last_press_time{};
auto g_last_draw_time = std::chrono::steady_clock::time_point::min();

constexpr auto k_key_debounce = std::chrono::milliseconds(40);
constexpr auto k_screen_refresh = std::chrono::milliseconds(50);

bool key_pressed_once(zf_driver_gpio &key, std::size_t index)
{
    const std::uint8_t level = key.get_level();
    const bool falling_edge = (g_last_key_level[index] == 1 && level == 0);
    g_last_key_level[index] = level;

    if (!falling_edge)
    {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - g_last_press_time[index] < k_key_debounce)
    {
        return false;
    }
    g_last_press_time[index] = now;
    return true;
}

void sync_start_switch(void)
{
    g_car_enabled.store(g_start_display);
}

void show_menu_value(Menu_Item *item, int row)
{
    if (item == nullptr || item->data == nullptr)
    {
        return;
    }

    const int y = (row + 1) * 20;
    g_ips200.show_string(140, y, item->select ? ">" : " ");

    switch (item->kind)
    {
    case float_box:
        g_ips200.show_float(150, y, *static_cast<float *>(item->data), 5, 2);
        break;
    case int32_box:
        g_ips200.show_int(150, y, *static_cast<std::int32_t *>(item->data), 6);
        break;
    case int8_box:
        g_ips200.show_int(150, y, *static_cast<std::int8_t *>(item->data), 3);
        break;
    case bool_box:
        g_ips200.show_char(150, y, *static_cast<bool *>(item->data) ? 'Y' : 'N');
        break;
    default:
        break;
    }
}

void show_menu(void)
{
    g_ips200.clear();
    if (g_page == nullptr)
    {
        return;
    }

    g_ips200.show_string(0, 0, g_page->name);
    g_ips200.show_string(210, 0, Menu_Car_Enabled() ? "RUN" : "STOP");

    Menu_Item *item = g_first_item;
    for (int row = 0; row < g_page->sons && item != nullptr && row < 11; ++row)
    {
        g_ips200.show_string(0, (row + 1) * 20, item == g_selected_item ? "->" : "  ");
        g_ips200.show_string(16, (row + 1) * 20, item->name);
        show_menu_value(item, row);
        item = item->next_brother;
    }
}

void adjust_selected(float direction)
{
    if (g_selected_item == nullptr || g_selected_item->data == nullptr || Menu_Car_Enabled())
    {
        return;
    }

    switch (g_selected_item->kind)
    {
    case float_box:
    {
        float &value = *static_cast<float *>(g_selected_item->data);
        value = std::clamp(value + direction * g_selected_item->step,
                           g_selected_item->min_value,
                           g_selected_item->max_value);
        break;
    }
    case int32_box:
    {
        std::int32_t &value = *static_cast<std::int32_t *>(g_selected_item->data);
        const float adjusted = std::clamp(static_cast<float>(value) + direction * g_selected_item->step,
                                          g_selected_item->min_value,
                                          g_selected_item->max_value);
        value = static_cast<std::int32_t>(std::lround(adjusted));
        break;
    }
    case int8_box:
    {
        std::int8_t &value = *static_cast<std::int8_t *>(g_selected_item->data);
        const float adjusted = std::clamp(static_cast<float>(value) + direction * g_selected_item->step,
                                          g_selected_item->min_value,
                                          g_selected_item->max_value);
        value = static_cast<std::int8_t>(std::lround(adjusted));
        break;
    }
    default:
        break;
    }
}

void key_next_or_plus(void)
{
    if (g_selected_item == nullptr)
    {
        return;
    }
    if (g_selected_item->select)
    {
        adjust_selected(1.0f);
    }
    else if (g_selected_item->next_brother != nullptr)
    {
        g_selected_item = g_selected_item->next_brother;
    }
}

void key_previous_or_sub(void)
{
    if (g_selected_item == nullptr)
    {
        return;
    }
    if (g_selected_item->select)
    {
        adjust_selected(-1.0f);
    }
    else if (g_selected_item->last_brother != nullptr)
    {
        g_selected_item = g_selected_item->last_brother;
    }
}

void enter_folder(void)
{
    if (g_selected_item == nullptr || g_selected_item->select ||
        g_selected_item->kind != Menu_Folder || g_selected_item->first_son == nullptr)
    {
        return;
    }
    g_page = g_selected_item;
    g_first_item = g_page->first_son;
    g_selected_item = g_first_item;
}

void leave_folder(void)
{
    if (g_page == nullptr || g_page->father == nullptr ||
        (g_selected_item != nullptr && g_selected_item->select))
    {
        return;
    }
    Menu_Item *old_page = g_page;
    g_page = g_page->father;
    g_first_item = g_page->first_son;
    g_selected_item = old_page;
}

void select_or_toggle(void)
{
    if (g_selected_item == nullptr || g_selected_item->data == nullptr ||
        g_selected_item->kind == Menu_Folder)
    {
        return;
    }

    if (g_selected_item->kind == bool_box)
    {
        bool &value = *static_cast<bool *>(g_selected_item->data);
        value = !value;
        if (&value == &g_start_display)
        {
            if (value)
            {
                // 发车前最后一次写 framebuffer：填黑后永久停用菜单任务。
                // IPS200 驱动没有背光/反初始化接口，因此本次运行不再读按键、
                // 不再刷新屏幕；只有重启程序或整车重新上电才恢复菜单。
                g_ips200.full(RGB565_BLACK);
                g_menu_disabled_until_restart = true;
            }
            sync_start_switch();
        }
        Set_Beeptime(80);
        return;
    }

    if (Menu_Car_Enabled())
    {
        Set_Beeptime(200);
        return;
    }

    g_selected_item->select = !g_selected_item->select;
    Set_Beeptime(60);
}

void key_scan(void)
{
    switch (Board_Key_Get())
    {
    case 1:
        enter_folder();
        break;
    case 2:
        leave_folder();
        break;
    case 3:
        key_next_or_plus();
        break;
    case 4:
        key_previous_or_sub();
        break;
    case 5:
        select_or_toggle();
        break;
    default:
        break;
    }
}
}

void Menu_Init(void)
{
    g_ips200.init(FB_PATH);
    Reset_Menu_Storage();
    Creat_Menu_Folder(nullptr, &g_head, "Chun Cai Team");

    Menu_Item *smart_car = DynamicCreat_Menu_Folder(&g_head, "Smart Car");
    DynamicCreat_Menu_File(smart_car, "start car", &g_start_display, bool_box, 0.0f, 0.0f, 1.0f);
    Menu_Item *speed = DynamicCreat_Menu_Folder(smart_car, "Speed");
    Menu_Item *speed_pid = DynamicCreat_Menu_Folder(smart_car, "Speed PID");
    Menu_Item *angle_pid = DynamicCreat_Menu_Folder(smart_car, "Angle PID");

    DynamicCreat_Menu_File(speed, "straight", &k_speed_straight, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "curve", &k_speed_curve, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "sharp", &k_speed_sharp, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "obstacle", &k_speed_obstacle_avoid, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "circle", &k_speed_circle, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "lost", &k_speed_lost, float_box, 5.0f, 0.0f, 250.0f);
    DynamicCreat_Menu_File(speed, "up step", &k_speed_up_step, float_box, 1.0f, 1.0f, 50.0f);
    DynamicCreat_Menu_File(speed, "down step", &k_speed_down_step, float_box, 1.0f, 1.0f, 50.0f);

    DynamicCreat_Menu_File(speed_pid, "left kp", pid_left.kp_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(speed_pid, "left ki", pid_left.ki_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(speed_pid, "left kd", pid_left.kd_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(speed_pid, "right kp", pid_right.kp_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(speed_pid, "right ki", pid_right.ki_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(speed_pid, "right kd", pid_right.kd_ptr(), float_box, 0.1f, 0.0f, 20.0f);

    DynamicCreat_Menu_File(angle_pid, "angle kp", pid_angle.kp_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(angle_pid, "angle ki", pid_angle.ki_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(angle_pid, "angle kd", pid_angle.kd_ptr(), float_box, 0.1f, 0.0f, 20.0f);
    DynamicCreat_Menu_File(angle_pid, "angle out", pid_angle.output_limit_ptr(), float_box, 1.0f, 0.0f, 200.0f);
    DynamicCreat_Menu_File(angle_pid, "angle i lim", pid_angle.integral_limit_ptr(), float_box, 1.0f, 0.0f, 100.0f);
    DynamicCreat_Menu_File(angle_pid, "angle dead", pid_angle.deadband_ptr(), float_box, 0.1f, 0.0f, 10.0f);

    g_page = &g_head;
    g_first_item = g_head.first_son;
    g_selected_item = g_first_item;
    g_start_display = false;
    g_menu_disabled_until_restart = false;
    sync_start_switch();

    const auto now = std::chrono::steady_clock::now();
    g_last_draw_time = now - k_screen_refresh;
    g_last_press_time.fill(now - k_key_debounce);
    g_last_key_level = {{g_key1.get_level(), g_key2.get_level(), g_key3.get_level(),
                         g_key4.get_level(), g_key5_hall.get_level()}};
    show_menu();
}

std::uint8_t Board_Key_Get(void)
{
    if (key_pressed_once(g_key1, 0))
        return 1;
    if (key_pressed_once(g_key2, 1))
        return 2;
    if (key_pressed_once(g_key3, 2))
        return 3;
    if (key_pressed_once(g_key4, 3))
        return 4;
    if (key_pressed_once(g_key5_hall, 4))
        return 5;
    return 0;
}

void Menu_Task(void)
{
    if (g_menu_disabled_until_restart)
    {
        return;
    }

    key_scan();
    if (g_menu_disabled_until_restart)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - g_last_draw_time >= k_screen_refresh)
    {
        g_last_draw_time = now;
        show_menu();
    }
}

void Menu_Force_Stop(void)
{
    g_start_display = false;
    sync_start_switch();
}

bool Menu_Car_Enabled(void)
{
    return g_car_enabled.load();
}
