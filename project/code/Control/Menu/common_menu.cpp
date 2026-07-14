#include "common_menu.hpp"

#include <array>
#include <cstddef>

namespace
{
constexpr std::size_t k_max_menu_items = 64;
std::array<Menu_Item, k_max_menu_items> g_dynamic_items{};
std::size_t g_menu_index = 0;

void Creat_Menu_Item(Menu_Item *father,
                     Menu_Item *me,
                     const char name[],
                     void *data,
                     Menu_Kind kind,
                     float step,
                     float min_value,
                     float max_value)
{
    if (me == nullptr)
    {
        return;
    }

    *me = Menu_Item{};
    me->name = name;
    me->data = data;
    me->kind = kind;
    me->step = step;
    me->min_value = min_value;
    me->max_value = max_value;
    me->father = father;

    if (father == nullptr || father->kind != Menu_Folder)
    {
        return;
    }

    if (father->first_son == nullptr)
    {
        father->first_son = me;
    }
    else
    {
        Menu_Item *last = father->first_son;
        while (last->next_brother != nullptr)
        {
            last = last->next_brother;
        }
        last->next_brother = me;
        me->last_brother = last;
    }

    me->NO = father->sons;
    father->sons++;
}
}

void Reset_Menu_Storage(void)
{
    g_dynamic_items.fill(Menu_Item{});
    g_menu_index = 0;
}

void Creat_Menu_Folder(Menu_Item *father, Menu_Item *me, const char name[])
{
    Creat_Menu_Item(father, me, name, nullptr, Menu_Folder, 0.0f, 0.0f, 0.0f);
}

void Creat_Menu_File(Menu_Item *father,
                     Menu_Item *me,
                     const char name[],
                     void *data,
                     Menu_Kind kind,
                     float step,
                     float min_value,
                     float max_value)
{
    Creat_Menu_Item(father, me, name, data, kind, step, min_value, max_value);
}

Menu_Item *DynamicCreat_Menu_Folder(Menu_Item *father, const char name[])
{
    if (father == nullptr || father->kind != Menu_Folder || g_menu_index >= k_max_menu_items)
    {
        return nullptr;
    }

    Menu_Item *me = &g_dynamic_items[g_menu_index++];
    Creat_Menu_Folder(father, me, name);
    return me;
}

Menu_Item *DynamicCreat_Menu_File(Menu_Item *father,
                                 const char name[],
                                 void *data,
                                 Menu_Kind kind,
                                 float step,
                                 float min_value,
                                 float max_value)
{
    if (father == nullptr || father->kind != Menu_Folder || data == nullptr ||
        g_menu_index >= k_max_menu_items)
    {
        return nullptr;
    }

    Menu_Item *me = &g_dynamic_items[g_menu_index++];
    Creat_Menu_File(father, me, name, data, kind, step, min_value, max_value);
    return me;
}
