#ifndef __COMMON_MENU_HPP__
#define __COMMON_MENU_HPP__

#include <cstdint>

enum Menu_Kind
{
    Menu_Folder = 0,
    float_box,
    int32_box,
    uint32_box,
    int8_box,
    uint8_box,
    int16_box,
    uint16_box,
    bool_box,
};

struct Menu_Item
{
    const char *name = nullptr;
    void *data = nullptr;
    Menu_Kind kind = Menu_Folder;
    std::uint8_t select = 0;

    float step = 1.0f;
    float min_value = 0.0f;
    float max_value = 0.0f;

    std::uint8_t sons = 0;
    std::uint8_t NO = 0;
    Menu_Item *father = nullptr;
    Menu_Item *first_son = nullptr;
    Menu_Item *last_brother = nullptr;
    Menu_Item *next_brother = nullptr;
};

void Reset_Menu_Storage(void);
void Creat_Menu_Folder(Menu_Item *father, Menu_Item *me, const char name[]);
void Creat_Menu_File(Menu_Item *father,
                     Menu_Item *me,
                     const char name[],
                     void *data,
                     Menu_Kind kind,
                     float step,
                     float min_value,
                     float max_value);
Menu_Item *DynamicCreat_Menu_Folder(Menu_Item *father, const char name[]);
Menu_Item *DynamicCreat_Menu_File(Menu_Item *father,
                                 const char name[],
                                 void *data,
                                 Menu_Kind kind,
                                 float step,
                                 float min_value,
                                 float max_value);

#endif
