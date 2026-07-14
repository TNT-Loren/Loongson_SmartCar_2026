#ifndef __COMMON_MYMENU_HPP__
#define __COMMON_MYMENU_HPP__

#include <cstdint>

void Menu_Init(void);
void Menu_Task(void);
void Menu_Force_Stop(void);
bool Menu_Car_Enabled(void);
std::uint8_t Board_Key_Get(void);

#endif
