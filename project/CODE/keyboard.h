#ifndef __IPS114_H
#define __IPS114_H
#include "headfile.h"
#include "data.h"
#include "ringjudge.h"

#define ButtonCnt 5 //定义按键数量，按键引脚定义在.c内
#define KeystrokeONE 1
#define KeystrokeTWO 2
#define KeystrokeTHREE 3
#define KeystrokeFOUR 4
#define KeystrokeFIVE 5

#define KeyONE P70
#define KeyTWO P71
#define KeyTHREE P72
#define KeyFOUR P73
#define KeyFIVE P54

#define Switch1 P50
#define Switch2 P51

#define EEPROM_Garage_Out_Dir (0)
#define EEPROM_Circle_Switch (1)
#define EEPROM_Speed (2)
#define EEPROM_Direction (3)
#define EEPROM_DistanceG (4)
#define EEPROM_DistanceC (5)
#define EEPROM_THRESHOLD (6)

//按键标签枚举
typedef enum
{
    ButtonOne,
    ButtonTwo,
    ButtonThree,
    ButtonFour,
    ButtonFive
} ButtonLabel_enum;

/**************************************变量定义**************************************/
extern const PIN_enum ButtonPins[ButtonCnt];           //按键引脚
extern const ButtonLabel_enum ButtonLabels[ButtonCnt]; //引脚对应的标签
extern uint16 KeystrokeLabel;
extern uint16 mode;
extern int16 Dispay_Codename;
extern float step;
extern uint16 display;
extern uint16 step_back;

void Keystroke_Scan(void);
void Keystroke_Menu(void);
void Keystroke_Menu_STEP(void);          //-1
void Keystroke_Menu_HOME(void);          // 0
void Keystroke_Menu_ONE(void);           // 1
void Keystroke_Menu_ONE_ONE(void);       // 1.1
void Keystroke_Menu_TWO(void);           // 2
void Keystroke_Menu_TWO_ONE(void);       // 2.1
void Keystroke_Menu_TWO_TWO(void);       // 2.2
void Keystroke_Menu_THREE(void);         // 3
void Keystroke_Menu_FOUR(void);          //4
//void Keystroke_Menu_FIVE(void);          //5
//void Keystroke_Menu_RUNNING(void);       //666

#endif