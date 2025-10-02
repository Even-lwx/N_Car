/************************************************************************
  Copyright (C), 2024 , ASTA ASC.
  File name:        menu.h
  Author: CANG_HAI  Version:  2.00 (Cleaned)   Date: 2025/10/02
  Description:      Clean menu framework header file
                    Adapted for IPS114 display (240x135 pixels)
                    All business logic removed
**************************************************************************/
#ifndef _MENU_H_
#define _MENU_H_

#include "zf_common_typedef.h"
#include "zf_device_ips114.h"
#include "zf_driver_gpio.h"

/**************** GPIO Pin Definitions ****************/
#define KEY1_PIN    P20_6   // KEY1 - UP
#define KEY2_PIN    P20_7   // KEY2 - DOWN
#define KEY3_PIN    P11_2   // KEY3 - OK (Enter/Confirm)
#define KEY4_PIN    P11_3   // KEY4 - BACK (Return)

/**************** Key Code Definitions ****************/
#define KEY_NONE    0
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_OK      3
#define KEY_BACK    4

/**************** Long Press Configuration ****************/
#define LONG_PRESS_CNT   15     // 长按阈值：连续检测15次认为长按（约300ms）
#define REPEAT_INTERVAL  3      // 长按后每3次循环触发一次（约60ms间隔）

/**************** Font Configuration ****************/
#define FONT_W      8   // Character width
#define FONT_H      8   // Character height

/**************** Enumerations ****************/

/**
 * @brief Page type enumeration
 */
typedef enum {
    Menu,           // Menu page
    Change,         // Parameter adjustment page
    Funtion,        // Function execution page
} Page_Type;

/**
 * @brief Data type enumeration
 */
typedef enum {
    data_float_show = 0,    // Float
    data_int16_show = 1,    // 16-bit integer
    data_uint32_show = 2,   // 32-bit unsigned integer
    data_int_show = 3,      // Integer
} Data_Type;

/**************** Structure Definitions ****************/

/**
 * @brief Custom data structure
 */
typedef struct {
    void *address;          // Data address
    Data_Type type;         // Data type
    char name[20];          // Parameter name
    void *step;             // Step array pointer
    uint8 step_len;         // Step array length
    uint8 step_num;         // Current step index
    uint8 digit_int;        // Integer digits
    uint8 digit_point;      // Decimal digits
} CustomData;

/**
 * @brief Page structure (forward declaration)
 */
typedef struct Page Page;

/**
 * @brief Page structure
 */
struct Page {
    char name[20];          // Page name
    CustomData *data;       // Data array pointer
    uint8 len;              // Data array length
    Page_Type stage;        // Page type
    Page *back;             // Parent page
    Page *enter[4];         // Child pages (max 4)
    union {
        void (*function)(void);     // Function pointer
    } content;
    uint8 order;            // Current selection index
};

/**************** Global Variable Declarations ****************/
extern Page *Now_Menu;          // Current menu pointer

extern int16 add_mode[5];       // Step arrays
extern float add_float[5];
extern int16 add_int16[5];
extern uint32 add_uint32[5];
extern int add_int[5];

/**************** Function Declarations ****************/

// Key scan functions
void Key_Init(void);            // Key initialization
uint8 Key_Scan(void);           // Key scan with debounce

// Menu operation functions
void Menu_Init(void);           // Menu initialization
void Menu_Enter(void);          // Enter key operation
void Menu_Back(void);           // Back key operation
void Menu_Up(void);             // Up key operation
void Menu_Down(void);           // Down key operation
void Menu_Left(void);           // Left key operation (enter adjustment)
void Menu_Right(void);          // Right key operation (execute function)
void Menu_Show(void);           // Menu display
void Key_operation(uint8 key);  // Key handler

// Display functions
void show_string(uint16 x, uint16 y, const char *str);
void show_int(uint16 x, uint16 y, int32 value, uint8 num);
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);
void show_uint(uint16 x, uint16 y, uint32 value, uint8 num);

// Utility functions
void Screen_Clear(void);
void Screen_SetColor(uint16 pen_color, uint16 bg_color);

/**************** Backward Compatibility Macros ****************/
#define menu_init()     Menu_Init()
#define menu_update()   Menu_Show()

#endif // _MENU_H_
