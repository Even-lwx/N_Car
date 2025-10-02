/*********************************************************************
  File: menu_clean.c
  Clean menu system implementation for IPS114 display
  Adapted from IPS200 menu system with all business logic removed
*********************************************************************/

#include "zf_common_typedef.h"
#include "zf_device_ips114.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "menu.h"

/**************** Function Declarations ****************/
void ips_clear(void);
void show_string(uint16 x, uint16 y, const char *str);
void show_int(uint16 x, uint16 y, int value, uint8 num);
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum);

/**************** Global Variables ****************/

Page *Now_Menu = NULL;  // Current menu page pointer
static uint8 need_refresh = 1;  // Screen refresh flag

/**************** Key Scan Functions ****************/

/**
 * @brief Initialize keys (GPIO configuration)
 */
void Key_Init(void)
{
    // Initialize all 4 keys as input with pull-up
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/**
 * @brief Scan keys with debounce
 * @return Key code (KEY_NONE, KEY_UP, KEY_DOWN, KEY_OK, KEY_BACK)
 */
uint8 Key_Scan(void)
{
    static uint8 key_state = 0;  // 0: idle, 1: pressed, 2: wait release
    uint8 key_value = KEY_NONE;
    
    if(key_state == 0)  // Idle state, check for key press
    {
        // KEY1 - UP
        if(gpio_get_level(KEY1_PIN) == GPIO_LOW)
        {
            system_delay_ms(10);  // Debounce delay
            if(gpio_get_level(KEY1_PIN) == GPIO_LOW)
            {
                key_value = KEY_UP;
                key_state = 1;
            }
        }
        // KEY2 - DOWN
        else if(gpio_get_level(KEY2_PIN) == GPIO_LOW)
        {
            system_delay_ms(10);
            if(gpio_get_level(KEY2_PIN) == GPIO_LOW)
            {
                key_value = KEY_DOWN;
                key_state = 1;
            }
        }
        // KEY3 - OK (Confirm/Enter)
        else if(gpio_get_level(KEY3_PIN) == GPIO_LOW)
        {
            system_delay_ms(10);
            if(gpio_get_level(KEY3_PIN) == GPIO_LOW)
            {
                key_value = KEY_OK;
                key_state = 1;
            }
        }
        // KEY4 - BACK (Return)
        else if(gpio_get_level(KEY4_PIN) == GPIO_LOW)
        {
            system_delay_ms(10);
            if(gpio_get_level(KEY4_PIN) == GPIO_LOW)
            {
                key_value = KEY_BACK;
                key_state = 1;
            }
        }
    }
    else if(key_state == 1)  // Key pressed, wait for release
    {
        if(gpio_get_level(KEY1_PIN) == GPIO_HIGH && 
           gpio_get_level(KEY2_PIN) == GPIO_HIGH &&
           gpio_get_level(KEY3_PIN) == GPIO_HIGH &&
           gpio_get_level(KEY4_PIN) == GPIO_HIGH)
        {
            key_state = 0;  // Return to idle state
        }
    }
    
    return key_value;
}

/**************** Example Parameters (for testing) ****************/

// Example parameter storage
float example_float1 = 1.5f;
float example_float2 = 2.0f;
int16 example_int16_1 = 100;
int16 example_int16_2 = 200;
uint32 example_uint32 = 1000;

// Step arrays for parameter adjustment - each parameter can have its own step array
float float1_step[] = {0.01f, 0.1f, 1.0f};           // 3 steps for float1
float float2_step[] = {0.1f, 0.5f};                  // 2 steps for float2 (different from float1)
int16 int16_1_step[] = {1, 10, 100};                 // 3 steps for int16_1
int16 int16_2_step[] = {5, 50};                      // 2 steps for int16_2 (different from int16_1)
uint32 uint32_step[] = {1, 10, 100, 1000};           // 4 steps for uint32

// Parameter definitions for Example Menu
CustomData example_data[] = {
    // address, type, name, step_array, step_len, step_num, digit_int, digit_point
    {&example_float1, data_float_show, "Float Param 1", float1_step, 3, 0, 1, 2},
    {&example_float2, data_float_show, "Float Param 2", float2_step, 2, 0, 1, 2},
    {&example_int16_1, data_int16_show, "Int16 Param 1", int16_1_step, 3, 0, 3, 0},
    {&example_int16_2, data_int16_show, "Int16 Param 2", int16_2_step, 2, 0, 3, 0},
    {&example_uint32, data_uint32_show, "Uint32 Param", uint32_step, 4, 0, 5, 0}
};

// ===== Add your new menu parameters here =====
// Example: Motor Control Menu
float motor_speed = 0.0f;
int16 motor_duty = 0;

float motor_speed_step[] = {0.1f, 1.0f, 10.0f};
int16 motor_duty_step[] = {10, 100, 1000};

CustomData motor_data[] = {
    {&motor_speed, data_float_show, "Motor Speed", motor_speed_step, 3, 0, 3, 1},
    {&motor_duty, data_int16_show, "Motor Duty", motor_duty_step, 3, 0, 5, 0}
};

// Page definitions
Page page_example = {
    "Example Menu",     // name
    example_data,       // data
    5,                  // len (number of parameters)
    Menu,               // stage
    NULL,               // back (will be set in Menu_Init)
    {NULL, NULL, NULL, NULL},  // enter
    {NULL},             // content
    0                   // order
};

// ===== Add your new menu page here =====
Page page_motor = {
    "Motor Control",    // name
    motor_data,         // data
    2,                  // len (number of parameters: motor_speed, motor_duty)
    Menu,               // stage
    NULL,               // back (will be set in Menu_Init)
    {NULL, NULL, NULL, NULL},  // enter
    {NULL},             // content
    0                   // order
};

Page main_page = {
    "Main Menu",        // name
    NULL,               // data
    2,                  // len (number of sub-menus) - CHANGED from 1 to 2
    Menu,               // stage
    NULL,               // back
    {&page_example, &page_motor, NULL, NULL},  // enter - ADDED &page_motor
    {NULL},             // content
    0                   // order
};

/**************** Menu Control Functions ****************/

/**
 * @brief Initialize menu system
 */
void Menu_Init(void)
{
    // Initialize keys first
    Key_Init();
    
    // Set up menu relationships (back pointers)
    page_example.back = &main_page;
    page_motor.back = &main_page;  // ADD this line for new menu
    
    // Set current menu to main page
    Now_Menu = &main_page;
}

/**
 * @brief Enter key operation
 */
void Menu_Enter(void)
{
    if(Now_Menu->stage == Menu)
    {
        // Navigate to sub-menu if available
        if(Now_Menu->enter[Now_Menu->order] != NULL)
        {
            ips_clear();  // Clear screen immediately
            Now_Menu = Now_Menu->enter[Now_Menu->order];
            Now_Menu->order = 0;
            need_refresh = 1;  // Request screen refresh when entering submenu
        }
    }
    else if(Now_Menu->stage == Change)
    {
        // Switch to next step size
        Now_Menu->data[Now_Menu->order].step_num++;
        if(Now_Menu->data[Now_Menu->order].step_num >= Now_Menu->data[Now_Menu->order].step_len)
        {
            Now_Menu->data[Now_Menu->order].step_num = 0;
        }
    }
}

/**
 * @brief Back key operation
 */
void Menu_Back(void)
{
    if(Now_Menu->stage == Menu)
    {
        // Return to parent menu
        if(Now_Menu->back != NULL)
        {
            ips_clear();  // Clear screen immediately
            Now_Menu = Now_Menu->back;
            need_refresh = 1;  // Request screen refresh when returning to parent menu
        }
    }
    else if(Now_Menu->stage == Change)
    {
        // Exit parameter adjustment mode - return to Menu stage
        ips_clear();  // Clear screen immediately
        Now_Menu->stage = Menu;
        need_refresh = 1;  // Request screen refresh when exiting Change mode
    }
    else if(Now_Menu->stage == Funtion)
    {
        // Exit function page - return to Menu stage
        ips_clear();  // Clear screen immediately
        Now_Menu->stage = Menu;
        need_refresh = 1;  // Request screen refresh when exiting Function mode
    }
}

/**
 * @brief Up key operation
 */
void Menu_Up(void)
{
    if(Now_Menu->stage == Menu)
    {
        // Move selection up
        if(Now_Menu->order > 0)
        {
            Now_Menu->order--;
        }
        else
        {
            // Loop to end
            if(Now_Menu->len > 0)
            {
                Now_Menu->order = Now_Menu->len - 1;
            }
        }
    }
    else if(Now_Menu->stage == Change)
    {
        // Increase parameter value
        switch(Now_Menu->data[Now_Menu->order].type)
        {
            case data_float_show:
                *(float*)Now_Menu->data[Now_Menu->order].address += 
                    ((float*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_int16_show:
                *(int16*)Now_Menu->data[Now_Menu->order].address += 
                    ((int16*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_int_show:
                *(int*)Now_Menu->data[Now_Menu->order].address += 
                    ((int*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_uint32_show:
                *(uint32*)Now_Menu->data[Now_Menu->order].address += 
                    ((uint32*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
        }
    }
}

/**
 * @brief Down key operation
 */
void Menu_Down(void)
{
    if(Now_Menu->stage == Menu)
    {
        // Move selection down
        if(Now_Menu->len > 0 && Now_Menu->order < Now_Menu->len - 1)
        {
            Now_Menu->order++;
        }
        else
        {
            // Loop to start
            Now_Menu->order = 0;
        }
    }
    else if(Now_Menu->stage == Change)
    {
        // Decrease parameter value
        switch(Now_Menu->data[Now_Menu->order].type)
        {
            case data_float_show:
                *(float*)Now_Menu->data[Now_Menu->order].address -= 
                    ((float*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_int16_show:
                *(int16*)Now_Menu->data[Now_Menu->order].address -= 
                    ((int16*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_int_show:
                *(int*)Now_Menu->data[Now_Menu->order].address -= 
                    ((int*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
            
            case data_uint32_show:
                *(uint32*)Now_Menu->data[Now_Menu->order].address -= 
                    ((uint32*)Now_Menu->data[Now_Menu->order].step)[Now_Menu->data[Now_Menu->order].step_num];
                break;
        }
    }
}

/**
 * @brief Left key operation (enter parameter adjustment mode)
 */
void Menu_Left(void)
{
    if(Now_Menu->stage == Menu && Now_Menu->len > 0)
    {
        // Enter parameter adjustment mode
        ips_clear();  // Clear screen immediately
        Now_Menu->stage = Change;
        need_refresh = 1;  // Request screen refresh when entering Change mode
    }
}

/**
 * @brief Right key operation (execute function)
 */
void Menu_Right(void)
{
    if(Now_Menu->stage == Menu)
    {
        // Execute function if available
        if(Now_Menu->content.function != NULL)
        {
            Now_Menu->stage = Funtion;
            Now_Menu->content.function();
        }
    }
}

/**************** Display Functions ****************/

/**
 * @brief Display string (wrapper)
 */
void show_string(uint16 x, uint16 y, const char *str)
{
    ips114_show_string(x * 8, y * 8, str);
}

/**
 * @brief Display integer
 */
void show_int(uint16 x, uint16 y, int32 value, uint8 num)
{
    ips114_show_int(x * 8, y * 8, value, num);
}

/**
 * @brief Display float
 */
void show_float(uint16 x, uint16 y, float value, uint8 num, uint8 pointnum)
{
    ips114_show_float(x * 8, y * 8, value, num, pointnum);
}

/**
 * @brief Clear screen
 */
void ips_clear(void)
{
    ips114_clear();
}

/**
 * @brief Clear single line
 */
void ips_clear_line(uint16 x, uint16 y, uint8 len)
{
    for(uint8 i = 0; i < len; i++)
    {
        show_string(x + i, y, " ");
    }
}

/**
 * @brief Display menu (main display function)
 */
void Menu_Show(void)
{
    static uint8 last_stage = 0xFF;  // Track stage changes
    uint8 full_redraw = 0;           // Full redraw flag
    
    // Check if screen refresh is needed
    if(need_refresh)
    {
        ips_clear();
        need_refresh = 0;
        full_redraw = 1;
        last_stage = Now_Menu->stage;
    }
    
    // Display menu title (only in Menu mode)
    if(Now_Menu->stage == Menu)
    {
        show_string(1, 0, Now_Menu->name);
    }
    
    if(Now_Menu->stage == Menu)
    {
        // Display menu items
        if(Now_Menu->data != NULL)
        {
            // Parameter menu
            for(uint8 i = 0; i < Now_Menu->len && i < 6; i++)
            {
                // Clear or display selection indicator
                if(i == Now_Menu->order)
                {
                    show_string(0, i * 2 + 2, ">");
                }
                else
                {
                    show_string(0, i * 2 + 2, " ");  // Clear cursor at other positions
                }
                
                // Display parameter name
                show_string(1, i * 2 + 2, Now_Menu->data[i].name);
                
                // Display parameter value
                switch(Now_Menu->data[i].type)
                {
                    case data_float_show:
                        show_float(15, i * 2 + 2, *(float*)Now_Menu->data[i].address, 
                                  Now_Menu->data[i].digit_int, Now_Menu->data[i].digit_point);
                        break;
                    
                    case data_int16_show:
                        show_int(15, i * 2 + 2, *(int16*)Now_Menu->data[i].address, 
                                Now_Menu->data[i].digit_int);
                        break;
                    
                    case data_int_show:
                        show_int(15, i * 2 + 2, *(int*)Now_Menu->data[i].address, 
                                Now_Menu->data[i].digit_int);
                        break;
                    
                    case data_uint32_show:
                        show_int(15, i * 2 + 2, *(uint32*)Now_Menu->data[i].address, 
                                Now_Menu->data[i].digit_int);
                        break;
                }
            }
        }
        else
        {
            // Sub-menu list
            for(uint8 i = 0; i < Now_Menu->len && i < 6; i++)
            {
                // Clear or display selection indicator
                if(i == Now_Menu->order)
                {
                    show_string(0, i * 2 + 2, ">");
                }
                else
                {
                    show_string(0, i * 2 + 2, " ");  // Clear cursor at other positions
                }
                
                if(Now_Menu->enter[i] != NULL)
                {
                    show_string(1, i * 2 + 2, Now_Menu->enter[i]->name);
                }
            }
        }
    }
    else if(Now_Menu->stage == Change)
    {
        // Parameter adjustment mode - always redraw everything
        if(Now_Menu->data != NULL && Now_Menu->len > 0)
        {
            CustomData *param = &Now_Menu->data[Now_Menu->order];
            
            // Display title (Y=0, same as menu title)
            show_string(1, 0, "ADJUST MODE");
            
            // Display parameter name (Y=2, with spacing)
            show_string(1, 2, param->name);
            
            // Display current value (Y=6, spacing: 2+4=6, similar to i*2+2 pattern)
            show_string(1, 6, "Value:");
            switch(param->type)
            {
                case data_float_show:
                    show_float(8, 6, *(float*)param->address, 
                              param->digit_int, param->digit_point);
                    break;
                
                case data_int16_show:
                    show_int(8, 6, *(int16*)param->address, param->digit_int);
                    break;
                
                case data_int_show:
                    show_int(8, 6, *(int*)param->address, param->digit_int);
                    break;
                
                case data_uint32_show:
                    show_int(8, 6, *(uint32*)param->address, param->digit_int);
                    break;
            }
            
            // Display current step size (Y=10, spacing: 6+4=10)
            show_string(1, 8, "Step: ");
            switch(param->type)
            {
                case data_float_show:
                    show_float(7, 8, ((float*)param->step)[param->step_num], 1, 3);
                    break;
                
                case data_int16_show:
                    show_int(7, 8, ((int16*)param->step)[param->step_num], 5);
                    break;
                
                case data_int_show:
                    show_int(7, 8, ((int*)param->step)[param->step_num], 5);
                    break;
                
                case data_uint32_show:
                    show_int(7, 8, ((uint32*)param->step)[param->step_num], 5);
                    break;
            }
            
            // Display operation hints (Y=14, spacing: 10+4=14)
            show_string(1, 10, "--------------------");
            show_string(1, 12, "UP/DN:Value");
            show_string(1, 14, "OK:Step  BACK:Exit");
        }
    }
}

/**
 * @brief Key operation handler
 * Maps physical key codes to menu operations
 */
void Key_operation(uint8 key)
{
    switch(key)
    {
        case KEY_UP:    // KEY1 - Up
            Menu_Up();
            break;
        
        case KEY_DOWN:  // KEY2 - Down
            Menu_Down();
            break;
        
        case KEY_OK:    // KEY3 - OK/Enter/Confirm
            if(Now_Menu->stage == Menu)
            {
                // If current menu has data (parameter list), enter adjustment mode
                if(Now_Menu->data != NULL && Now_Menu->len > 0)
                {
                    Menu_Left();  // Enter parameter adjustment mode
                }
                // Otherwise enter submenu
                else
                {
                    Menu_Enter();  // Enter submenu
                }
            }
            else if(Now_Menu->stage == Change)
            {
                Menu_Enter();  // Switch step size
            }
            break;
        
        case KEY_BACK:  // KEY4 - Back/Return
            Menu_Back();
            break;
        
        default:
            break;
    }
}
