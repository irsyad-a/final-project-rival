#include "stm32_adapter.h"
#include <string.h>

#ifndef UART_RX_MAX
#define UART_RX_MAX 128
#endif

static UART_HandleTypeDef *s_uart = NULL;
static uint8_t s_rxch = 0;
static char s_line[UART_RX_MAX];
static uint16_t s_len = 0;

// weak hooks (can be overridden by user code)
__attribute__((weak)) void Motor_Stop(void) {}
__attribute__((weak)) void Move_OpenLoop_cm(int dist_cm, int speed_0_255) {(void)dist_cm; (void)speed_0_255;}
__attribute__((weak)) void Turn_OpenLoop_deg(int angle_deg, int speed_0_255, int left) {(void)angle_deg; (void)speed_0_255; (void)left;}

void STM32A_StartIT(void)
{
    if (s_uart) HAL_UART_Receive_IT(s_uart, &s_rxch, 1);
}

void STM32A_Init(UART_HandleTypeDef *uart)
{
    s_uart = uart;
    s_len = 0;
    STM32A_StartIT();
}

static void handle_complete_line(void)
{
    s_line[s_len] = 0;
    proto_cmd_t cmd;
    if (parse_line(s_line, &cmd))
    {
        switch (cmd.type)
        {
        case CMD_STOP:
            Motor_Stop();
            break;
        case CMD_HB:
            // Optional: send OK
            // const char ok[] = "OK\n";
            // HAL_UART_Transmit(s_uart, (uint8_t*)ok, sizeof(ok)-1, 20);
            break;
        case CMD_MOVE:
            if (cmd.forward) Move_OpenLoop_cm(cmd.dist_cm, cmd.speed);
            else             Move_OpenLoop_cm(-cmd.dist_cm, cmd.speed);
            break;
        case CMD_TURN:
            Turn_OpenLoop_deg(cmd.angle_deg, cmd.speed, cmd.left);
            break;
        default:
            break;
        }
    }
    s_len = 0;
}

void STM32A_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_uart) return;

    if (s_len < UART_RX_MAX - 1)
    {
        s_line[s_len++] = (char)s_rxch;
        if (s_rxch == '\n')
        {
            handle_complete_line();
        }
    }
    else
    {
        s_len = 0; // overflow -> reset buffer
    }

    STM32A_StartIT();
}


