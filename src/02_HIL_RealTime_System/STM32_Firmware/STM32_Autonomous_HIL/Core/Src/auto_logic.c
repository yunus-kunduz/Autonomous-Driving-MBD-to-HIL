#include "auto_logic.h"

void Logic_Init(Autonomous_Logic_t *logic)
{
    logic->current_state = STATE_FOLLOW;
    logic->state_timer = 0;
    logic->last_command.header = 0xBB; // STM32 Komut Başlığı
}

void Logic_Update(Autonomous_Logic_t *logic, Perception_State_t *perc, PD_Controller_t *pid)
{

    // --- DURUM GEÇİŞ MANTIĞI (STATE TRANSITIONS) ---
    switch (logic->current_state) {

        case STATE_FOLLOW:
            if (perc->is_red_light || (perc->obstacle_center && perc->obstacle_left && perc->obstacle_right))
            {
                logic->current_state = STATE_STOP;
            }
            else if (perc->obstacle_center && !perc->obstacle_left)
            {
                logic->current_state = STATE_AVOID_LEFT;
            }
            else if (perc->obstacle_center && !perc->obstacle_right)
            {
                logic->current_state = STATE_AVOID_RIGHT;
            }
            break;

        case STATE_AVOID_LEFT:
            if (!perc->obstacle_center && !perc->obstacle_left) logic->current_state = STATE_FOLLOW;
            break;

        case STATE_AVOID_RIGHT:
            if (!perc->obstacle_center && !perc->obstacle_right) logic->current_state = STATE_FOLLOW;
            break;

        case STATE_STOP:
            if (!perc->is_red_light && !perc->obstacle_center) logic->current_state = STATE_FOLLOW;
            break;

        default:
            logic->current_state = STATE_FOLLOW;
    }

    // --- DURUM ÇIKTILARI (STATE OUTPUTS) ---
    switch (logic->current_state)
    {

        case STATE_FOLLOW:
            logic->last_command.steer_cmd = PD_Compute(pid, perc->current_lane_error);
            logic->last_command.throttle_cmd = 0.4f; // Normal hız
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Yeşil LED
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Kırmızı LED OFF
            break;

        case STATE_AVOID_LEFT:
            logic->last_command.steer_cmd = -0.6f; // Sola sert kır
            logic->last_command.throttle_cmd = 0.25f; // Manevrada yavaşla
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // Turuncu LED
            break;

        case STATE_STOP:
            logic->last_command.steer_cmd = 0.0f;
            logic->last_command.throttle_cmd = -1.0f; // Fren/Durma
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   // Kırmızı LED
            break;

        default:
            break;
    }

    // Checksum hesapla (Haberleşme güvenliği için)
    logic->last_command.checksum = logic->last_command.header ^
                                   (uint8_t)logic->last_command.steer_cmd ^
                                   (uint8_t)logic->last_command.throttle_cmd;
}
