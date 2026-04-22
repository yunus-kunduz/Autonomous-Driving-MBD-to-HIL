#include "perception.h"

void Perception_Init(Perception_State_t *state)
{
    state->obstacle_center = 0;
    state->obstacle_left = 0;
    state->obstacle_right = 0;
    state->is_red_light = 0;
    state->current_lane_error = 0.0f;
}

void Perception_Process(Perception_State_t *state, Autonomous_Data_t *raw_packet)
{
    // 1. Şerit hatasını doğrudan aktar
    state->current_lane_error = raw_packet->lane_error;

    // 2. Trafik ışığı durumunu aktar
    state->is_red_light = raw_packet->light_status;

    // 3. Bitmasking ile engelleri ayıkla
    // 0x01 = 0000 0001 (Bit 0 - Center)
    // 0x02 = 0000 0010 (Bit 1 - Left)
    // 0x04 = 0000 0100 (Bit 2 - Right)

    state->obstacle_center = (raw_packet->obs_flags & 0x01) ? 1 : 0;
    state->obstacle_left   = (raw_packet->obs_flags & 0x02) ? 1 : 0;
    state->obstacle_right  = (raw_packet->obs_flags & 0x04) ? 1 : 0;
}
