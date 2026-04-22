#ifndef INC_PERCEPTION_H_
#define INC_PERCEPTION_H_

#include "main.h"
#include "hil_proto.h" // Autonomous_Data_t yapısını kullanmak için

// Algılama Sonuçları Yapısı
typedef struct
{
    uint8_t obstacle_center;
    uint8_t obstacle_left;
    uint8_t obstacle_right;
    uint8_t is_red_light;
    float current_lane_error;
} Perception_State_t;

// Fonksiyon Prototipleri
void Perception_Init(Perception_State_t *state);
void Perception_Process(Perception_State_t *state, Autonomous_Data_t *raw_packet);

#endif /* INC_PERCEPTION_H_ */
