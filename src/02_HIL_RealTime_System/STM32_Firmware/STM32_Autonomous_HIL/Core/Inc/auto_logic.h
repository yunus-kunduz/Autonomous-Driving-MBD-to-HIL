#ifndef INC_AUTO_LOGIC_H_
#define INC_AUTO_LOGIC_H_

#include "perception.h"
#include "pid_ctrl.h"
#include "hil_proto.h"

// Araç Durumları (States)
typedef enum
{
    STATE_FOLLOW = 0,    // Normal Şerit Takibi
    STATE_AVOID_LEFT,    // Sola Kaçınma Manevrası
    STATE_AVOID_RIGHT,   // Sağa Kaçınma Manevrası
    STATE_STOP,          // Kırmızı Işık veya Tam Engel
    STATE_RECOVERY       // Sıkışma Durumu (Geri Vites)
} System_State_t;

// Karar Mekanizması Yapısı
typedef struct
{
    System_State_t current_state;
    Control_Command_t last_command; // Python'a gönderilecek son komutlar
    uint32_t state_timer;           // Belirli durumlarda kalma süresi
} Autonomous_Logic_t;

// Fonksiyon Prototipleri
void Logic_Init(Autonomous_Logic_t *logic);
void Logic_Update(Autonomous_Logic_t *logic, Perception_State_t *perc, PD_Controller_t *pid);

#endif /* INC_AUTO_LOGIC_H_ */
