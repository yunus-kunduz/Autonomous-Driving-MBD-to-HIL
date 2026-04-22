#ifndef INC_PID_CTRL_H_
#define INC_PID_CTRL_H_

#include "main.h"

// PD Kontrolcü Parametreleri
typedef struct
{
    float Kp;           // Oransal Katsayı (Anlık tepki)
    float Kd;           // Türevsel Katsayı (Gelecek tahmini/Sönümleme)
    float last_error;   // Bir önceki hata (Türev hesabı için)
    float out_max;      // Maksimum direksiyon açısı (Doyum)
    float out_min;      // Minimum direksiyon açısı
} PD_Controller_t;

// Fonksiyon Prototipleri
void PD_Init(PD_Controller_t *pid, float kp, float kd, float limit);
float PD_Compute(PD_Controller_t *pid, float error);

#endif /* INC_PID_CTRL_H_ */
