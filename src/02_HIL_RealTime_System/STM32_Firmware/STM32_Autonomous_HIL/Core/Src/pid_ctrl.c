#include "pid_ctrl.h"

// Kontrolcüyü sıfırla ve katsayıları yükle
void PD_Init(PD_Controller_t *pid, float kp, float kd, float limit)
{
    pid->Kp = kp;
    pid->Kd = kd;
    pid->last_error = 0.0f;
    pid->out_max = limit;
    pid->out_min = -limit;
}

// PD Hesaplama Fonksiyonu
float PD_Compute(PD_Controller_t *pid, float error)
{
    // 1. P Terimi: Mevcut hata ile orantılı tepki
    float p_out = pid->Kp * error;

    // 2. D Terimi: Hata değişim hızı (Frenleme/Sönümleme etkisi)
    float d_out = pid->Kd * (error - pid->last_error);

    // 3. Toplam Çıkış
    float output = p_out + d_out;

    // 4. Bir sonraki adım için hatayı sakla
    pid->last_error = error;

    // 5. Anti-Saturation (Doyum Engelleme)
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    return output;
}
