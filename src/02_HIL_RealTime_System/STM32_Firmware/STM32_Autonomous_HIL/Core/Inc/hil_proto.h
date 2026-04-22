#ifndef INC_HIL_PROTO_H_
#define INC_HIL_PROTO_H_

#include "main.h"

// Python'dan gelecek veri paketi yapısı
typedef struct
{
    uint8_t header;         // 0xAA sabiti
    float lane_error;       // Şerit hatası (PD için)
    uint8_t light_status;   // Trafik ışığı
    uint8_t obs_flags;      // Engel bitleri (C-L-R)
    uint8_t checksum;       // Hata kontrolü
} __attribute__((packed)) Autonomous_Data_t;

// STM32'den Python'a gidecek komut paketi
typedef struct
{
    uint8_t header;         // 0xBB sabiti
    float steer_cmd;        // Hesaplanan direksiyon açısı
    float throttle_cmd;     // Hesaplanan gaz/fren
    uint8_t checksum;
} Control_Command_t;

// Fonksiyon Prototipleri
void HIL_Proto_Init(void);
uint8_t HIL_Validate_Checksum(Autonomous_Data_t *data);

#endif /* INC_HIL_PROTO_H_ */
