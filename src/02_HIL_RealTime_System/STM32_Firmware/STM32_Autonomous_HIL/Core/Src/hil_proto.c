#include "hil_proto.h"

// Checksum hesabı (Basit XOR yöntemi)
uint8_t HIL_Validate_Checksum(Autonomous_Data_t *data)
{
    uint8_t calculated = data->header ^ (uint8_t)data->lane_error ^ data->light_status ^ data->obs_flags;
    return (calculated == data->checksum);
}


