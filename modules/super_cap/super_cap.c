#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"


static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    SuperCap_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &super_cap_instance->cap_msg;
    Msg->input_vol = (int16_t)(rxbuff[1] << 8 | rxbuff[0]);
    Msg->cap_vol = (int16_t)(rxbuff[3] << 8 | rxbuff[2]);
    Msg->input_cur = (int16_t)(rxbuff[5] << 8 | rxbuff[4]);
    Msg->power_set = (int16_t)(rxbuff[7] << 8 | rxbuff[6]);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);

    PIDInit(&super_cap_instance->buffer_pid, &supercap_config->buffer_config_pid);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

void SuperCapSetPower(SuperCapInstance *instance, float power_set)
{
    uint16_t tmpPower = (uint16_t)(power_set * 100);
    uint8_t data[8] = {0};
    data[0] = tmpPower >> 8;
    data[1] = tmpPower;
    SuperCapSend(instance,data);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}