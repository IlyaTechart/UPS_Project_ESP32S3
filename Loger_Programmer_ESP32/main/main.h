
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>


typedef enum{
    WEB_FACE, 
    LOGGER,
    BRIDGE
}DeviceFuncState_t;

typedef enum{
    Rx_DATA_NOP,
    RX_DATA_CMPT
}RX_USB_data_state_t;