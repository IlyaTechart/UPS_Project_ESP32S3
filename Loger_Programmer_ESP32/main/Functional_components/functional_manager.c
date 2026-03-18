
#include "functional_manager.h"


// Текущее состояние — какие компоненты сейчас живые
static ComponentMask_t g_initialized = COMP_NONE;

////////////////////////////////////////////////////////////////////////////////////////////
// Что нужно для каждого режима
static const ComponentMask_t MODE_REQUIRES[] = {
    [BRIDGE]   = COMP_TUSB | COMP_BRIDGE | COMP_MSC | COMP_DEBUG_PROB,
    [LOGGER]   = COMP_TUSB | COMP_SPI | COMP_LOGGER,
    [WEB_FACE] = COMP_TUSB | COMP_SPI | COMP_LOGGER | COMP_WIFI,
};
///////////////////////////////////////////////////////////////////////////////////////////
RX_USB_data_state_t RX_USB_data_state = Rx_DATA_NOP;
extern QueueHandle_t queue_serial_RX;

static const char *TAG = "Maneger";

static void main_Task(void *pvParameters);
static void Func_manege(SetCommand_t STATE);

void Function_Init(void)
{
    BaseType_t  xTaskReturned = {0};

    xTaskReturned = xTaskCreatePinnedToCore( main_Task, "Main Taks", 4 * 1024, NULL, 8, NULL, 0);
    if(xTaskReturned != pdPASS)
    {
        ESP_LOGE(TAG, "IS NOT CREATED: Main Taks");
    }

    Func_manege(RESET_LOGGER);                       // Начальное функциональное состояние устройства 
    Func_manege(RESET_WEB);

    COMP_SET(COMP_TUSB);

}

static void main_Task(void *pvParameters)
{
    uint8_t cmd_buf[CDC_CMD_COMAND_SIZE];
    memset(cmd_buf,0x00,sizeof(cmd_buf));
    uint8_t crc = 0;

    for(;;)
    {
        if (xQueueReceive(queue_serial_RX, cmd_buf, portMAX_DELAY) != pdPASS) continue;

        if( (cmd_buf[0] == 0xAA) && (cmd_buf[1] == 0x55) && (cmd_buf[7] == 0x55))
        {
            crc = 0xFF^cmd_buf[2]^cmd_buf[3];
            if(crc == cmd_buf[6])
            {
                Func_manege((SetCommand_t)cmd_buf[3]);
                memset(cmd_buf,0x00,sizeof(cmd_buf));

            }else{
                memset(cmd_buf,0x00,sizeof(cmd_buf));
                ESP_LOGE(TAG, "USB Massage FAILED!: Wrong CRC");
            }
        }
        

        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

static void Func_manege(SetCommand_t STATE)
{
    switch (STATE)
    {
    case SET_BRIDGE:

        if (!COMP_IS_INIT(MODE_REQUIRES[BRIDGE]))
        {
            //TODO: bridge init
            COMP_SET(COMP_TUSB | COMP_BRIDGE | COMP_MSC | COMP_DEBUG_PROB);
            ESP_LOGI(TAG, "APP TO: BRIDGE (resume)");
        }
        else
        {
            ESP_LOGI(TAG, "BRIDGE already resume — no action");
        }
        break;

    case SET_LOGGER:

        if (!COMP_IS_INIT(MODE_REQUIRES[LOGGER]))
        {
            /* Задачи уже созданы, просто возобновляем */
            spi_slave_resume();
            logger_resume();
            COMP_SET(COMP_TUSB | COMP_SPI | COMP_LOGGER);
            ESP_LOGI(TAG, "APP TO: LOGGER (resume)");
        }
        else
        {
            ESP_LOGI(TAG, "LOGGER already resume — no action");
        }
        break;

    case SET_WEB:

        if (!COMP_IS_INIT(MODE_REQUIRES[WEB_FACE]))
        {
            /* WiFi-драйвер и netif уже живые, только стартуем WiFi и HTTP */
            wifi_web_resume();
            COMP_SET(COMP_TUSB | COMP_SPI | COMP_LOGGER | COMP_WIFI);
            ESP_LOGI(TAG, "APP TO: WEB_FACE (resume)");
        }
        else
        {
            ESP_LOGI(TAG, "WEB_FACE already resume — no action");
        }
        break;

    case RESET_BRIDGE:

        if (COMP_IS_INIT(MODE_REQUIRES[BRIDGE]))
        {
            //TODO: bridge suspend
            COMP_CLEAR(COMP_BRIDGE | COMP_MSC | COMP_DEBUG_PROB);
            ESP_LOGI(TAG, "Suspend: BRIDGE");
        }
        else
        {
            ESP_LOGI(TAG, "BRIDGE already suspend");
        }
        break;

    case RESET_LOGGER:

        if (COMP_IS_INIT(MODE_REQUIRES[LOGGER]))
        {
            /* Приостанавливаем задачи, память остаётся — без фрагментации */
            spi_slave_suspend();
            logger_suspend();
            COMP_CLEAR(COMP_SPI | COMP_LOGGER);
            /* Бит инициализации НЕ сбрасываем: память выделена, задачи живы (suspended) */
            ESP_LOGI(TAG, "Suspend: LOGGER");
        }
        else
        {
            ESP_LOGI(TAG, "LOGGER already suspend");
        }
        break;

    case RESET_WEB:

        if (COMP_IS_INIT(MODE_REQUIRES[WEB_FACE]))
        {
            /* Останавливаем WiFi и HTTP, драйвер и netif остаются в памяти */
            wifi_web_suspend();
            COMP_CLEAR(COMP_SPI | COMP_LOGGER | COMP_WIFI);
            /* Бит инициализации НЕ сбрасываем: драйвер жив, resume не нужна повторная инит */
            ESP_LOGI(TAG, "Suspend: WEB_FACE");
        }
        else
        {
            ESP_LOGI(TAG, "WEB_FACE already suspend");
        }
        break;

    default:
        ESP_LOGI(TAG, "WRONG COMMAND");
        break;
    }

}
