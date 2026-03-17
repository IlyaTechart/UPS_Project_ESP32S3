
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
static void switch_mode(void);

void Function_Init(void)
{
    BaseType_t  xTaskReturned = {0};

    xTaskReturned = xTaskCreatePinnedToCore( main_Task, "Main Taks", 4 * 1024, NULL, 8, NULL, 0);
    if(xTaskReturned != pdPASS)
    {
        ESP_LOGE(TAG, "IS NOT CREATED: Main Taks");
    }

}

static void main_Task(void *pvParameters)
{
    uint8_t cmd_buf[CFG_TUD_CDC_RX_BUFSIZE * 2];
    uint8_t cnt = 0;
    uint8_t crc = 0;

    for(;;)
    {
        if (xQueueReceive(queue_serial_RX, &cmd_buf[cnt], portMAX_DELAY) != pdPASS) continue;
        if(cnt % (CFG_TUD_CDC_RX_BUFSIZE * 2))
        {
            cnt += CFG_TUD_CDC_RX_BUFSIZE;
        }else{
            cnt = 0;
        }

        for(uint8_t i = 0; i < 128; i++ )
        {
            if( (cmd_buf[i] == 0xAA) && (cmd_buf[i + 1] == 0x55) && (cmd_buf[i + 7] == 0x55))
            {
                crc = 0xFF^cmd_buf[i + 2]^cmd_buf[i + 3];
                if(crc == cmd_buf[i + 6])
                {
                    Func_manege((SetCommand_t)cmd_buf[i + 3]);
                    break;

                }else{
                    ESP_LOGE(TAG, "USB Massage FAILED!: Wrong CRC");
                    break;
                }
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
            COMP_SET(MODE_REQUIRES[BRIDGE]);
            ESP_LOGI(TAG, "APP TO: BRIDGE (init)");
        }
        else
        {
            ESP_LOGI(TAG, "BRIDGE already init — no action");
        }
        break;

    case SET_LOGGER:

        if (COMP_IS_INIT(COMP_SPI | COMP_LOGGER))
        {
            /* Задачи уже созданы, просто возобновляем */
            spi_slave_resume();
            logger_resume();
            ESP_LOGI(TAG, "APP TO: LOGGER (resume)");
        }
        else
        {
            /* Первый запуск — полная инициализация с выделением памяти */
            spi_slave_init();
            logger_Inint();
            COMP_SET(COMP_SPI | COMP_LOGGER);
            ESP_LOGI(TAG, "APP TO: LOGGER (init)");
        }
        break;

    case SET_WEB:

        if (COMP_IS_INIT(COMP_WIFI))
        {
            /* WiFi-драйвер и netif уже живые, только стартуем WiFi и HTTP */
            wifi_web_resume();
            ESP_LOGI(TAG, "APP TO: WEB_FACE (resume)");
        }
        else
        {
            /* Первый запуск — полная инициализация */
            wifi_web_init();
            COMP_SET(COMP_WIFI);
            ESP_LOGI(TAG, "APP TO: WEB_FACE (init)");
        }
        break;

    case RESET_BRIDGE:

        if (COMP_IS_INIT(MODE_REQUIRES[BRIDGE]))
        {
            //TODO: bridge suspend
            COMP_CLEAR(MODE_REQUIRES[BRIDGE]);
            ESP_LOGI(TAG, "Suspend: BRIDGE");
        }
        else
        {
            ESP_LOGI(TAG, "BRIDGE already stopped");
        }
        break;

    case RESET_LOGGER:

        if (COMP_IS_INIT(COMP_SPI | COMP_LOGGER))
        {
            /* Приостанавливаем задачи, память остаётся — без фрагментации */
            spi_slave_suspend();
            logger_suspend();
            /* Бит инициализации НЕ сбрасываем: память выделена, задачи живы (suspended) */
            ESP_LOGI(TAG, "Suspend: LOGGER");
        }
        else
        {
            ESP_LOGI(TAG, "LOGGER already stopped");
        }
        break;

    case RESET_WEB:

        if (COMP_IS_INIT(COMP_WIFI))
        {
            /* Останавливаем WiFi и HTTP, драйвер и netif остаются в памяти */
            wifi_web_suspend();
            /* Бит инициализации НЕ сбрасываем: драйвер жив, resume не нужна повторная инит */
            ESP_LOGI(TAG, "Suspend: WEB_FACE");
        }
        else
        {
            ESP_LOGI(TAG, "WEB_FACE already stopped");
        }
        break;

    default:
        ESP_LOGI(TAG, "WRONG COMMAND");
        break;
    }

}

static void switch_mode()
{

}