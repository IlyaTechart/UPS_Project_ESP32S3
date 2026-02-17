#include "esp_timer.h"
#include "wifi_control.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "cJSON.h" 
#include "spi_handler.h"

#define EXAMPLE_ESP_WIFI_SSID      "PED"
#define EXAMPLE_ESP_WIFI_PASS      "kt7DMstt"
#define EXAMPLE_MAX_RETRY          5
#define LED_GPIO                   13 

static const char *TAG = "WEB_CTRL";
static int s_retry_num = 0;
static int s_led_state = 0;


// Инициализация стартовыми значениями
UpsData_t ups_data = {
    .v_in_a = 228.8, .v_in_b = 228.7, .v_in_c = 227.2,
    .c_in_a = 2.2,   .c_in_b = 2.2,   .c_in_c = 1.7,
    .f_in_a = 50.01, .f_in_b = 50.03, .f_in_c = 50.03,
    
    .p_act_a = 0.5,  .p_act_b = 0.5,  .p_act_c = 0.3,
    .p_rea_a = 0.1,  .p_rea_b = 0.1,  .p_rea_c = 0.0,
    .load_a = 0.4,   .load_b = 0.3,   .load_c = 0.1,
    
    .fan_time = 2753,
    .module_id = 8
};
extern volatile TransferBuffer_t TransferBuffer;

/* --- HTML Страница --- */
static const char* index_html = 
"<!DOCTYPE html><html><head><meta charset='utf-8'><title>UPS POWER MTR</title>"
"<style>"
"body { margin: 0; padding: 0; background: linear-gradient(to bottom, #6cbdfa, #bfdfff); font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; height: 100vh; overflow: hidden; }"
".main-window { width: 95%; max-width: 1000px; margin: 20px auto; background-color: #badcf7; border: 2px solid #fff; box-shadow: 0 0 10px rgba(0,0,0,0.3); height: 90vh; display: flex; flex-direction: column; }"
/* Вкладки */
".tabs { display: flex; background: #e0eef9; padding-left: 5px; border-bottom: 1px solid #999; }"
".tab { padding: 5px 15px; margin-top: 5px; background: #f0f0f0; border: 1px solid #999; border-bottom: none; cursor: pointer; color: #555; }"
".tab.active { background: #fff; font-weight: bold; color: #000; position: relative; top: 1px; height: 22px; }"
/* Область данных */
".content { flex-grow: 1; background: #fff; padding: 0; overflow-y: auto; display: flex; position: relative; border: 1px solid #7a98af; margin: 5px; }"
".col { width: 50%; border-right: 1px solid #ccc; }"
".row { display: flex; justify-content: space-between; padding: 3px 5px; border-bottom: 1px solid #e0e0e0; font-size: 14px; }"
".row:nth-child(even) { background-color: #f2f6fa; }"
".row:hover { background-color: #cce8ff; }"
".lbl { color: #000; }"
".val { font-weight: bold; color: #333; min-width: 60px; text-align: right; }"
/* Нижняя панель */
".bottom-panel { background: #5c9ce6; padding: 10px; display: flex; align-items: center; border-top: 2px solid #fff; color: white; font-weight: bold; }"
".control-group { display: flex; align-items: center; margin-right: 20px; }"
"input { padding: 3px; width: 60px; margin: 0 5px; border: 1px solid #fff; text-align: center; }"
"button { padding: 4px 15px; background: #eee; border: 1px solid #555; cursor: pointer; font-weight: bold; }"
"button:active { background: #ccc; }"
".status-txt { margin-left: auto; font-size: 12px; font-weight: normal; }"
"</style></head><body>"

"<div class='main-window'>"
"  <div class='tabs'>"
"    <div class='tab'>Unit Status</div>"
"    <div class='tab active'>Module Data</div>"
"  </div>"

"  <div class='content'>"
    // Левая колонка (IDs: via, vib, vic...)
"    <div class='col'>"
"      <div class='row'><span class='lbl'>Main Input Voltage Phase A(V)</span><span class='val' id='via'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Voltage Phase B(V)</span><span class='val' id='vib'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Voltage Phase C(V)</span><span class='val' id='vic'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Current Phase A(A)</span><span class='val' id='cia'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Current Phase B(A)</span><span class='val' id='cib'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Current Phase C(A)</span><span class='val' id='cic'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Frequency Phase A(Hz)</span><span class='val' id='fia'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Frequency Phase B(Hz)</span><span class='val' id='fib'>-</span></div>"
"      <div class='row'><span class='lbl'>Main Input Frequency Phase C(Hz)</span><span class='val' id='fic'>-</span></div>"
"    </div>"
    // Правая колонка (IDs: paa, pab, pac...)
"    <div class='col'>"
"      <div class='row'><span class='lbl'>Output Active Power Phase A(kW)</span><span class='val' id='paa'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Active Power Phase B(kW)</span><span class='val' id='pab'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Active Power Phase C(kW)</span><span class='val' id='pac'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Reactive Power Phase A(kVar)</span><span class='val' id='pra'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Reactive Power Phase B(kVar)</span><span class='val' id='prb'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Reactive Power Phase C(kVar)</span><span class='val' id='prc'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Load Percentage Phase A(%)</span><span class='val' id='lda'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Load Percentage Phase B(%)</span><span class='val' id='ldb'>-</span></div>"
"      <div class='row'><span class='lbl'>Output Load Percentage Phase C(%)</span><span class='val' id='ldc'>-</span></div>"
"      <div class='row'><span class='lbl'>Fan Running Time (hour)</span><span class='val' id='fan'>-</span></div>"
"    </div>"
"  </div>"

"  <div class='bottom-panel'>"
"    <div class='control-group'>"
"      Module ID "
"      <input type='number' id='mod_id_val' value='8'> "
"      <button onclick='setModuleId()'>Set</button>"
"    </div>"
"    <div class='control-group'>"
"       <button onclick='toggleLed()'>Toggle LED</button>" 
"    </div>"
"    <span class='status-txt' id='status_line'>Connecting...</span>"
"  </div>"
"</div>"

"<script>"
"function toggleLed() { fetch('/api/cmd', {method: 'POST', body: 'toggle_led'}); }"
"function setModuleId() {"
"  let val = document.getElementById('mod_id_val').value;"
"  fetch('/api/cmd', {method: 'POST', body: 'set_id:' + val});"
"  document.getElementById('status_line').innerText = 'Setting ID...';"
"}"

"function updateStatus() {"
"  fetch('/api/status').then(r => r.json()).then(d => {"
     // Заполняем Левую колонку (ключи JSON совпадают с ID в HTML)
"    document.getElementById('via').innerText = d.via.toFixed(1);"
"    document.getElementById('vib').innerText = d.vib.toFixed(1);"
"    document.getElementById('vic').innerText = d.vic.toFixed(1);"
"    document.getElementById('cia').innerText = d.cia.toFixed(1);"
"    document.getElementById('cib').innerText = d.cib.toFixed(1);"
"    document.getElementById('cic').innerText = d.cic.toFixed(1);"
"    document.getElementById('fia').innerText = d.fia.toFixed(2);"
"    document.getElementById('fib').innerText = d.fib.toFixed(2);"
"    document.getElementById('fic').innerText = d.fic.toFixed(2);"
     // Заполняем Правую колонку
"    document.getElementById('paa').innerText = d.paa.toFixed(1);"
"    document.getElementById('pab').innerText = d.pab.toFixed(1);"
"    document.getElementById('pac').innerText = d.pac.toFixed(1);"
"    document.getElementById('pra').innerText = d.pra.toFixed(1);"
"    document.getElementById('prb').innerText = d.prb.toFixed(1);"
"    document.getElementById('prc').innerText = d.prc.toFixed(1);"
"    document.getElementById('lda').innerText = d.lda.toFixed(1);"
"    document.getElementById('ldb').innerText = d.ldb.toFixed(1);"
"    document.getElementById('ldc').innerText = d.ldc.toFixed(1);"
"    document.getElementById('fan').innerText = d.fan;"
     // Нижняя панель
"    if(document.activeElement.id !== 'mod_id_val') document.getElementById('mod_id_val').value = d.mid;"
"    document.getElementById('status_line').innerText = 'Connected. Uptime: ' + d.up + 's';"
"  }).catch(e => {"
"    console.log(e);"
"    document.getElementById('status_line').innerText = 'Connection Lost';"
"  });"
"}"
"setInterval(updateStatus, 500);"
"</script></body></html>";


/* --- Wi-Fi Event Handler --- */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
    }
}

/* --- HTTP Handlers --- */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) return ESP_FAIL;
    content[ret] = 0; 

    if (strcmp(content, "toggle_led") == 0) {
        s_led_state = !s_led_state;
        gpio_set_level(LED_GPIO, s_led_state);
    } 
    else if (strncmp(content, "set_id:", 7) == 0) {
        ups_data.module_id = atoi(content + 7);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Отправка статуса (GET) - Формируем JSON
static esp_err_t status_handler(httpd_req_t *req)
{
    /*
    Simulation DATA 
    */
   // Voltage 
   memcpy(&ups_data, &TransferBuffer, sizeof(ups_data));


    
    char resp_str[800]; // Увеличили буфер
    
    // ВНИМАНИЕ: Ключи здесь (via, vib) должны совпадать с JS (d.via, d.vib)
    snprintf(resp_str, sizeof(resp_str), 
             "{"
             "\"via\":%.2f,\"vib\":%.2f,\"vic\":%.2f,"
             "\"cia\":%.2f,\"cib\":%.2f,\"cic\":%.2f,"
             "\"fia\":%.2f,\"fib\":%.2f,\"fic\":%.2f,"
             "\"paa\":%.2f,\"pab\":%.2f,\"pac\":%.2f,"
             "\"pra\":%.2f,\"prb\":%.2f,\"prc\":%.2f,"
             "\"lda\":%.2f,\"ldb\":%.2f,\"ldc\":%.2f,"
             "\"fan\":%d,\"mid\":%d,\"up\":%lld"
             "}", 
             ups_data.v_in_a, ups_data.v_in_b, ups_data.v_in_c,
             ups_data.c_in_a, ups_data.c_in_b, ups_data.c_in_c,
             ups_data.f_in_a, ups_data.f_in_b, ups_data.f_in_c,
             ups_data.p_act_a, ups_data.p_act_b, ups_data.p_act_c,
             ups_data.p_rea_a, ups_data.p_rea_b, ups_data.p_rea_c,
             ups_data.load_a, ups_data.load_b, ups_data.load_c,
             ups_data.fan_time, ups_data.module_id, esp_timer_get_time() / 1000000);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; 
    config.max_uri_handlers = 8;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_index = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_index);

        httpd_uri_t uri_cmd = { .uri = "/api/cmd", .method = HTTP_POST, .handler = cmd_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_cmd);

        httpd_uri_t uri_status = { .uri = "/api/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_status);

        return server;
    }
    return NULL;
}

void wifi_web_init(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    start_webserver();
}
void set_web_status_message(const char* msg) {}