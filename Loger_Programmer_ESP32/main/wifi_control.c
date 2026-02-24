#include "esp_timer.h"
#include "wifi_control.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> // Для sqrt
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
// #define EXAMPLE_ESP_WIFI_SSID      "TP-Link_CDED"
// #define EXAMPLE_ESP_WIFI_PASS      "95257006"
#define EXAMPLE_MAX_RETRY          5
#define LED_GPIO                   13 

static const char *TAG = "WEB_CTRL";
static int s_retry_num = 0;
static int s_led_state = 0;

// Ссылка на глобальные данные (определены в main.c или msp.c)
extern volatile ModulData_t ModulData;

static const char* index_html = 
"<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>Modular UPS</title>"
"<style>"
"body { font-family: 'Segoe UI', Arial, sans-serif; background-color: #eef2f5; margin: 0; padding: 0; display: flex; justify-content: center; height: 100vh; }"
".container { width: 95%; max-width: 1000px; background-color: white; height: 90vh; margin-top: 2vh; display: flex; flex-direction: column; box-shadow: 0 4px 15px rgba(0,0,0,0.1); border-radius: 8px; overflow: hidden; }"
/* Header */
".header { background-color: #2c3e50; color: white; padding: 15px; display: flex; justify-content: space-between; align-items: center; }"
".header h2 { margin: 0; font-size: 20px; }"
/* Tabs */
".tabs { display: flex; background-color: #34495e; }"
".tab { flex: 1; padding: 12px; text-align: center; cursor: pointer; color: #bdc3c7; border-bottom: 3px solid transparent; transition: 0.3s; }"
".tab:hover { background-color: #3e5871; color: white; }"
".tab.active { background-color: #ecf0f1; color: #2c3e50; border-bottom: 3px solid #3498db; font-weight: bold; }"
/* Content Area */
".content-wrapper { flex: 1; overflow-y: auto; padding: 20px; background-color: #ecf0f1; }"
".tab-content { display: none; animation: fadeIn 0.3s; }"
".tab-content.active { display: block; }"
"@keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }"
/* Data Groups */
".group-box { background: white; border-radius: 6px; padding: 15px; margin-bottom: 15px; box-shadow: 0 2px 5px rgba(0,0,0,0.05); }"
".group-title { border-bottom: 1px solid #eee; padding-bottom: 8px; margin-bottom: 10px; font-weight: bold; color: #3498db; text-transform: uppercase; font-size: 14px; }"
".row { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px dashed #eee; font-size: 14px; }"
".row:last-child { border-bottom: none; }"
".lbl { color: #555; }"
".val { font-weight: bold; color: #333; }"
/* Status Indicators */
".badge { padding: 3px 8px; border-radius: 4px; font-size: 12px; color: white; }"
".bg-ok { background-color: #27ae60; }"
".bg-warn { background-color: #f39c12; }"
".bg-err { background-color: #c0392b; }"
/* Footer */
".footer { background-color: #2c3e50; color: #bdc3c7; padding: 10px 20px; font-size: 12px; display: flex; justify-content: space-between; }"
"</style></head><body>"

"<div class=\"container\">"
"  <div class=\"header\">"
"    <h2>UPS Module Monitor</h2>"
"    <div id=\"sys_status\" class=\"badge bg-warn\">Connecting...</div>"
"  </div>"

"  <div class=\"tabs\">"
"    <div class=\"tab active\" onclick=\"openTab('rect')\">Rectifier (AC/DC)</div>"
"    <div class=\"tab\" onclick=\"openTab('inv')\">Inverter (DC/AC)</div>"
"    <div class=\"tab\" onclick=\"openTab('sts')\">Static Switch (STS)</div>"
"  </div>"
  
"  <div class=\"content-wrapper\">"
    
    /* === TAB 1: RECTIFIER === */
"    <div id=\"rect\" class=\"tab-content active\">"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Main Input (Mains)</div>"
"        <div class=\"row\"><span class=\"lbl\">Voltage A/B/C (V)</span><span class=\"val\"><span id=\"v_in_ab\">-</span> / <span id=\"v_in_bc\">-</span> / <span id=\"v_in_ca\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Current A/B/C (A)</span><span class=\"val\"><span id=\"i_in_a\">-</span> / <span id=\"i_in_b\">-</span> / <span id=\"i_in_c\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Frequency (Hz)</span><span class=\"val\" id=\"freq_in\">-</span></div>"
"      </div>"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Rectifier Status</div>"
"        <div class=\"row\"><span class=\"lbl\">Operation State</span><span class=\"val\" id=\"rect_st\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Grid Status</span><span class=\"val\" id=\"grid_st\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Temp / Overheat</span><span class=\"val\" id=\"rect_temp_alarm\">-</span></div>"
"      </div>"
"    </div>"

    /* === TAB 2: INVERTER === */
"    <div id=\"inv\" class=\"tab-content\">"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Output Parameters</div>"
"        <div class=\"row\"><span class=\"lbl\">Voltage A/B/C (V)</span><span class=\"val\"><span id=\"v_out_a\">-</span> / <span id=\"v_out_b\">-</span> / <span id=\"v_out_c\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Current A/B/C (A)</span><span class=\"val\"><span id=\"i_out_a\">-</span> / <span id=\"i_out_b\">-</span> / <span id=\"i_out_c\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Active Power (kW)</span><span class=\"val\"><span id=\"p_act_a\">-</span> / <span id=\"p_act_b\">-</span> / <span id=\"p_act_c\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Load (%)</span><span class=\"val\"><span id=\"load_a\">-</span> / <span id=\"load_b\">-</span> / <span id=\"load_c\">-</span></span></div>"
"        <div class=\"row\"><span class=\"lbl\">Frequency (Hz)</span><span class=\"val\" id=\"freq_out\">-</span></div>"
"      </div>"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Battery & DC Bus</div>"
"        <div class=\"row\"><span class=\"lbl\">DC Bus Voltage (V)</span><span class=\"val\" id=\"dc_bus\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Battery Voltage (V)</span><span class=\"val\" id=\"bat_v\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Battery Current (A)</span><span class=\"val\" id=\"bat_i\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Capacity / Backup</span><span class=\"val\"><span id=\"bat_cap\">-</span> Ah / <span id=\"bat_time\">-</span> min</span></div>"
"      </div>"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Inverter Status</div>"
"        <div class=\"row\"><span class=\"lbl\">State</span><span class=\"val\" id=\"inv_st\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Mode</span><span class=\"val\" id=\"ups_mode\">-</span></div>"
"      </div>"
"    </div>"

    /* === TAB 3: STS (BYPASS) === */
"    <div id=\"sts\" class=\"tab-content\">"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Bypass Input</div>"
"        <div class=\"row\"><span class=\"lbl\">Voltage A/B/C (V)</span><span class=\"val\"><span id=\"v_bp_a\">-</span> / <span id=\"v_bp_b\">-</span> / <span id=\"v_bp_c\">-</span></span></div>"
"      </div>"
"      <div class=\"group-box\">"
"        <div class=\"group-title\">Switch Status</div>"
"        <div class=\"row\"><span class=\"lbl\">Sync Status</span><span class=\"val\" id=\"sync_st\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Power via Bypass</span><span class=\"val\" id=\"via_bp\">-</span></div>"
"        <div class=\"row\"><span class=\"lbl\">Bypass Grid</span><span class=\"val\" id=\"bp_grid_st\">-</span></div>"
"      </div>"
"    </div>"
"  </div>" // content-wrapper

"  <div class=\"footer\">"
"    <span>API Status: OK</span>"
"    <span>Module FW: v1.0</span>"
"  </div>"
"</div>"

"<script>"
"function openTab(name) {"
"  var i; var x = document.getElementsByClassName('tab-content');"
"  for (i = 0; i < x.length; i++) { x[i].classList.remove('active'); }"
"  var tabs = document.getElementsByClassName('tab');"
"  for (i = 0; i < tabs.length; i++) { tabs[i].classList.remove('active'); }"
"  document.getElementById(name).classList.add('active');"
// Костыль, чтобы найти кнопку таба по тексту или порядку, но проще через event.currentTarget
// Здесь упростим логику выделения активного таба
"  event.currentTarget.classList.add('active');"
"}"

"function setTxt(id, val, fixed) {"
"  var el = document.getElementById(id);"
"  if(el) el.innerText = (fixed !== undefined) ? Number(val).toFixed(fixed) : val;"
"}"

"function setBadge(id, val, txtOk, txtErr) {"
"  var el = document.getElementById(id);"
"  if(!el) return;"
"  if(val == 0 || val == '0') { el.innerText = txtOk; el.className = 'val badge bg-ok'; }"
"  else { el.innerText = txtErr; el.className = 'val badge bg-err'; }"
"}"
"function setBool(id, val, txtTrue, txtFalse) {"
"  var el = document.getElementById(id);"
"  if(el) el.innerText = val ? txtTrue : txtFalse;"
"}"

"function update() {"
"  fetch('/api/status').then(r => r.json()).then(d => {"
     // System Status
"    var stEl = document.getElementById('sys_status');"
"    stEl.innerText = 'Connected'; stEl.className = 'badge bg-ok';"
     
     // --- RECTIFIER TAB ---
"    setTxt('v_in_ab', d.input.v_ab, 1);"
"    setTxt('v_in_bc', d.input.v_bc, 1);"
"    setTxt('v_in_ca', d.input.v_ca, 1);"
"    setTxt('i_in_a', d.input.i_a, 1); setTxt('i_in_b', d.input.i_b, 1); setTxt('i_in_c', d.input.i_c, 1);"
"    setTxt('freq_in', d.input.freq, 2);"
     // Status logic: 0=Norm, 1=Alarm for grid
"    setBadge('grid_st', d.status.grid, 'OK', 'FAIL');" 
"    setBool('rect_st', d.status.rect, 'Running', 'Stopped');"
"    setBadge('rect_temp_alarm', d.alarms.rect_hot, 'Normal', 'OVERHEAT');"

     // --- INVERTER TAB ---
"    setTxt('v_out_a', d.output.v_a, 1); setTxt('v_out_b', d.output.v_b, 1); setTxt('v_out_c', d.output.v_c, 1);"
"    setTxt('i_out_a', d.output.i_a, 1); setTxt('i_out_b', d.output.i_b, 1); setTxt('i_out_c', d.output.i_c, 1);"
"    setTxt('p_act_a', d.output.p_act_a, 1); setTxt('p_act_b', d.output.p_act_b, 1); setTxt('p_act_c', d.output.p_act_c, 1);"
"    setTxt('load_a', d.output.load_a, 0); setTxt('load_b', d.output.load_b, 0); setTxt('load_c', d.output.load_c, 0);"
"    setTxt('freq_out', d.output.freq, 2);"
"    setTxt('dc_bus', d.bat.dc_bus, 1);"
"    setTxt('bat_v', d.bat.v, 1);"
"    setTxt('bat_i', d.bat.curr, 1);"
"    setTxt('bat_cap', d.bat.cap, 0);"
"    setTxt('bat_time', d.bat.time, 0);"
"    setBool('inv_st', d.status.inv, 'Running', 'Stopped');"
"    setBool('ups_mode', d.status.mode, 'Battery Mode', 'Line Mode');"

     // --- STS TAB ---
"    setTxt('v_bp_a', d.input.v_bp_a, 1); setTxt('v_bp_b', d.input.v_bp_b, 1); setTxt('v_bp_c', d.input.v_bp_c, 1);"
"    setBadge('sync_st', d.status.sync, 'Synchronized', 'Async');" // Note: Check struct logic (0=Sync usually?)
"    setBool('via_bp', d.status.via_bp, 'Yes', 'No');"
"    setBadge('bp_grid_st', d.status.bp_grid, 'OK', 'FAIL');"

"  }).catch(e => {"
"    var stEl = document.getElementById('sys_status');"
"    stEl.innerText = 'Disconnected'; stEl.className = 'badge bg-err';"
"  });"
"}"
"setInterval(update, 1000);"
"update();"
"</script></body></html>";


/* --- 3. ФУНКЦИЯ ФОРМИРОВАНИЯ JSON --- */
/* Создает JSON строку из структуры данных. Caller должен освободить память (free). */
char* generate_ups_json_string(void)
{
     cJSON *root = cJSON_CreateObject();
    
    // --- 1. STATUS ---
    cJSON *status = cJSON_CreateObject();
    cJSON_AddNumberToObject(status, "grid", ModulData.packet.status.grid_status);
    cJSON_AddNumberToObject(status, "bp_grid", ModulData.packet.status.bypass_grid_status);
    cJSON_AddNumberToObject(status, "rect", ModulData.packet.status.rectifier_status);
    cJSON_AddNumberToObject(status, "inv", ModulData.packet.status.inverter_status);
    cJSON_AddNumberToObject(status, "via_inv", ModulData.packet.status.pwr_via_inverter);
    cJSON_AddNumberToObject(status, "via_bp", ModulData.packet.status.pwr_via_bypass);
    cJSON_AddNumberToObject(status, "sync", ModulData.packet.status.sync_status);
    cJSON_AddNumberToObject(status, "load_m", ModulData.packet.status.load_mode);
    cJSON_AddNumberToObject(status, "bat", ModulData.packet.status.battery_status);
    cJSON_AddNumberToObject(status, "mode", ModulData.packet.status.ups_mode);
    cJSON_AddItemToObject(root, "status", status);

    // --- 2. ALARMS ---
    cJSON *alarms = cJSON_CreateObject();
    cJSON_AddNumberToObject(alarms, "low_in", ModulData.packet.alarms.err_low_input_vol);
    cJSON_AddNumberToObject(alarms, "hi_dc", ModulData.packet.alarms.err_high_dc_bus);
    cJSON_AddNumberToObject(alarms, "low_bat", ModulData.packet.alarms.err_low_bat_charge);
    cJSON_AddNumberToObject(alarms, "no_bat", ModulData.packet.alarms.err_bat_not_conn);
    cJSON_AddNumberToObject(alarms, "inv_flt", ModulData.packet.alarms.err_inv_fault);
    cJSON_AddNumberToObject(alarms, "inv_oc", ModulData.packet.alarms.err_inv_overcurrent);
    cJSON_AddNumberToObject(alarms, "hi_out", ModulData.packet.alarms.err_high_out_vol);
    cJSON_AddNumberToObject(alarms, "fan", ModulData.packet.alarms.err_fan_fault);
    cJSON_AddNumberToObject(alarms, "bat_repl", ModulData.packet.alarms.err_replace_bat);
    cJSON_AddNumberToObject(alarms, "rect_hot", ModulData.packet.alarms.err_rect_overheat);
    cJSON_AddNumberToObject(alarms, "inv_hot", ModulData.packet.alarms.err_inv_overheat);
    cJSON_AddItemToObject(root, "alarms", alarms);

    // --- 3. INPUT (Rectifier Input + Bypass) ---
    cJSON *input = cJSON_CreateObject();
    cJSON_AddNumberToObject(input, "v_ab", ModulData.packet.input.v_in_AB);
    cJSON_AddNumberToObject(input, "v_bc", ModulData.packet.input.v_in_BC);
    cJSON_AddNumberToObject(input, "v_ca", ModulData.packet.input.v_in_CA);
    cJSON_AddNumberToObject(input, "v_bp_a", ModulData.packet.input.v_bypass_A);
    cJSON_AddNumberToObject(input, "v_bp_b", ModulData.packet.input.v_bypass_B);
    cJSON_AddNumberToObject(input, "v_bp_c", ModulData.packet.input.v_bypass_C);
    cJSON_AddNumberToObject(input, "i_a", ModulData.packet.input.i_in_A);
    cJSON_AddNumberToObject(input, "i_b", ModulData.packet.input.i_in_B);
    cJSON_AddNumberToObject(input, "i_c", ModulData.packet.input.i_in_C);
    cJSON_AddNumberToObject(input, "freq", ModulData.packet.input.freq_in);
    cJSON_AddItemToObject(root, "input", input);

    // --- 4. OUTPUT (Inverter) ---
    cJSON *output = cJSON_CreateObject();
    cJSON_AddNumberToObject(output, "v_a", ModulData.packet.output.v_out_A);
    cJSON_AddNumberToObject(output, "v_b", ModulData.packet.output.v_out_B);
    cJSON_AddNumberToObject(output, "v_c", ModulData.packet.output.v_out_C);
    cJSON_AddNumberToObject(output, "freq", ModulData.packet.output.freq_out);
    cJSON_AddNumberToObject(output, "i_a", ModulData.packet.output.i_out_A);
    cJSON_AddNumberToObject(output, "i_b", ModulData.packet.output.i_out_B);
    cJSON_AddNumberToObject(output, "i_c", ModulData.packet.output.i_out_C);
    cJSON_AddNumberToObject(output, "p_act_a", ModulData.packet.output.p_active_A);
    cJSON_AddNumberToObject(output, "p_act_b", ModulData.packet.output.p_active_B);
    cJSON_AddNumberToObject(output, "p_act_c", ModulData.packet.output.p_active_C);
    cJSON_AddNumberToObject(output, "load_a", ModulData.packet.output.load_pct_A);
    cJSON_AddNumberToObject(output, "load_b", ModulData.packet.output.load_pct_B);
    cJSON_AddNumberToObject(output, "load_c", ModulData.packet.output.load_pct_C);
    cJSON_AddItemToObject(root, "output", output);

    // --- 5. BATTERY ---
    cJSON *bat = cJSON_CreateObject();
    cJSON_AddNumberToObject(bat, "v", ModulData.packet.battery.bat_voltage);
    cJSON_AddNumberToObject(bat, "dc_bus", ModulData.packet.battery.dc_bus_voltage);
    cJSON_AddNumberToObject(bat, "curr", ModulData.packet.battery.bat_current);
    cJSON_AddNumberToObject(bat, "cap", ModulData.packet.battery.bat_capacity);
    cJSON_AddNumberToObject(bat, "time", ModulData.packet.battery.backup_time);
    cJSON_AddItemToObject(root, "bat", bat);

    // Генерация строки (без форматирования для экономии байт)
    char *string = cJSON_PrintUnformatted(root);
    
    // Удаляем объект JSON, чтобы не было утечки памяти (строка string останется)
    cJSON_Delete(root);

    return string;
}


/* --- 2. МОДЕРНИЗИРОВАННЫЙ ОБРАБОТЧИК STATUS --- */
static esp_err_t status_handler(httpd_req_t *req)
{
    // 1. Формируем JSON с помощью отдельной функции
    char *json_response = generate_ups_json_string();

    if (json_response == NULL) {
        ESP_LOGE(TAG, "Failed to generate JSON");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 2. Отправляем заголовки и данные
    httpd_resp_set_type(req, "application/json");
    // Разрешаем CORS (опционально, для отладки)
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); 
    
    httpd_resp_send(req, json_response, strlen(json_response));

    // 3. Освобождаем память, выделенную cJSON
    free(json_response);

    return ESP_OK;
}


/* --- Остальной код Wi-Fi и HTTP (без изменений логики) --- */

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
    // Пример обработки set_id. Нужно добавить поле id в ModulData если требуется сохранять
    else if (strncmp(content, "set_id:", 7) == 0) {
        // int new_id = atoi(content + 7);
        // ModulData.id = new_id;
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; 
    config.max_uri_handlers = 8;
    config.max_uri_len = 1024;
    config.max_req_hdr_len = 2048;
    
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