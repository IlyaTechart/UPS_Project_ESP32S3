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
#define EXAMPLE_MAX_RETRY          5
#define LED_GPIO                   13 

static const char *TAG = "WEB_CTRL";
static int s_retry_num = 0;
static int s_led_state = 0;

// Ссылка на глобальные данные (определены в main.c или msp.c)
extern volatile ModulData_t ModulData;

/* --- 1. HTML СТРАНИЦА (Упакована в C-строку) --- */
/* Стиль подогнан под скриншот: светло-голубой фон, вкладки, таблица данных */
// static const char* index_html = 
// "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>UPS Monitor</title>"
// "<style>"
// "body { font-family: Arial, sans-serif; background-color: #99ccff; margin: 0; padding: 20px; display: flex; justify-content: center; height: 100vh; }"
// ".container { width: 900px; background-color: white; border: 1px solid #777; height: 85vh; display: flex; flex-direction: column; box-shadow: 2px 2px 10px rgba(0,0,0,0.2); }"
// /* Tabs */
// ".tabs { display: flex; background-color: #f0f0f0; border-bottom: 1px solid #aaa; }"
// ".tab { padding: 8px 20px; border-right: 1px solid #aaa; background-color: #e0e0e0; color: #555; cursor: pointer; font-size: 14px; }"
// ".tab.active { background-color: white; font-weight: bold; color: black; border-bottom: 1px solid white; margin-bottom: -1px; }"
// /* Content */
// ".content { flex: 1; display: flex; padding: 0; overflow-y: auto; }"
// ".col { width: 50%; border-right: 1px solid #ddd; }"
// ".row { display: flex; justify-content: space-between; padding: 5px 10px; border-bottom: 1px solid #eee; font-size: 13px; align-items: center; }"
// ".row:nth-child(odd) { background-color: #fcfcfc; }"
// ".row:hover { background-color: #eef7ff; }"
// ".lbl { color: #333; }"
// ".val { font-weight: bold; color: #000; }"
// /* Footer */
// ".footer { background-color: #5b9bd5; color: white; padding: 8px 15px; display: flex; justify-content: space-between; align-items: center; font-size: 13px; border-top: 1px solid #3377aa; }"
// ".btn-grp { display: flex; align-items: center; gap: 10px; }"
// "input { width: 40px; text-align: center; border: none; padding: 2px; }"
// "button { border: 1px solid #fff; background: #eee; cursor: pointer; padding: 2px 10px; font-weight: bold; color: #333; }"
// "button:hover { background: #fff; }"
// "</style></head><body>"

// "<div class=\"container\">"
// "  <div class=\"tabs\">"
// "    <div class=\"tab\">Unit Status</div>"
// "    <div class=\"tab active\">Module Data</div>"
// "  </div>"
  
// "  <div class=\"content\">"
//     /* ЛЕВАЯ КОЛОНКА */
// "    <div class=\"col\">"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Voltage Phase A(V)</span><span class=\"val\" id=\"v_in_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Voltage Phase B(V)</span><span class=\"val\" id=\"v_in_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Voltage Phase C(V)</span><span class=\"val\" id=\"v_in_c\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Current Phase A(A)</span><span class=\"val\" id=\"c_in_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Current Phase B(A)</span><span class=\"val\" id=\"c_in_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Current Phase C(A)</span><span class=\"val\" id=\"c_in_c\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Frequency Phase A(Hz)</span><span class=\"val\" id=\"f_in_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Frequency Phase B(Hz)</span><span class=\"val\" id=\"f_in_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Main Input Frequency Phase C(Hz)</span><span class=\"val\" id=\"f_in_c\">---</span></div>"
// "    </div>"
//     /* ПРАВАЯ КОЛОНКА */
// "    <div class=\"col\">"
// "      <div class=\"row\"><span class=\"lbl\">Output Active Power Phase A(kW)</span><span class=\"val\" id=\"p_act_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Active Power Phase B(kW)</span><span class=\"val\" id=\"p_act_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Active Power Phase C(kW)</span><span class=\"val\" id=\"p_act_c\">---</span></div>"
//       /* Реактивная мощность рассчитывается в C коде */
// "      <div class=\"row\"><span class=\"lbl\">Output Reactive Power Phase A(kVar)</span><span class=\"val\" id=\"p_rea_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Reactive Power Phase B(kVar)</span><span class=\"val\" id=\"p_rea_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Reactive Power Phase C(kVar)</span><span class=\"val\" id=\"p_rea_c\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Load Percentage Phase A(%)</span><span class=\"val\" id=\"load_a\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Load Percentage Phase B(%)</span><span class=\"val\" id=\"load_b\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Output Load Percentage Phase C(%)</span><span class=\"val\" id=\"load_c\">---</span></div>"
// "      <div class=\"row\"><span class=\"lbl\">Fan Running Time (hour)</span><span class=\"val\" id=\"fan_time\">---</span></div>"
// "    </div>"
// "  </div>"

// "  <div class=\"footer\">"
// "    <div class=\"btn-grp\">"
// "      <span>Module ID</span>"
// "      <input type=\"number\" id=\"mid_input\" value=\"8\">"
// "      <button onclick=\"setId()\">Set</button>"
// "      <button onclick=\"toggleLed()\">Toggle LED</button>"
// "    </div>"
// "    <div id=\"status\">Connected. Uptime: 0s</div>"
// "  </div>"
// "</div>"

// "<script>"
// "function toggleLed() { fetch('/api/cmd', {method: 'POST', body: 'toggle_led'}); }"
// "function setId() { fetch('/api/cmd', {method: 'POST', body: 'set_id:' + document.getElementById('mid_input').value}); }"
// "function update() {"
// "  fetch('/api/status').then(r => r.json()).then(d => {"
//      // Input Volts
// "    document.getElementById('v_in_a').innerText = d.in.va.toFixed(1);"
// "    document.getElementById('v_in_b').innerText = d.in.vb.toFixed(1);"
// "    document.getElementById('v_in_c').innerText = d.in.vc.toFixed(1);"
//      // Input Amps
// "    document.getElementById('c_in_a').innerText = d.in.ca.toFixed(1);"
// "    document.getElementById('c_in_b').innerText = d.in.cb.toFixed(1);"
// "    document.getElementById('c_in_c').innerText = d.in.cc.toFixed(1);"
//      // Input Freq
// "    document.getElementById('f_in_a').innerText = d.in.fa.toFixed(2);"
// "    document.getElementById('f_in_b').innerText = d.in.fb.toFixed(2);"
// "    document.getElementById('f_in_c').innerText = d.in.fc.toFixed(2);"
//      // Output Active Power
// "    document.getElementById('p_act_a').innerText = d.out.pa.toFixed(1);"
// "    document.getElementById('p_act_b').innerText = d.out.pb.toFixed(1);"
// "    document.getElementById('p_act_c').innerText = d.out.pc.toFixed(1);"
//      // Output Reactive Power
// "    document.getElementById('p_rea_a').innerText = d.out.qa.toFixed(1);"
// "    document.getElementById('p_rea_b').innerText = d.out.qb.toFixed(1);"
// "    document.getElementById('p_rea_c').innerText = d.out.qc.toFixed(1);"
//      // Load
// "    document.getElementById('load_a').innerText = d.out.la.toFixed(1);"
// "    document.getElementById('load_b').innerText = d.out.lb.toFixed(1);"
// "    document.getElementById('load_c').innerText = d.out.lc.toFixed(1);"
//      // Misc
// "    document.getElementById('fan_time').innerText = d.misc.fan;"
// "    if(document.activeElement.id !== 'mid_input') document.getElementById('mid_input').value = d.misc.id;"
// "    document.getElementById('status').innerText = 'Connected. Uptime: ' + d.misc.up + 's';"
// "  }).catch(e => document.getElementById('status').innerText = 'Disconnected');"
// "}"
// "setInterval(update, 1000);"
// "update();"
// "</script></body></html>";

static const char* index_html = 
"<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>UPS Monitor</title>"
"<style>"
"body { font-family: Arial, sans-serif; background-color: #99ccff; margin: 0; padding: 20px; display: flex; justify-content: center; height: 100vh; }"
".container { width: 900px; background-color: white; border: 1px solid #777; height: 85vh; display: flex; flex-direction: column; box-shadow: 2px 2px 10px rgba(0,0,0,0.2); }"
"/* Tabs */"
".tabs { display: flex; background-color: #f0f0f0; border-bottom: 1px solid #aaa; }"
".tab { padding: 8px 20px; border-right: 1px solid #aaa; background-color: #e0e0e0; color: #555; cursor: pointer; font-size: 14px; }"
".tab.active { background-color: white; font-weight: bold; color: black; border-bottom: 1px solid white; margin-bottom: -1px; }"
"/* Content */"
".content { flex: 1; display: flex; padding: 0; overflow-y: auto; }"
".col { width: 50%; border-right: 1px solid #ddd; }"
".row { display: flex; justify-content: space-between; padding: 5px 10px; border-bottom: 1px solid #eee; font-size: 13px; align-items: center; }"
".row:nth-child(odd) { background-color: #fcfcfc; }"
".row:hover { background-color: #eef7ff; }"
".lbl { color: #333; }"
".val { font-weight: bold; color: #000; }"
"</style>"
"</head><body>"
"<div class=\"container\">"
"<div class=\"tabs\">"
"<div class=\"tab active\">STS</div>"
"<div class=\"tab\">ECT</div>"
"<div class=\"tab\">INV</div>"
"</div>"
"<div class=\"content\">"
"<div class=\"col\">"
"<div class=\"row\"><span class=\"lbl\">Сеть на входе:</span><span class=\"val\" id=\"grid_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Сеть байпаса:</span><span class=\"val\" id=\"bypass_grid_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Выпрямитель:</span><span class=\"val\" id=\"rectifier_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Инвертор:</span><span class=\"val\" id=\"inverter_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Питание через инвертор:</span><span class=\"val\" id=\"pwr_via_inverter\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Питание по байпасу:</span><span class=\"val\" id=\"pwr_via_bypass\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Синхронизация:</span><span class=\"val\" id=\"sync_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Режим нагрузки:</span><span class=\"val\" id=\"load_mode\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Звуковой сигнал:</span><span class=\"val\" id=\"sound_alarm\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Состояние АКБ:</span><span class=\"val\" id=\"battery_status\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Режим ИБП:</span><span class=\"val\" id=\"ups_mode\">-</span></div>"
"</div>"
"<div class=\"col\">"
"<div class=\"row\"><span class=\"lbl\">Низкое напряжение на входе:</span><span class=\"val\" id=\"err_low_input_vol\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Высокое напряжение DC шины:</span><span class=\"val\" id=\"err_high_dc_bus\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Низкий заряд АКБ:</span><span class=\"val\" id=\"err_low_bat_charge\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">АКБ не подключены:</span><span class=\"val\" id=\"err_bat_not_conn\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Неисправность инвертора:</span><span class=\"val\" id=\"err_inv_fault\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Перегрузка инвертора:</span><span class=\"val\" id=\"err_inv_overcurrent\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Высокое напряжение на выходе:</span><span class=\"val\" id=\"err_high_out_vol\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Неисправность вентилятора:</span><span class=\"val\" id=\"err_fan_fault\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Требуется замена АКБ:</span><span class=\"val\" id=\"err_replace_bat\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Перегрев выпрямителя:</span><span class=\"val\" id=\"err_rect_overheat\">-</span></div>"
"<div class=\"row\"><span class=\"lbl\">Перегрев инвертора:</span><span class=\"val\" id=\"err_inv_overheat\">-</span></div>"
"</div>"
"</div>"
"</div>"
"<script>"
"let socket;"
"function connectWebSocket() {"
"    socket = new WebSocket('ws://' + window.location.hostname + '/ws');"
"    socket.onmessage = function(event) {"
"        const data = JSON.parse(event.data);"
"        for (const key in data) {"
"            const el = document.getElementById(key);"
"            if (el) el.textContent = data[key];"
"        }"
"    };"
"    socket.onclose = function() {"
"        setTimeout(connectWebSocket, 3000);"
"    };"
"}"
"window.onload = connectWebSocket;"
"</script>"
"</body></html>";


/* --- 3. ФУНКЦИЯ ФОРМИРОВАНИЯ JSON --- */
/* Создает JSON строку из структуры данных. Caller должен освободить память (free). */
// char* generate_ups_json_string(void)
// {
//     // Безопасно берем данные (если структура меняется в прерывании, лучше использовать копию или мьютекс)
//     // Здесь предполагаем, что чтение float атомарно или не критично для отображения
//     FpgaToEspPacket_t *pkt = (FpgaToEspPacket_t*)&ModulData.packet;

//     cJSON *root = cJSON_CreateObject();

//     // --- Группа INPUT ---
//     cJSON *in = cJSON_CreateObject();
//     cJSON_AddNumberToObject(in, "va", pkt->input.v_in_AB);
//     cJSON_AddNumberToObject(in, "vb", pkt->input.v_in_BC);
//     cJSON_AddNumberToObject(in, "vc", pkt->input.v_in_CA);
//     cJSON_AddNumberToObject(in, "ca", pkt->input.i_in_A);
//     cJSON_AddNumberToObject(in, "cb", pkt->input.i_in_B);
//     cJSON_AddNumberToObject(in, "cc", pkt->input.i_in_C);
//     // Частота одна на вход, но в HTML 3 строки, продублируем или выведем разные если есть
//     cJSON_AddNumberToObject(in, "fa", pkt->input.freq_in);
//     cJSON_AddNumberToObject(in, "fb", pkt->input.freq_in); 
//     cJSON_AddNumberToObject(in, "fc", pkt->input.freq_in);
//     cJSON_AddItemToObject(root, "in", in);

//     // --- Группа OUTPUT ---
//     cJSON *out = cJSON_CreateObject();
//     // Активная мощность
//     cJSON_AddNumberToObject(out, "pa", pkt->output.p_active_A);
//     cJSON_AddNumberToObject(out, "pb", pkt->output.p_active_B);
//     cJSON_AddNumberToObject(out, "pc", pkt->output.p_active_C);
    
//     // Реактивная мощность (Q = sqrt(S^2 - P^2))
//     // S = Apparent, P = Active.
//     float qa = sqrtf(fmaxf(0, powf(pkt->output.p_apparent_A, 2) - powf(pkt->output.p_active_A, 2)));
//     float qb = sqrtf(fmaxf(0, powf(pkt->output.p_apparent_B, 2) - powf(pkt->output.p_active_B, 2)));
//     float qc = sqrtf(fmaxf(0, powf(pkt->output.p_apparent_C, 2) - powf(pkt->output.p_active_C, 2)));

//     cJSON_AddNumberToObject(out, "qa", qa);
//     cJSON_AddNumberToObject(out, "qb", qb);
//     cJSON_AddNumberToObject(out, "qc", qc);

//     cJSON_AddNumberToObject(out, "la", pkt->output.load_pct_A);
//     cJSON_AddNumberToObject(out, "lb", pkt->output.load_pct_B);
//     cJSON_AddNumberToObject(out, "lc", pkt->output.load_pct_C);
//     cJSON_AddItemToObject(root, "out", out);

//     // --- Группа MISC ---
//     cJSON *misc = cJSON_CreateObject();
//     // Fan time нет в структуре пакета, берем event_count или 0 как заглушку
//     cJSON_AddNumberToObject(misc, "fan", (int)pkt->output.event_count); 
//     cJSON_AddNumberToObject(misc, "id", 8); // ID модуля, можно вынести в переменную
//     cJSON_AddNumberToObject(misc, "up", esp_timer_get_time() / 1000000);
//     cJSON_AddItemToObject(root, "misc", misc);

//     // Печать в строку
//     char *json_str = cJSON_PrintUnformatted(root);
//     cJSON_Delete(root); // Удаляем объект JSON из памяти

//     return json_str;
// }
char* generate_ups_json_string(void)
{
    // Безопасно берем данные (если структура меняется в прерывании, лучше использовать копию или мьютекс)
    FpgaToEspPacket_t *pkt = (FpgaToEspPacket_t*)&ModulData.packet;

    cJSON *root = cJSON_CreateObject();

    // --- Статусы (GroupStatus_t) ---
    cJSON_AddNumberToObject(root, "grid_status", pkt->status.grid_status);
    cJSON_AddNumberToObject(root, "bypass_grid_status", pkt->status.bypass_grid_status);
    cJSON_AddNumberToObject(root, "rectifier_status", pkt->status.rectifier_status);
    cJSON_AddNumberToObject(root, "inverter_status", pkt->status.inverter_status);
    cJSON_AddNumberToObject(root, "pwr_via_inverter", pkt->status.pwr_via_inverter);
    cJSON_AddNumberToObject(root, "pwr_via_bypass", pkt->status.pwr_via_bypass);
    cJSON_AddNumberToObject(root, "sync_status", pkt->status.sync_status);
    cJSON_AddNumberToObject(root, "load_mode", pkt->status.load_mode);
    cJSON_AddNumberToObject(root, "sound_alarm", pkt->status.sound_alarm);
    cJSON_AddNumberToObject(root, "battery_status", pkt->status.battery_status);
    cJSON_AddNumberToObject(root, "ups_mode", pkt->status.ups_mode);

    // --- Аварии (GroupAlarms_t) ---
    cJSON_AddNumberToObject(root, "err_low_input_vol", pkt->alarms.err_low_input_vol);
    cJSON_AddNumberToObject(root, "err_high_dc_bus", pkt->alarms.err_high_dc_bus);
    cJSON_AddNumberToObject(root, "err_low_bat_charge", pkt->alarms.err_low_bat_charge);
    cJSON_AddNumberToObject(root, "err_bat_not_conn", pkt->alarms.err_bat_not_conn);
    cJSON_AddNumberToObject(root, "err_inv_fault", pkt->alarms.err_inv_fault);
    cJSON_AddNumberToObject(root, "err_inv_overcurrent", pkt->alarms.err_inv_overcurrent);
    cJSON_AddNumberToObject(root, "err_high_out_vol", pkt->alarms.err_high_out_vol);
    cJSON_AddNumberToObject(root, "err_fan_fault", pkt->alarms.err_fan_fault);
    cJSON_AddNumberToObject(root, "err_replace_bat", pkt->alarms.err_replace_bat);
    cJSON_AddNumberToObject(root, "err_rect_overheat", pkt->alarms.err_rect_overheat);
    cJSON_AddNumberToObject(root, "err_inv_overheat", pkt->alarms.err_inv_overheat);

    // --- Входные параметры (GroupInput_t) ---
    cJSON_AddNumberToObject(root, "v_in_AB", pkt->input.v_in_AB);
    cJSON_AddNumberToObject(root, "v_in_BC", pkt->input.v_in_BC);
    cJSON_AddNumberToObject(root, "v_in_CA", pkt->input.v_in_CA);
    cJSON_AddNumberToObject(root, "v_bypass_A", pkt->input.v_bypass_A);
    cJSON_AddNumberToObject(root, "v_bypass_B", pkt->input.v_bypass_B);
    cJSON_AddNumberToObject(root, "v_bypass_C", pkt->input.v_bypass_C);
    cJSON_AddNumberToObject(root, "i_in_A", pkt->input.i_in_A);
    cJSON_AddNumberToObject(root, "i_in_B", pkt->input.i_in_B);
    cJSON_AddNumberToObject(root, "i_in_C", pkt->input.i_in_C);
    cJSON_AddNumberToObject(root, "freq_in", pkt->input.freq_in);

    // --- Выходные параметры (GroupOutput_t) ---
    cJSON_AddNumberToObject(root, "v_out_A", pkt->output.v_out_A);
    cJSON_AddNumberToObject(root, "v_out_B", pkt->output.v_out_B);
    cJSON_AddNumberToObject(root, "v_out_C", pkt->output.v_out_C);
    cJSON_AddNumberToObject(root, "freq_out", pkt->output.freq_out);
    cJSON_AddNumberToObject(root, "i_out_A", pkt->output.i_out_A);
    cJSON_AddNumberToObject(root, "i_out_B", pkt->output.i_out_B);
    cJSON_AddNumberToObject(root, "i_out_C", pkt->output.i_out_C);
    cJSON_AddNumberToObject(root, "p_active_A", pkt->output.p_active_A);
    cJSON_AddNumberToObject(root, "p_active_B", pkt->output.p_active_B);
    cJSON_AddNumberToObject(root, "p_active_C", pkt->output.p_active_C);
    cJSON_AddNumberToObject(root, "p_apparent_A", pkt->output.p_apparent_A);
    cJSON_AddNumberToObject(root, "p_apparent_B", pkt->output.p_apparent_B);
    cJSON_AddNumberToObject(root, "p_apparent_C", pkt->output.p_apparent_C);
    cJSON_AddNumberToObject(root, "load_pct_A", pkt->output.load_pct_A);
    cJSON_AddNumberToObject(root, "load_pct_B", pkt->output.load_pct_B);
    cJSON_AddNumberToObject(root, "load_pct_C", pkt->output.load_pct_C);
    cJSON_AddNumberToObject(root, "event_count", pkt->output.event_count);

    // --- Параметры АКБ (GroupBattery_t) ---
    cJSON_AddNumberToObject(root, "bat_voltage", pkt->battery.bat_voltage);
    cJSON_AddNumberToObject(root, "bat_capacity", pkt->battery.bat_capacity);
    cJSON_AddNumberToObject(root, "bat_groups_count", pkt->battery.bat_groups_count);
    cJSON_AddNumberToObject(root, "dc_bus_voltage", pkt->battery.dc_bus_voltage);
    cJSON_AddNumberToObject(root, "bat_current", pkt->battery.bat_current);
    cJSON_AddNumberToObject(root, "backup_time", pkt->battery.backup_time);

    // Печать в строку
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); // Удаляем объект JSON из памяти

    return json_str;
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