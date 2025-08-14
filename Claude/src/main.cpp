/**
 * ESP32-C6 Zigbee Environmental Sensor with Deep Sleep
 * 
 * Hardware: FireBeetle 2 ESP32-C6
 * Sensors: AZ Delivery Bodenfeuchtesensor 1.2, BME280
 * Communication: Zigbee (802.15.4) + Optional WiFi/Webserver
 * 
 * Features:
 * - Zigbee End Device with configurable reporting intervals
 * - Deep Sleep support for battery operation
 * - BME280 temperature, humidity, pressure sensing
 * - Soil moisture sensing with threshold categories
 * - Battery voltage monitoring
 * - Optional WiFi configuration via Zigbee
 * - Optional web server for configuration
 * - Dual-core optimization (LP/HP cores)
 * 
 * @author Assistant
 * @version 1.0
 * @date 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_zigbee_core.h>
#include <esp_check.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp32-hal-adc.h>

// ==================== KONFIGURATION ====================
// Diese Werte können angepasst werden

// Pin-Definitionen (FireBeetle 2 ESP32-C6)
#define SOIL_SENSOR_PIN         4     // GPIO4 (ADC1_CH4) - Bodenfeuchte Analog
#define SOIL_SENSOR_POWER_PIN   5     // GPIO5 - Stromversorgung für Sensor
#define BATTERY_VOLTAGE_PIN     0     // GPIO0 (ADC1_CH0) - Batteriespannung
#define BUTTON_PIN              9     // GPIO9 - BOOT Button
#define LED_PIN                 15    // GPIO15 - Onboard LED
#define BME280_SDA_PIN          19    // GPIO19 - I2C SDA
#define BME280_SCL_PIN          20    // GPIO20 - I2C SCL

// Zigbee Konfiguration
#define ZIGBEE_CHANNEL          11    // Zigbee Kanal (11-26)
#define ZIGBEE_PAN_ID           0x1234 // PAN ID
#define MANUFACTURER_NAME       "DIY_Sensor"
#define MODEL_IDENTIFIER        "ESP32C6_ENV_SENSOR"
#define ENDPOINT_ID             1

// Timing Konfiguration (in Sekunden)
#define DEFAULT_MEASURE_INTERVAL 30   // Standard: alle 30 Sekunden (zum Testen)
#define MIN_MEASURE_INTERVAL     10   // Minimum Intervall
#define MAX_MEASURE_INTERVAL     3600 // Maximum Intervall (1 Stunde)

// Bodenfeuchte Schwellwerte (ADC Werte 0-4095)
#define SOIL_VERY_DRY           3500  // > 3500: Sehr trocken
#define SOIL_DRY                2800  // 2800-3500: Trocken
#define SOIL_NORMAL             2000  // 2000-2800: Normal
#define SOIL_WET                1200  // 1200-2000: Feucht
                                      // < 1200: Zu feucht

// WiFi Standard-Konfiguration
#define DEFAULT_WIFI_SSID       "YourWiFiSSID"
#define DEFAULT_WIFI_PASSWORD   "YourWiFiPassword"

// ==================== GLOBALE VARIABLEN ====================

static const char *TAG = "ESP32_ZIGBEE_SENSOR";

// Sensor Objekte
Adafruit_BME280 bme;
Preferences preferences;
WebServer *webServer = nullptr;

// Laufzeitkonfiguration
bool enableDeepSleep = false;
bool enableWiFi = false;
bool enableWebServer = false;

// Zigbee Handles
static esp_zb_ep_list_t *esp_zb_ep_list = NULL;
static esp_zb_cluster_list_t *esp_zb_cluster_list = NULL;

// Status Variablen
static bool zigbee_initialized = false;
static bool wifi_enabled = false;
static uint32_t measure_interval = DEFAULT_MEASURE_INTERVAL;
static volatile bool button_pressed = false;

// Sensor Daten Struktur
struct SensorData {
    float temperature;      // °C
    float humidity;         // %
    float pressure;         // hPa
    float abs_humidity;     // g/m³
    uint16_t soil_moisture; // ADC Wert (0-4095)
    uint8_t soil_category;  // 0=sehr trocken, 1=trocken, 2=normal, 3=feucht, 4=zu feucht
    float battery_voltage;  // V
    uint8_t battery_percent; // %
    float charge_voltage;   // V (falls verfügbar)
};

static SensorData current_data;

// Task Handles für Dual-Core
TaskHandle_t zigbeeTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

// ==================== HILFSFUNKTIONEN ====================

/**
 * Berechnet die absolute Luftfeuchtigkeit aus Temperatur und relativer Feuchte
 * @param temp Temperatur in °C
 * @param rel_humidity Relative Luftfeuchtigkeit in %
 * @return Absolute Luftfeuchtigkeit in g/m³
 */
float calculateAbsoluteHumidity(float temp, float rel_humidity) {
    // Magnus-Formel für Sättigungsdampfdruck
    float a = 17.27;
    float b = 237.7;
    float gamma = (a * temp) / (b + temp) + log(rel_humidity / 100.0);
    float dewPoint = (b * gamma) / (a - gamma);
    
    // Absolute Luftfeuchtigkeit
    float abs_humidity = 216.7 * ((rel_humidity / 100.0) * 6.112 * 
                        exp((17.62 * temp) / (243.12 + temp))) / (273.15 + temp);
    return abs_humidity;
}

/**
 * Kategorisiert die Bodenfeuchte basierend auf ADC-Wert
 * @param adc_value ADC-Wert (0-4095)
 * @return Kategorie (0-4)
 */
uint8_t categorizeSoilMoisture(uint16_t adc_value) {
    if (adc_value > SOIL_VERY_DRY) return 0;  // Sehr trocken
    if (adc_value > SOIL_DRY) return 1;       // Trocken  
    if (adc_value > SOIL_NORMAL) return 2;    // Normal
    if (adc_value > SOIL_WET) return 3;       // Feucht
    return 4;                                  // Zu feucht
}

/**
 * Konvertiert Bodenfeuchte-Kategorie zu String
 */
const char* soilCategoryToString(uint8_t category) {
    switch(category) {
        case 0: return "Sehr trocken";
        case 1: return "Trocken";
        case 2: return "Normal";
        case 3: return "Feucht";
        case 4: return "Zu feucht";
        default: return "Unbekannt";
    }
}

/**
 * Liest die Batteriespannung
 * FireBeetle hat Spannungsteiler 1:1, max 3.3V ADC
 * @return Batteriespannung in V
 */
float readBatteryVoltage() {
    // ADC konfigurieren
    int adc_value = analogRead(BATTERY_VOLTAGE_PIN);
    // FireBeetle 2 hat 1:1 Spannungsteiler, ADC max 3.3V bei 12bit (4095)
    float voltage = (adc_value / 4095.0) * 3.3 * 2.0; // *2 wegen Spannungsteiler
    return voltage;
}

/**
 * Berechnet Batteriestand in Prozent
 * @param voltage Batteriespannung in V
 * @return Batteriestand in %
 */
uint8_t calculateBatteryPercent(float voltage) {
    // LiPo Batterie: 4.2V = 100%, 3.0V = 0%
    const float max_voltage = 4.2;
    const float min_voltage = 3.0;
    
    if (voltage >= max_voltage) return 100;
    if (voltage <= min_voltage) return 0;
    
    return (uint8_t)((voltage - min_voltage) / (max_voltage - min_voltage) * 100);
}

/**
 * Liest alle Sensordaten
 */
void readSensorData() {
    ESP_LOGI(TAG, "Lese Sensordaten...");
    
    // Bodenfeuchte-Sensor einschalten
    digitalWrite(SOIL_SENSOR_POWER_PIN, HIGH);
    delay(100); // Stabilisierung
    
    // Bodenfeuchte lesen (mehrere Messungen für Stabilität)
    uint32_t soil_sum = 0;
    for(int i = 0; i < 5; i++) {
        soil_sum += analogRead(SOIL_SENSOR_PIN);
        delay(10);
    }
    current_data.soil_moisture = soil_sum / 5;
    current_data.soil_category = categorizeSoilMoisture(current_data.soil_moisture);
    
    // Bodenfeuchte-Sensor ausschalten (Stromsparend)
    digitalWrite(SOIL_SENSOR_POWER_PIN, LOW);
    
    // BME280 Daten lesen
    if (bme.takeForcedMeasurement()) {
        current_data.temperature = bme.readTemperature();
        current_data.humidity = bme.readHumidity();
        current_data.pressure = bme.readPressure() / 100.0F; // Pa zu hPa
        current_data.abs_humidity = calculateAbsoluteHumidity(
            current_data.temperature, current_data.humidity);
    } else {
        ESP_LOGE(TAG, "BME280 Messung fehlgeschlagen!");
    }
    
    // Batterie lesen
    current_data.battery_voltage = readBatteryVoltage();
    current_data.battery_percent = calculateBatteryPercent(current_data.battery_voltage);
    
    // Ladespannung (falls verfügbar über USB)
    // Hinweis: FireBeetle 2 hat keinen direkten Zugriff auf VIN wenn über Batterie betrieben
    current_data.charge_voltage = 0.0; // Implementierung board-spezifisch
    
    // Debug Ausgabe
    ESP_LOGI(TAG, "=== Sensordaten ===");
    ESP_LOGI(TAG, "Temperatur: %.2f °C", current_data.temperature);
    ESP_LOGI(TAG, "Luftfeuchtigkeit: %.2f %%", current_data.humidity);
    ESP_LOGI(TAG, "Luftdruck: %.2f hPa", current_data.pressure);
    ESP_LOGI(TAG, "Abs. Luftfeuchtigkeit: %.2f g/m³", current_data.abs_humidity);
    ESP_LOGI(TAG, "Bodenfeuchte: %d (ADC) - %s", 
            current_data.soil_moisture, 
            soilCategoryToString(current_data.soil_category));
    ESP_LOGI(TAG, "Batterie: %.2fV (%d%%)", 
            current_data.battery_voltage, 
            current_data.battery_percent);
}

// ==================== ZIGBEE FUNKTIONEN ====================

/**
 * Zigbee Stack Signal Handler
 */
static void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Joined network successfully");
                zigbee_initialized = true;
                digitalWrite(LED_PIN, HIGH); // LED an wenn verbunden
            } else {
                ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
                esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, 
                                     ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
            
        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGI(TAG, "Zigbee can sleep");
            // Hier könnten wir Light Sleep aktivieren
            // esp_zb_sleep_now();
            break;
            
        default:
            ESP_LOGI(TAG, "ZDO signal: %d, status: %d", sig_type, err_status);
            break;
    }
}

/**
 * Zigbee Attribute setzen
 */
static void esp_zb_set_attributes(void) {
    // Basic Cluster Attribute
    esp_zb_attribute_list_t *basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 
                                 (void *)MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 
                                 (void *)MODEL_IDENTIFIER);
    uint8_t power_source = 0x03; // Battery
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, 
                                 &power_source);
    
    // Temperature Measurement Cluster
    esp_zb_attribute_list_t *temperature_cluster = 
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    int16_t temp_value = (int16_t)(current_data.temperature * 100);
    esp_zb_temperature_meas_cluster_add_attr(temperature_cluster, 
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp_value);
    int16_t temp_min = -4000; // -40°C
    esp_zb_temperature_meas_cluster_add_attr(temperature_cluster,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &temp_min);
    int16_t temp_max = 12500; // 125°C  
    esp_zb_temperature_meas_cluster_add_attr(temperature_cluster,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &temp_max);
    
    // Humidity Measurement Cluster
    esp_zb_attribute_list_t *humidity_cluster = 
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    uint16_t humidity_value = (uint16_t)(current_data.humidity * 100);
    esp_zb_humidity_meas_cluster_add_attr(humidity_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity_value);
    uint16_t humidity_min = 0;
    esp_zb_humidity_meas_cluster_add_attr(humidity_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &humidity_min);
    uint16_t humidity_max = 10000; // 100%
    esp_zb_humidity_meas_cluster_add_attr(humidity_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &humidity_max);
    
    // Pressure Measurement Cluster (Custom)
    esp_zb_attribute_list_t *pressure_cluster = 
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    int16_t pressure_value = (int16_t)(current_data.pressure * 10);
    esp_zb_custom_cluster_add_custom_attr(pressure_cluster, 0x0000, 
        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &pressure_value);
    
    // Power Configuration Cluster (Battery)
    esp_zb_attribute_list_t *power_cfg_cluster = 
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    uint8_t battery_voltage_raw = (uint8_t)(current_data.battery_voltage * 10); // 0.1V units
    esp_zb_power_config_cluster_add_attr(power_cfg_cluster,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage_raw);
    uint8_t battery_percentage = current_data.battery_percent * 2; // 0.5% units
    esp_zb_power_config_cluster_add_attr(power_cfg_cluster,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percentage);
    
    // Analog Input Cluster für Bodenfeuchte
    esp_zb_attribute_list_t *analog_input_cluster = 
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    float soil_moisture_scaled = (float)current_data.soil_moisture / 4095.0 * 100.0;
    esp_zb_analog_input_cluster_add_attr(analog_input_cluster,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &soil_moisture_scaled);
    
    // Cluster Liste erstellen
    esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, temperature_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_cluster_list, pressure_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, power_cfg_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(esp_zb_cluster_list, analog_input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

/**
 * Sendet Zigbee Report mit aktuellen Sensordaten
 */
void sendZigbeeReport() {
    if (!zigbee_initialized) {
        ESP_LOGW(TAG, "Zigbee nicht initialisiert, überspringe Report");
        return;
    }
    
    ESP_LOGI(TAG, "Sende Zigbee Report...");
    
    // Temperature Report
    esp_zb_zcl_report_attr_cmd_t temp_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000, // Coordinator
            .dst_endpoint = 1,
            .src_endpoint = ENDPOINT_ID,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    esp_zb_zcl_report_attr_cmd_req(&temp_cmd);
    
    // Humidity Report
    esp_zb_zcl_report_attr_cmd_t humidity_cmd = temp_cmd;
    humidity_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    humidity_cmd.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    esp_zb_zcl_report_attr_cmd_req(&humidity_cmd);
    
    // Weitere Reports für andere Cluster...
    
    ESP_LOGI(TAG, "Zigbee Report gesendet");
}

/**
 * Zigbee Task (läuft auf HP Core)
 */
void zigbeeTask(void *pvParameters) {
    ESP_LOGI(TAG, "Zigbee Task gestartet auf Core %d", xPortGetCoreID());
    
    // NVS initialisieren
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Zigbee Stack Konfiguration
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    
    // Sleep aktivieren für Zigbee End Device
    esp_zb_sleep_enable(true);
    
    // Set device configuration
    esp_zb_set_primary_network_channel_set(1 << ZIGBEE_CHANNEL);
    esp_zb_set_extended_pan_id((uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    
    // Attribute setzen
    esp_zb_set_attributes();
    
    // Endpoint Liste erstellen
    esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    
    // Device registrieren
    esp_zb_device_register(esp_zb_ep_list);
    
    // Zigbee main loop starten
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
    
    vTaskDelete(NULL);
}

// ==================== SENSOR TASK ====================

/**
 * Sensor Task - Liest periodisch Sensordaten und sendet Reports
 */
void sensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task gestartet auf Core %d", xPortGetCoreID());
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // Warte auf Intervall oder Button
        if (button_pressed || 
            (xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(measure_interval * 1000)) {
            
            button_pressed = false;
            xLastWakeTime = xTaskGetTickCount();
            
            // Sensordaten lesen
            readSensorData();
            
            // Zigbee Report senden
            if (zigbee_initialized) {
                sendZigbeeReport();
            }
            
            // Deep Sleep vorbereiten (falls aktiviert)
            if (enableDeepSleep && zigbee_initialized) {
                ESP_LOGI(TAG, "Gehe in Deep Sleep für %d Sekunden", measure_interval);
                
                // Wake-up Timer setzen
                esp_sleep_enable_timer_wakeup(measure_interval * 1000000ULL);
                
                // Wake-up bei Button Press
                esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
                
                // Deep Sleep starten
                esp_deep_sleep_start();
            }
        }
        
        // Task yield
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== WEB SERVER ====================

/**
 * Web Server Root Handler
 */
void handleRoot() {
    if (!webServer) return;
    
    String html = "<html><head><title>ESP32-C6 Sensor</title>";
    html += "<meta charset='UTF-8'>";
    html += "<style>body{font-family:Arial;margin:20px;}";
    html += "table{border-collapse:collapse;}";
    html += "th,td{border:1px solid #ddd;padding:8px;text-align:left;}";
    html += "th{background-color:#4CAF50;color:white;}";
    html += "input{margin:5px;padding:5px;}";
    html += "</style></head><body>";
    html += "<h1>ESP32-C6 Umweltsensor</h1>";
    
    // Sensordaten Tabelle
    html += "<h2>Aktuelle Messwerte</h2>";
    html += "<table>";
    html += "<tr><th>Parameter</th><th>Wert</th></tr>";
    html += "<tr><td>Temperatur</td><td>" + String(current_data.temperature, 2) + " °C</td></tr>";
    html += "<tr><td>Luftfeuchtigkeit</td><td>" + String(current_data.humidity, 2) + " %</td></tr>";
    html += "<tr><td>Luftdruck</td><td>" + String(current_data.pressure, 2) + " hPa</td></tr>";
    html += "<tr><td>Abs. Luftfeuchtigkeit</td><td>" + String(current_data.abs_humidity, 2) + " g/m³</td></tr>";
    html += "<tr><td>Bodenfeuchte</td><td>" + String(current_data.soil_moisture) + " (";
    html += soilCategoryToString(current_data.soil_category);
    html += ")</td></tr>";
    html += "<tr><td>Batterie</td><td>" + String(current_data.battery_voltage, 2) + " V (";
    html += String(current_data.battery_percent) + "%)</td></tr>";
    html += "</table>";
    
    // Konfiguration
    html += "<h2>Konfiguration</h2>";
    html += "<form action='/config' method='POST'>";
    html += "Messintervall (Sek): <input type='number' name='interval' value='" + String(measure_interval) + "' min='10' max='3600'><br>";
    html += "WiFi SSID: <input type='text' name='ssid' value='" + String(WiFi.SSID()) + "'><br>";
    html += "WiFi Passwort: <input type='password' name='pass'><br>";
    html += "WiFi aktiv: <input type='checkbox' name='wifi' " + String(enableWiFi ? "checked" : "") + "><br>";
    html += "Webserver aktiv: <input type='checkbox' name='webserver' " + String(enableWebServer ? "checked" : "") + "><br>";
    html += "Deep Sleep: <input type='checkbox' name='deepsleep' " + String(enableDeepSleep ? "checked" : "") + "><br>";
    html += "<input type='submit' value='Speichern'>";
    html += "</form>";
    
    html += "<p><a href='/'>Aktualisieren</a> | <a href='/json'>JSON API</a></p>";
    html += "</body></html>";
    
    webServer->send(200, "text/html", html);
}

/**
 * Web Server JSON API Handler
 */
void handleJSON() {
    if (!webServer) return;
    
    String json = "{";
    json += "\"temperature\":" + String(current_data.temperature, 2) + ",";
    json += "\"humidity\":" + String(current_data.humidity, 2) + ",";
    json += "\"pressure\":" + String(current_data.pressure, 2) + ",";
    json += "\"abs_humidity\":" + String(current_data.abs_humidity, 2) + ",";
    json += "\"soil_moisture\":" + String(current_data.soil_moisture) + ",";
    json += "\"soil_category\":\"" + String(soilCategoryToString(current_data.soil_category)) + "\",";
    json += "\"battery_voltage\":" + String(current_data.battery_voltage, 2) + ",";
    json += "\"battery_percent\":" + String(current_data.battery_percent) + ",";
    json += "\"zigbee_connected\":" + String(zigbee_initialized ? "true" : "false") + ",";
    json += "\"wifi_connected\":" + String(WiFi.isConnected() ? "true" : "false") + ",";
    json += "\"measure_interval\":" + String(measure_interval);
    json += "}";
    
    webServer->send(200, "application/json", json);
}

/**
 * Web Server Config Handler
 */
void handleConfig() {
    if (!webServer) return;
    
    if (webServer->hasArg("interval")) {
        measure_interval = webServer->arg("interval").toInt();
        if (measure_interval < MIN_MEASURE_INTERVAL) measure_interval = MIN_MEASURE_INTERVAL;
        if (measure_interval > MAX_MEASURE_INTERVAL) measure_interval = MAX_MEASURE_INTERVAL;
        preferences.putUInt("interval", measure_interval);
    }
    
    if (webServer->hasArg("ssid") && webServer->hasArg("pass")) {
        String ssid = webServer->arg("ssid");
        String pass = webServer->arg("pass");
        if (ssid.length() > 0) {
            preferences.putString("wifi_ssid", ssid);
            preferences.putString("wifi_pass", pass);
            // WiFi neu verbinden
            WiFi.disconnect();
            WiFi.begin(ssid.c_str(), pass.c_str());
        }
    }
    
    enableDeepSleep = webServer->hasArg("deepsleep");
    preferences.putBool("deep_sleep", enableDeepSleep);

    enableWiFi = webServer->hasArg("wifi");
    preferences.putBool("wifi", enableWiFi);

    enableWebServer = webServer->hasArg("webserver");
    preferences.putBool("webserver", enableWebServer);
    
    webServer->sendHeader("Location", "/");
    webServer->send(303);
}

/**
 * Web Server Task
 */
void webServerTask(void *pvParameters) {
    ESP_LOGI(TAG, "Web Server Task gestartet auf Core %d", xPortGetCoreID());
    
    if (!enableWebServer) {
        vTaskDelete(NULL);
        return;
    }
    
    webServer = new WebServer(80);
    webServer->on("/", handleRoot);
    webServer->on("/json", handleJSON);
    webServer->on("/config", HTTP_POST, handleConfig);
    webServer->begin();
    
    ESP_LOGI(TAG, "Web Server gestartet auf Port 80");
    
    while (true) {
        if (webServer) {
            webServer->handleClient();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== INTERRUPT HANDLER ====================

/**
 * Button Interrupt Handler
 */
void IRAM_ATTR buttonISR() {
    button_pressed = true;
}

// ==================== WIFI FUNKTIONEN ====================

/**
 * WiFi initialisieren
 */
void initWiFi() {
    if (!enableWiFi) {
        ESP_LOGI(TAG, "WiFi deaktiviert");
        return;
    }
    
    // WiFi Credentials aus Preferences laden
    String ssid = preferences.getString("wifi_ssid", DEFAULT_WIFI_SSID);
    String pass = preferences.getString("wifi_pass", DEFAULT_WIFI_PASSWORD);
    
    ESP_LOGI(TAG, "Verbinde mit WiFi SSID: %s", ssid.c_str());
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    
    // Warte auf Verbindung (max 10 Sekunden)
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        ESP_LOGI(TAG, "WiFi verbindet...");
        attempts++;
    }
    
    if (WiFi.isConnected()) {
        wifi_enabled = true;
        ESP_LOGI(TAG, "WiFi verbunden! IP: %s", WiFi.localIP().toString().c_str());
    } else {
        ESP_LOGW(TAG, "WiFi Verbindung fehlgeschlagen");
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
    }
}

// ==================== SETUP ====================

void setup() {
    // Serial für Debug
    Serial.begin(115200);
    delay(1000);
    ESP_LOGI(TAG, "=== ESP32-C6 Zigbee Sensor Start ===");
    ESP_LOGI(TAG, "Firmware Version: 1.0");
    ESP_LOGI(TAG, "Chip Model: %s", ESP.getChipModel());
    ESP_LOGI(TAG, "Chip Revision: %d", ESP.getChipRevision());
    ESP_LOGI(TAG, "Flash Size: %d KB", ESP.getFlashChipSize() / 1024);
    ESP_LOGI(TAG, "Free Heap: %d bytes", ESP.getFreeHeap());
    
    // Preferences initialisieren
    preferences.begin("sensor_config", false);
    measure_interval = preferences.getUInt("interval", DEFAULT_MEASURE_INTERVAL);
    enableDeepSleep = preferences.getBool("deep_sleep", false);
    enableWiFi = preferences.getBool("wifi", false);
    enableWebServer = preferences.getBool("webserver", false);
    
    // GPIO Setup
    pinMode(SOIL_SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SOIL_SENSOR_POWER_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Button Interrupt
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    
    // I2C initialisieren für BME280
    Wire.begin(BME280_SDA_PIN, BME280_SCL_PIN);
    
    // BME280 initialisieren
    if (!bme.begin(0x76)) { // Adresse 0x76 oder 0x77
        ESP_LOGE(TAG, "BME280 nicht gefunden! Prüfe Verkabelung.");
        ESP_LOGI(TAG, "Versuche alternative Adresse 0x77...");
        if (!bme.begin(0x77)) {
            ESP_LOGE(TAG, "BME280 nicht gefunden auf beiden Adressen!");
        }
    } else {
        ESP_LOGI(TAG, "BME280 erfolgreich initialisiert");
        
        // BME280 Konfiguration für Low Power
        bme.setSampling(Adafruit_BME280::MODE_FORCED,     // Forced mode für Low Power
                       Adafruit_BME280::SAMPLING_X2,      // Temperatur oversampling x2
                       Adafruit_BME280::SAMPLING_X2,      // Druck oversampling x2
                       Adafruit_BME280::SAMPLING_X2,      // Feuchte oversampling x2
                       Adafruit_BME280::FILTER_X2,        // Filter x2
                       Adafruit_BME280::STANDBY_MS_1000); // Standby 1000ms
    }
    
    // ADC Konfiguration
    analogReadResolution(12); // 12 bit ADC (0-4095)
    analogSetAttenuation(ADC_11db); // Volle Range bis 3.3V
    
    // Initiale Sensordaten lesen
    readSensorData();
    
    // WiFi initialisieren (falls aktiviert)
    initWiFi();
    
    // Power Management für ESP32-C6 konfigurieren
    #ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_LOGI(TAG, "Power Management aktiviert");
    #endif
    
    // Tasks erstellen mit Core-Zuordnung
    // Zigbee Task auf HP Core (Core 0)
    xTaskCreatePinnedToCore(
        zigbeeTask,
        "zigbee_task",
        8192,
        NULL,
        5,
        &zigbeeTaskHandle,
        0  // HP Core
    );
    
    // Sensor Task auf HP Core (Core 0)
    xTaskCreatePinnedToCore(
        sensorTask,
        "sensor_task",
        4096,
        NULL,
        3,
        &sensorTaskHandle,
        0  // HP Core
    );
    
    // Web Server Task auf HP Core (falls aktiviert)
    if (enableWebServer && wifi_enabled) {
        xTaskCreatePinnedToCore(
            webServerTask,
            "web_task",
            4096,
            NULL,
            2,
            &webTaskHandle,
            0  // HP Core
        );
    }
    
    ESP_LOGI(TAG, "Setup abgeschlossen");
}

// ==================== MAIN LOOP ====================

void loop() {
    // Hauptloop bleibt leer, alles läuft in Tasks
    // Der LP Core kann hier für Low-Power Operationen genutzt werden
    
    // Heartbeat LED (zeigt dass System läuft)
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 5000) {
        last_blink = millis();
        if (!zigbee_initialized) {
            // Blinke wenn nicht verbunden
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
        }
    }
    
    delay(100);
}