/**
 * Zigbee2MQTT External Converter for ESP32-C6 Environmental Sensor
 * 
 * Place this file in your zigbee2mqtt data folder under 'converters/'
 * Add to configuration.yaml:
 * external_converters:
 *   - esp32c6_sensor.js
 * 
 * @version 1.0
 * @compatible zigbee2mqtt v2.6.0+
 */

const {
    temperature,
    humidity,
    pressure,
    battery,
    identify,
    customCluster
} = require('zigbee-herdsman-converters/lib/modernExtend');

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

// Custom converter für Bodenfeuchte
const fzLocal = {
    soil_moisture: {
        cluster: 'genAnalogInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('presentValue')) {
                const raw_value = msg.data['presentValue'];
                // Konvertiere zu Prozent (0-100%)
                result.soil_moisture = parseFloat(raw_value.toFixed(2));
                
                // Kategorisierung
                let category = 'Unbekannt';
                if (raw_value > 85) category = 'Sehr trocken';
                else if (raw_value > 68) category = 'Trocken';
                else if (raw_value > 49) category = 'Normal';
                else if (raw_value > 29) category = 'Feucht';
                else category = 'Zu feucht';
                
                result.soil_category = category;
            }
            return result;
        },
    },
    
    abs_humidity: {
        cluster: 'msRelativeHumidity',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            // Berechne absolute Luftfeuchtigkeit wenn Temperatur und rel. Feuchte vorhanden
            if (meta.state && meta.state.temperature !== undefined && 
                msg.data.hasOwnProperty('measuredValue')) {
                const temp = meta.state.temperature;
                const relHumidity = msg.data['measuredValue'] / 100;
                
                // Magnus-Formel
                const a = 17.27;
                const b = 237.7;
                const gamma = (a * temp) / (b + temp) + Math.log(relHumidity / 100.0);
                
                // Absolute Luftfeuchtigkeit in g/m³
                const absHumidity = 216.7 * ((relHumidity / 100.0) * 6.112 * 
                    Math.exp((17.62 * temp) / (243.12 + temp))) / (273.15 + temp);
                
                return {abs_humidity: parseFloat(absHumidity.toFixed(2))};
            }
            return {};
        },
    },
    
    battery_ext: {
        cluster: 'genPowerCfg',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            if (msg.data.hasOwnProperty('batteryVoltage')) {
                // Batteriespannung in 100mV units
                result.battery_voltage = msg.data['batteryVoltage'] / 10.0;
            }
            if (msg.data.hasOwnProperty('batteryPercentageRemaining')) {
                // Batterie Prozent in 0.5% units
                result.battery = msg.data['batteryPercentageRemaining'] / 2.0;
            }
            return result;
        },
    },
};

// Custom toZigbee converter für Konfiguration
const tzLocal = {
    measure_interval: {
        key: ['measure_interval'],
        convertSet: async (entity, key, value, meta) => {
            // Sende Konfiguration an Device
            const payload = {
                0x0201: {  // Custom Attribute ID für Intervall
                    value: value,
                    type: 0x21  // uint16
                }
            };
            await entity.write('genBasic', payload);
            return {state: {measure_interval: value}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read('genBasic', [0x0201]);
        },
    },
    
    wifi_config: {
        key: ['wifi_ssid', 'wifi_password'],
        convertSet: async (entity, key, value, meta) => {
            // Sende WiFi Konfiguration
            if (key === 'wifi_ssid') {
                const payload = {
                    0x0202: {  // Custom Attribute ID für SSID
                        value: value,
                        type: 0x42  // string
                    }
                };
                await entity.write('genBasic', payload);
                return {state: {wifi_ssid: value}};
            } else if (key === 'wifi_password') {
                const payload = {
                    0x0203: {  // Custom Attribute ID für Password
                        value: value,
                        type: 0x42  // string
                    }
                };
                await entity.write('genBasic', payload);
                return {state: {wifi_password: '***'}};  // Zeige Passwort nicht an
            }
        },
    },
    
    webserver_control: {
        key: ['webserver_enabled'],
        convertSet: async (entity, key, value, meta) => {
            const payload = {
                0x0204: {  // Custom Attribute ID für Webserver
                    value: value ? 1 : 0,
                    type: 0x10  // boolean
                }
            };
            await entity.write('genBasic', payload);
            return {state: {webserver_enabled: value}};
        },
    },
};

// Device Definition
const definition = {
    zigbeeModel: ['ESP32C6_ENV_SENSOR'],
    model: 'ESP32C6_ENV_SENSOR',
    vendor: 'DIY_Sensor',
    description: 'ESP32-C6 Environmental Sensor with Soil Moisture',
    
    // From Zigbee Converters
    fromZigbee: [
        fz.temperature,
        fz.humidity,
        fz.pressure,
        fzLocal.soil_moisture,
        fzLocal.abs_humidity,
        fzLocal.battery_ext,
        fz.battery,
    ],
    
    // To Zigbee Converters
    toZigbee: [
        tzLocal.measure_interval,
        tzLocal.wifi_config,
        tzLocal.webserver_control,
    ],
    
    // Konfiguration der Reporting-Intervalle
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        
        // Basis-Bindings
        await reporting.bind(endpoint, coordinatorEndpoint, [
            'genPowerCfg', 
            'msTemperatureMeasurement', 
            'msRelativeHumidity',
            'msPressureMeasurement',
            'genAnalogInput'
        ]);
        
        // Temperatur Reporting (min 10s, max 300s, Änderung 0.5°C)
        await reporting.temperature(endpoint, {
            min: 10,
            max: 300,
            change: 50  // 0.5°C in 0.01°C units
        });
        
        // Luftfeuchtigkeit Reporting (min 10s, max 300s, Änderung 2%)
        await reporting.humidity(endpoint, {
            min: 10,
            max: 300,
            change: 200  // 2% in 0.01% units
        });
        
        // Luftdruck Reporting (min 10s, max 300s, Änderung 1 hPa)
        await reporting.pressure(endpoint, {
            min: 10,
            max: 300,
            change: 1
        });
        
        // Batterie Reporting (min 1h, max 12h, Änderung 1%)
        await reporting.batteryPercentageRemaining(endpoint, {
            min: 3600,
            max: 43200,
            change: 1
        });
        
        // Batteriespannung Reporting
        await reporting.batteryVoltage(endpoint, {
            min: 3600,
            max: 43200,
            change: 1  // 0.1V
        });
        
        // Bodenfeuchte Reporting
        await endpoint.configureReporting('genAnalogInput', [{
            attribute: 'presentValue',
            minimumReportInterval: 10,
            maximumReportInterval: 300,
            reportableChange: 5  // 5% Änderung
        }]);
    },
    
    // Exposes - Was in der UI angezeigt wird
    exposes: [
        e.temperature(),
        e.humidity(),
        e.pressure(),
        e.battery(),
        exposes.numeric('battery_voltage', ea.STATE)
            .withUnit('V')
            .withDescription('Battery voltage'),
        exposes.numeric('soil_moisture', ea.STATE)
            .withUnit('%')
            .withDescription('Soil moisture percentage'),
        exposes.text('soil_category', ea.STATE)
            .withDescription('Soil moisture category'),
        exposes.numeric('abs_humidity', ea.STATE)
            .withUnit('g/m³')
            .withDescription('Absolute humidity'),
        exposes.numeric('measure_interval', ea.STATE_SET)
            .withUnit('s')
            .withValueMin(10)
            .withValueMax(3600)
            .withDescription('Measurement interval in seconds'),
        exposes.text('wifi_ssid', ea.SET)
            .withDescription('WiFi SSID'),
        exposes.text('wifi_password', ea.SET)
            .withDescription('WiFi password'),
        exposes.binary('webserver_enabled', ea.SET, true, false)
            .withDescription('Enable/disable web server'),
    ],
    
    // Meta Information
    meta: {
        multiEndpoint: false,
        disableDefaultResponse: false,
    },
};

module.exports = definition;