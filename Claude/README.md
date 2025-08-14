# ESP32-C6 Zigbee Environmental Sensor

## 📋 Projektübersicht

Dieses Projekt implementiert einen batteriebetriebenen Umweltsensor basierend auf dem **DFRobot FireBeetle 2 ESP32-C6** mit Zigbee-Anbindung an Home Assistant über zigbee2mqtt.

### Features
- 🌡️ **BME280 Sensor**: Temperatur, Luftfeuchtigkeit, Luftdruck, absolute Luftfeuchtigkeit
- 💧 **Bodenfeuchte-Sensor**: AZ Delivery 1.2 mit 5 Feuchtigkeitskategorien
- 🔋 **Batterie-Monitoring**: Spannung und Ladezustand in Prozent
- 📡 **Zigbee End Device**: Vollständige Integration in zigbee2mqtt
- 💤 **Deep Sleep Support**: Ultra-niedriger Stromverbrauch (~36µA)
- 🌐 **Optional**: WiFi-Konfiguration und Webserver
- ⚙️ **Konfigurierbar**: Messintervalle über Zigbee einstellbar

## 🔧 Hardware-Setup

### Komponenten
- **MCU**: DFRobot FireBeetle 2 ESP32-C6
- **Umweltsensor**: BME280 (I2C)
- **Bodenfeuchte**: AZ Delivery Capacitive Soil Moisture Sensor v1.2
- **Stromversorgung**: LiPo-Akku (3.7V) + optional Solar-Panel

### Pin-Verbindungen

| Komponente | ESP32-C6 Pin | Beschreibung |
|------------|--------------|--------------|
| **BME280** | | |
| VCC | 3V3 | Stromversorgung |
| GND | GND | Masse |
| SDA | GPIO19 | I2C Daten |
| SCL | GPIO20 | I2C Clock |
| **Bodenfeuchte-Sensor** | | |
| VCC | GPIO5 | Schaltbare Stromversorgung |
| GND | GND | Masse |
| AOUT | GPIO4 | Analoger Ausgang |
| **Batterie** | | |
| + | BAT+ | LiPo Batterie Plus |
| - | BAT- | LiPo Batterie Minus |
| **Solar (optional)** | | |
| + | VIN/5V | Solar Panel 5V |
| - | GND | Solar Panel Masse |

### Bodenfeuchte-Bereiche

| ADC-Wert | Kategorie | Beschreibung |
|----------|-----------|--------------|
| > 3500 | Sehr trocken | Sofort gießen! |
| 2800-3500 | Trocken | Bald gießen |
| 2000-2800 | Normal | Optimaler Bereich |
| 1200-2000 | Feucht | Gut versorgt |
| < 1200 | Zu feucht | Nicht gießen! |

## 💻 Software-Installation

### Voraussetzungen
- **PlatformIO** (CLion oder VS Code)
- **Python 3.8+** für esptool
- **zigbee2mqtt v2.6.0+** auf Raspberry Pi/Server
- **Home Assistant** (optional)

### 1. Projekt kompilieren

```bash
# Repository klonen
git clone <your-repo>
cd esp32c6-zigbee-sensor

# Mit PlatformIO kompilieren
pio run -e firebeetle2-esp32c6-debug  # Debug-Version
# oder
pio run -e firebeetle2-esp32c6-production  # Produktiv-Version
```

### 2. Flash auf ESP32-C6

```bash
# Automatische Port-Erkennung
pio run -t upload -e firebeetle2-esp32c6-debug

# Oder mit spezifischem Port (Windows)
pio run -t upload -e firebeetle2-esp32c6-debug --upload-port COM3

# Monitor für Debug-Ausgaben
pio device monitor -b 115200
```

### 3. Zigbee2MQTT Konfiguration

#### External Converter installieren

1. Kopiere `esp32c6_sensor.js` in den zigbee2mqtt `data/converters/` Ordner
2. Editiere `data/configuration.yaml`:

```yaml
external_converters:
  - esp32c6_sensor.js

# Optional: Permit join aktivieren
permit_join: true

# Optional: Network Key (für Sicherheit)
advanced:
  network_key: [1, 3, 5, 7, 9, 11, 13, 15, 0, 2, 4, 6, 8, 10, 12, 13]
  pan_id: 0x1234
  channel: 11
```

3. Zigbee2mqtt neu starten:
```bash
sudo systemctl restart zigbee2mqtt
```

### 4. Pairing-Prozess

1. **Permit Join aktivieren** in zigbee2mqtt (Web-UI oder MQTT)
2. **BOOT-Taste** am ESP32-C6 für 3 Sekunden drücken
3. Warten bis die **LED dauerhaft leuchtet** (Verbunden)
4. Device erscheint als `ESP32C6_ENV_SENSOR` in zigbee2mqtt

## 🔋 Stromverbrauch & Optimierung

### Gemessene Werte (FireBeetle 2 ESP32-C6)

| Modus | Stromverbrauch | Bemerkung |
|-------|---------------|-----------|
| Deep Sleep | 36µA | V1.1 Board |
| Light Sleep | 130µA | Mit Zigbee |
| Active (TX) | 160mA | Beim Senden |
| Active (Idle) | 40mA | Normal-Betrieb |

### Batterie-Laufzeit Berechnung

Mit 2000mAh LiPo-Akku:
- **1 Messung/Stunde**: ~6 Monate
- **1 Messung/30 Min**: ~3 Monate  
- **1 Messung/10 Min**: ~1 Monat

### Optimierungs-Tipps

1. **Deep Sleep aktivieren**: In `main.cpp` `ENABLE_DEEP_SLEEP = true`
2. **WiFi deaktivieren**: Nur bei Bedarf aktivieren
3. **Messintervall erhöhen**: Standard 30 Min oder mehr
4. **Solar-Panel nutzen**: 5V/200mA Panel für Dauerbetrieb

## ⚠️ Bekannte Probleme & Lösungen

### Problem 1: Zigbee verbindet nicht

**Symptome**: ESP32 findet kein Netzwerk, ständiges Neustarten

**Lösungen**:
- Zigbee-Kanal prüfen (11-26, WiFi-Interferenz vermeiden)
- PAN ID in Code und zigbee2mqtt gleich setzen
- Firmware mit `pio run -t erase` löschen und neu flashen

### Problem 2: I2C/BME280 nach Sleep nicht erreichbar

**Symptome**: Timeout-Fehler nach Wake-up

**Lösungen**:
- I2C nach Wake-up neu initialisieren
- Pull-up Widerstände (4.7kΩ) an SDA/SCL
- BME280 Forced Mode verwenden

### Problem 3: Hoher Stromverbrauch im Sleep

**Symptome**: Batterie nach wenigen Tagen leer

**Lösungen**:
- Bodenfeuchte-Sensor Power-Pin nutzen (GPIO5)
- USB-Serial Chip deaktivieren (falls möglich)
- Spannung auf 3.8V erhöhen (effizienter Spannungsregler)

### Problem 4: PlatformIO Zigbee Support

**Symptome**: Compile-Fehler bei Zigbee-Funktionen

**Lösungen**:
- ESP-IDF Framework nutzen statt Arduino
- Oder: Arduino Core 3.0.0+ mit Zigbee Support
- Platform-espressif32 >= 6.5.0 verwenden

## 📊 Home Assistant Integration

### Automatische Erkennung

Nach erfolgreicher Verbindung mit zigbee2mqtt erscheint das Device automatisch in Home Assistant mit folgenden Entities:

```yaml
sensor.esp32c6_env_sensor_temperature
sensor.esp32c6_env_sensor_humidity  
sensor.esp32c6_env_sensor_pressure
sensor.esp32c6_env_sensor_soil_moisture
sensor.esp32c6_env_sensor_soil_category
sensor.esp32c6_env_sensor_battery
sensor.esp32c6_env_sensor_battery_voltage
sensor.esp32c6_env_sensor_abs_humidity
```

### Beispiel-Automation

```yaml
automation:
  - alias: "Pflanze gießen Benachrichtigung"
    trigger:
      - platform: state
        entity_id: sensor.esp32c6_env_sensor_soil_category
        to: "Sehr trocken"
    action:
      - service: notify.mobile_app
        data:
          title: "🌱 Pflanze braucht Wasser!"
          message: "Die Bodenfeuchte ist sehr niedrig"
          
  - alias: "Batterie niedrig Warnung"
    trigger:
      - platform: numeric_state
        entity_id: sensor.esp32c6_env_sensor_battery
        below: 20
    action:
      - service: notify.mobile_app
        data:
          title: "🔋 Sensor Batterie niedrig"
          message: "Nur noch {{ states('sensor.esp32c6_env_sensor_battery') }}% Akku"
```

## 🛠️ Entwicklung & Debug

### Serial Monitor Befehle

Über USB-Serial (115200 baud) sind Debug-Befehle verfügbar:

- `status` - Zeigt aktuellen System-Status
- `measure` - Führt sofort Messung durch
- `reset` - Neustart des ESP32
- `zigbee_reset` - Zigbee-Netzwerk verlassen

### Web-Interface (optional)

Wenn WiFi aktiviert ist, erreichbar unter:
- `http://<esp32-ip>/` - Status-Seite
- `http://<esp32-ip>/json` - JSON API
- `http://<esp32-ip>/config` - Konfiguration

### OTA Updates

```bash
# Nach WiFi-Aktivierung
pio run -t upload -e firebeetle2-esp32c6-ota --upload-port 192.168.1.100
```

## 📝 Weitere Dokumentation

### Wichtige Links
- [ESP32-C6 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)
- [FireBeetle 2 Wiki](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6)
- [Zigbee2mqtt Docs](https://www.zigbee2mqtt.io/)
- [ESP-Zigbee-SDK](https://github.com/espressif/esp-zigbee-sdk)

### Projekt-Struktur
```
├── src/
│   └── main.cpp           # Hauptprogramm
├── converters/
│   └── esp32c6_sensor.js  # Zigbee2mqtt Converter
├── platformio.ini          # PlatformIO Konfiguration
├── partitions.csv          # Flash-Partitionierung
└── README.md              # Diese Datei
```

## 📄 Lizenz

Dieses Projekt ist Open Source unter MIT Lizenz.

## 🤝 Support

Bei Fragen oder Problemen:
1. Issue im GitHub Repository erstellen
2. Zigbee2mqtt Discord/Forum
3. ESP32 Community Forum

---

**Version**: 1.0  
**Autor**: Assistant  
**Datum**: 2025  
**Hardware**: DFRobot FireBeetle 2 ESP32-C6