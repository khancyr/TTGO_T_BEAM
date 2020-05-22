/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include "board_def.h"
#include <WiFi.h>
#include <Wire.h>
#include "axp20x.h"
#include <Button2.h>
#include <Ticker.h>
#include "droneID_FR.h"

extern "C" {
#include "esp_wifi.h"
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

#ifndef AXP192_SLAVE_ADDRESS
#define AXP192_SLAVE_ADDRESS    0x34
#endif

SSD1306_OBJECT();
UBLOX_GPS_OBJECT();

droneIDFR drone_idfr;

/********************************************************************************************************************
 * MODIFIEZ LES VALEURS ICI
 ********************************************************************************************************************/
// Set these to your desired credentials.
/**
  * Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
  */
const char ssid[] = "ILLEGAL_DRONE_AP";

// Mot de pass du wifi
const char *password = "123456789";
/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
  */
const char drone_id[] = "ILLEGAL_DRONE_APPELEZ_POLICE17";


/********************************************************************************************************************/
// NE PAS TOUCHEZ A PARTIR D'ICI !!!
// Le wifi est sur le channel 6 conformement à la spécification
static constexpr uint8_t wifi_channel = 6;
// Ensure the drone_id is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Ensure the drone_id is max 30 letters
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination
// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
        0x80, 0x00,							            // 0-1: Frame Control
        0x00, 0x00,							            // 2-3: Duration
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 10-15: Source address FAKE  // TODO should bet set manually
        0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 16-21: Source address FAKE
        0x00, 0x00,							            // 22-23: Sequence / fragment number (done by the SDK)
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
        0xB8, 0x0B,							            // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
        0x21, 0x04,							            // 34-35: Capability info
        0x03, 0x01, 0x06,						        // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
        0x00, 0x20,                     				// 39-40: SSID parameter set, 0x20:maxlength:content
                                                        // 41-XX: SSID (max 32)
};

bool has_set_home = false;
double home_alt = 0.0;

AXP20X_Class axp;
uint8_t program = 0;
bool ssd1306_found = false;
bool axp192_found = false;
bool loraBeginOK = false;

uint64_t dispMap = 0;
String dispInfo;
char buff[5][256];

uint64_t gpsSec = 0;
bool pmu_irq = false;
#define BUTTONS_MAP {BUTTON_PIN}

Button2 *pBtns = nullptr;
uint8_t g_btns[] =  BUTTONS_MAP;
#define ARRARY_SIZE(a)   (sizeof(a) / sizeof(a[0]))

Ticker btnTick;

String baChStatus = "No charging";
String recv = "";

/************************************
 *      BUTTON
 * *********************************/
void button_callback(Button2 &b)
{
    for (int i = 0; i < ARRARY_SIZE(g_btns); ++i) {
        if (pBtns[i] == b) {
#ifdef ENABLE_SSD1306
            if (ssd1306_found) {
                ui.nextFrame();
            }
#endif
            program = program + 1 > 2 ? 0 : program + 1;
        }
    }
}

void button_loop()
{
    for (int i = 0; i < ARRARY_SIZE(g_btns); ++i) {
        pBtns[i].loop();
    }
}

void button_init()
{
    uint8_t args = ARRARY_SIZE(g_btns);
    pBtns = new Button2 [args];
    for (int i = 0; i < args; ++i) {
        pBtns[i] = Button2(g_btns[i]);
        pBtns[i].setPressedHandler(button_callback);
    }
    pBtns[0].setLongClickHandler([](Button2 & b) {
#ifdef ENABLE_SSD1306
        if (ssd1306_found) {
            oled.displayOff();
        }
#endif
        Serial.println("Go to Sleep");
        if (axp192_found) {
            axp.setChgLEDMode(AXP20X_LED_OFF);
            axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
            axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
            axp.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
            // axp.setPowerOutPut(AXP192_DCDC3, AXP202_OFF);
            axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);
            axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
        }

        delay(20);
        esp_sleep_enable_ext1_wakeup(BUTTON_PIN_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
    });
}
/*#ifdef ENABLE_SSD1306
void msOverlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
    static char volbuffer[128];
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(ArialMT_Plain_10);

    display->drawString(0, 0, baChStatus);

    if (axp.isBatteryConnect()) {
        snprintf(volbuffer, sizeof(volbuffer), "%.2fV/%.2fmA", axp.getBattVoltage() / 1000.0, axp.isChargeing() ? axp.getBattChargeCurrent() : axp.getBattDischargeCurrent());
        display->drawString(62, 0, volbuffer);
    } else {
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);
        snprintf(volbuffer, sizeof(volbuffer), "%u/%uKB",  info.total_allocated_bytes / 1024, info.total_free_bytes / 1024);
        display->drawString(75, 0, volbuffer);
    }
}

void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{

    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);

    if (!gps.location.isValid()) {
        display->drawString(64 + x, 11 + y, buff[0]);
        display->drawString(64 + x, 22 + y, buff[1]);
    } else {
        display->drawString(64 + x, 11 + y, buff[0]);
        display->drawString(64 + x, 22 + y, buff[1]);
        display->drawString(64 + x, 33 + y, buff[2]);
        display->drawString(64 + x, 44 + y, buff[3]);
    }
}

void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 11 + y, buff[0]);
    display->drawString(64 + x, 22 + y, buff[1]);
}

void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 9 + y, buff[0]);
    display->drawString(64 + x, 22 + y, recv == "" ? "No message" : recv);
    display->drawString(64 + x, 35 + y, buff[1]);
}

//PMU
void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    if (!axp192_found) {
        display->drawString(64 + x, 22 + y, "PMU Begin FAIL");
        return;
    }
    //TODO::
    display->drawString(64 + x, 22 + y, "Empty");
}


FrameCallback frames[] = {drawFrame1, drawFrame2, drawFrame3, *//*drawFrame4*//*};
OverlayCallback overlays[] = { msOverlay };
#endif*/

void ssd1306_init()
{
#ifdef ENABLE_SSD1306
    if (!ssd1306_found) {
        Serial.println("SSD1306 not found");
        return;
    }
    if (oled.init()) {
        oled.flipScreenVertically();
        oled.setFont(ArialMT_Plain_16);
        oled.setTextAlignment(TEXT_ALIGN_CENTER);
    } else {
        Serial.println("SSD1306 Begin FAIL");
    }
    Serial.println("SSD1306 Begin PASS");
    ui.setTargetFPS(30);
    ui.disableAutoTransition();
    ui.setIndicatorPosition(BOTTOM);
    ui.setIndicatorDirection(LEFT_RIGHT);
    ui.setFrameAnimation(SLIDE_LEFT);
    ui.setFrames(frames, ARRARY_SIZE(frames));
    if (axp192_found) {
        ui.setOverlays(overlays, ARRARY_SIZE(overlays));
    }
#endif
}



void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS) {
                ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }
            if (addr == AXP192_SLAVE_ADDRESS) {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}



void playSound()
{
#ifdef ENABLE_BUZZER
    ledcWriteTone(0, 1000);
    delay(200);
    ledcWriteTone(0, 0);
#endif
}



void lora_init()
{
#ifdef ENABLE_LOAR
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
    if (!LoRa.begin(BAND))
        Serial.println("LORA Begin FAIL");
    else {
        loraBeginOK = true;
        Serial.println("LORA Begin PASS");
    }
#endif
}

/**
 * Phase de configuration.
 */
void setup()
{
    Serial.begin(115200);

    delay(1000);

    Wire.begin(I2C_SDA, I2C_SCL);

    scanI2Cdevice();

#ifdef ENABLE_BUZZER
    ledcSetup(0, 1000, 8);
    ledcAttachPin(BUZZER_PIN, 0);
#endif

    playSound();
    playSound();

    if (axp192_found) {
        if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
        }

        // axp.setChgLEDMode(LED_BLINK_4HZ);

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

        Serial.println("----------------------------------------");

        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp.setDCDC1Voltage(3300);  //esp32 core VDD    3v3
        axp.setLDO2Voltage(3300);   //LORA VDD set 3v3
        axp.setLDO3Voltage(3300);   //GPS VDD      3v3

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");


        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp.clearIRQ();

        if (axp.isChargeing()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP192 not found");
    }

    button_init();

    ssd1306_init();

#ifdef ENABLE_GPS
    Serial1.begin(GPS_BANUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

#ifdef ENABLE_LOAR
    lora_init();
#endif
/********************************************************************************************************************
 * ICI ON INITIALISE LE WIFI
 */
    /**
     * Pour mon exemple, je crée un point d'accés. Il fait rien par defaut.
     */
    Serial.println("Starting AP");
    WiFi.softAP(ssid, nullptr, wifi_channel);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.print("AP mac address: ");
    Serial.println(WiFi.macAddress());
    wifi_config_t conf_current;
    esp_wifi_get_config(WIFI_IF_AP, &conf_current);
    // Change WIFI AP default beacon interval sending to 1s.
    conf_current.ap.beacon_interval = 1000;
    drone_idfr.set_drone_id(drone_id);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));

    btnTick.attach_ms(20, button_loop);
}

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
#ifdef ENABLE_LOAR
    static uint32_t loraMap = 0;
#endif
    static uint64_t gpsMap = 0;
    if (axp192_found && pmu_irq) {
        pmu_irq = false;
        axp.readIRQ();
        if (axp.isChargingIRQ()) {
            baChStatus = "Charging";
        } else {
            baChStatus = "No Charging";
        }
        if (axp.isVbusRemoveIRQ()) {
            baChStatus = "No Charging";
        }
        digitalWrite(2, !digitalRead(2));
        axp.clearIRQ();
    }
    switch (program) {
    case 0:
        // Ici on lit les données qui arrive du GPS et on les passes à la librairie TinyGPS++ pour les traiter
        while (Serial1.available())
            gps.encode(Serial1.read());
        // On traite le case ou le GPS a un probleme
        if (millis() > 5000 && gps.charsProcessed() < 10) {
            snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
            snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
            if (!ssd1306_found) {
                Serial.println(buff[1]);
            }
            return;
        }
        // On traite le cas si la position GPS n'est pas valide
        if (!gps.location.isValid()) {
            if (millis() - gpsMap > 1000) {
                snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
                snprintf(buff[1], sizeof(buff[1]), "Positioning(%llu)", gpsSec++);
                if (!ssd1306_found) {
                    Serial.println(buff[1]);
                }
                gpsMap = millis();
            }
        } else {
            // On traite le case si la position GPS est valide.
            // On renseigne le point de démarrage quand la précision est satisfaisante
            if (!has_set_home && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
                Serial.println("Setting Home Position");
                drone_idfr.set_home_lat_lon(gps.location.lat(), gps.location.lng());
                has_set_home = true;
                home_alt = gps.altitude.meters();
            }
            // On envoie les données a la librairie d'identification drone pour le formattage.
            drone_idfr.set_lat_lon(gps.location.lat(), gps.location.lng());
            drone_idfr.set_altitude(gps.altitude.meters());
            drone_idfr.set_heading(gps.course.deg());
            drone_idfr.set_ground_speed(gps.speed.mps());
            drone_idfr.set_heigth(gps.altitude.meters() - home_alt);
            // Ici on ecrit sur le port USB les données GPS pour visualisation seulement.
            if (millis() - gpsMap > 1000) {
                playSound();
                snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
                snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
                snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
                snprintf(buff[3], sizeof(buff[3]), "satellites:%u", gps.satellites.value());
                if (!ssd1306_found) {
                    Serial.println(buff[0]);
                    Serial.println(buff[1]);
                    Serial.println(buff[2]);
                    Serial.println(buff[3]);
                }
                gpsMap = millis();
            }
        }
        break;
#ifdef ENABLE_LOAR
    case 1:
        snprintf(buff[0], sizeof(buff[0]), "T-Beam Lora Sender");
        if (!loraBeginOK) {
            snprintf(buff[1], sizeof(buff[1]), "Lora Begin FAIL");
            if (!ssd1306_found) {
                Serial.println(buff[1]);
            }
            return;
        }

        if (millis() - loraMap > 3000) {
            LoRa.beginPacket();
            LoRa.print("lora: ");
            LoRa.print(loraMap);
            LoRa.endPacket();
            snprintf(buff[1], sizeof(buff[1]), "Send %u", loraMap);
            loraMap = millis();
            if (!ssd1306_found) {
                Serial.println(buff[1]);
            }
        }
        break;
    case 2:
        if (!loraBeginOK) {
            recv =  "Lora Begin FAIL";
            if (!ssd1306_found) {
                Serial.println(recv);
            }
            return;
        }
        snprintf(buff[0], sizeof(buff[0]), "T-Beam Lora Received");
        if (LoRa.parsePacket()) {
            recv = "";
            while (LoRa.available()) {
                recv += (char)LoRa.read();
            }
            if (!ssd1306_found) {
                Serial.printf("Lora Received:%s - rssi:%d\n", recv.c_str(), LoRa.packetRssi());
            }
        } else {
            // if (!ssd1306_found) {
            //     Serial.println("Wait for received message");
            //     delay(500);
            // }
        }
        snprintf(buff[1], sizeof(buff[1]), "rssi:%d", LoRa.packetRssi());
        break;
#endif
    }
    /**
     * On regarde s'il temps d'envoyer la trame d'identification drone: soit toutes les 3s soit si le drones s'est déplacé de 30m en moins de 3s.
     */
    if (drone_idfr.time_to_send()) {
        Serial.println("Send beacon");
        /**
         * On commence par renseigner le ssid du wifi dans la trame
         */
        // write new SSID into beacon frame
        const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
        beaconPacket[40] = ssid_size;  // set size
        memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
        const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
        /**
         * On génère la trame wifi avec l'identfication
         */
        const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
        // Décommenter ce block pour voir la trame entière sur le port usb
        /* Serial.println("beaconPacket : ");
        for (auto i=0; i<sizeof(beaconPacket);i++) {
            Serial.print(beaconPacket[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");*/

        /**
         * On envoie la trame
         */
        ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));
        /**
         * On reset la condition d'envoi
         */
        drone_idfr.set_last_send();
    }
    /*
    if (ssd1306_found) {
        if (ui.update()) {
            button_loop();
        }
    } else {
        button_loop();
    }
     */
#ifdef ENABLE_SSD1306
    if (ssd1306_found) {
        if (ui.update()) {
        }
    }
#endif
}
