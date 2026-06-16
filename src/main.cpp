#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"

// --- Hardware ---
TFT_eSPI tft = TFT_eSPI();

// --- Bluetooth & OBD ---
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
ELM327 myELM327;

const char* elmName = "OBDII"; 
bool connectedToELM = false;
bool btConnecting = false;

// --- LVGL Display Buffer ---
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 20]; // 20 lines

// --- UI Elements ---
lv_obj_t * shift_leds[10];
lv_obj_t * rpm_val_label;
lv_obj_t * bat_val_label;
lv_obj_t * coolant_val_label;
lv_obj_t * iat_val_label;
lv_obj_t * load_val_label;
lv_obj_t * map_val_label;
lv_obj_t * avg_kml_label;

lv_obj_t * load_bar;
lv_obj_t * status_label;

int currentRpm = 0;
float currentBat = 0.0;
int currentCoolantTemp = 0;
int currentLoad = 0;
int currentIat = 0;
int currentMap = 0;
int currentKph = 0;
float tripDistance = 0.0;
float tripFuel = 0.0;
float currentAvgKml = 0.0;
unsigned long lastCalcTime = 0;

// --- Diesel Fuel Economy Calibration Constants ---
const float ENGINE_DISPLACEMENT = 1.248;   // 1.3L Multijet (1248 cc)
const float VOLUMETRIC_EFFICIENCY = 0.85; // Est. average VE for turbo-diesel
const float DIESEL_DENSITY = 835.0;        // g/L

unsigned long lastElmUpdate = 0;
const unsigned long elmInterval = 20; // Decreased interval to make checks frequent and responsive
unsigned long lastCoolantQuery = 0;
unsigned long lastIatQuery = 0;
unsigned long lastBatQuery = 0;

// --- LVGL Display Flush Callback ---
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp_drv);
}

// --- Helper to create styled data blocks ---
lv_obj_t* createDataBlock(lv_obj_t* parent, int x, int y, int w, int h, const char* title, lv_color_t valColor) {
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_set_size(cont, w, h);
  lv_obj_set_pos(cont, x, y);
  
  // F1 aesthetic: very dark gray, crisp border
  lv_obj_set_style_bg_color(cont, lv_color_hex(0x111111), 0);
  lv_obj_set_style_border_color(cont, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(cont, 2, 0);
  lv_obj_set_style_radius(cont, 6, 0);
  lv_obj_set_style_pad_all(cont, 2, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* title_label = lv_label_create(cont);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title_label, lv_color_hex(0x888888), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t* val_label = lv_label_create(cont);
  lv_label_set_text(val_label, "--");
  lv_obj_set_style_text_font(val_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(val_label, valColor, 0);
  lv_obj_align(val_label, LV_ALIGN_BOTTOM_MID, 0, 0);

  return val_label;
}

// --- UI Construction ---
void buildUI() {
  lv_obj_t * scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN); // Pitch black background

  // 1. Shift Lights (Top Row)
  // 10 LEDs. 320 width / 10 = ~32 spacing. Let's make them 20x10 px.
  int led_start_x = 10;
  int led_spacing = 30;
  for(int i=0; i<10; i++) {
    shift_leds[i] = lv_led_create(scr);
    lv_obj_set_size(shift_leds[i], 24, 12);
    lv_obj_set_pos(shift_leds[i], led_start_x + (i * led_spacing), 5);
    
    // Set colors based on F1 logic (4 Green, 3 Yellow, 3 Red)
    if (i < 4) {
      lv_led_set_color(shift_leds[i], lv_color_hex(0x00FF00)); // Green
    } else if (i < 7) {
      lv_led_set_color(shift_leds[i], lv_color_hex(0xFFFF00)); // Yellow
    } else {
      lv_led_set_color(shift_leds[i], lv_color_hex(0xFF0000)); // Red
    }
    lv_led_off(shift_leds[i]);
  }

  // 2. Central RPM Module
  lv_obj_t* center_cont = lv_obj_create(scr);
  lv_obj_set_size(center_cont, 140, 140);
  lv_obj_set_pos(center_cont, 90, 30);
  lv_obj_set_style_bg_color(center_cont, lv_color_hex(0x111111), 0);
  lv_obj_set_style_border_color(center_cont, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(center_cont, 2, 0);
  lv_obj_set_style_radius(center_cont, 8, 0);
  lv_obj_clear_flag(center_cont, LV_OBJ_FLAG_SCROLLABLE);

  rpm_val_label = lv_label_create(center_cont);
  lv_label_set_text(rpm_val_label, "0");
  lv_obj_set_style_text_font(rpm_val_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(rpm_val_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(rpm_val_label, LV_ALIGN_CENTER, 0, -10);

  lv_obj_t* center_title = lv_label_create(center_cont);
  lv_label_set_text(center_title, "RPM");
  lv_obj_set_style_text_font(center_title, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(center_title, lv_color_hex(0x00FFFF), 0);
  lv_obj_align(center_title, LV_ALIGN_BOTTOM_MID, 0, -5);

  // 3. Peripheral Modules (Left Column)
  int col_w = 80;
  int row_h = 43;
  int left_x = 5;
  int right_x = 235;
  int y1 = 30;
  int y2 = 78;
  int y3 = 126;

  coolant_val_label = createDataBlock(scr, left_x, y1, col_w, row_h, "TEMP C", lv_color_hex(0xFF8800));
  iat_val_label     = createDataBlock(scr, left_x, y2, col_w, row_h, "IAT C", lv_color_hex(0xFF8800));
  load_val_label    = createDataBlock(scr, left_x, y3, col_w, row_h, "LOAD %", lv_color_hex(0xBF00FF));

  // 4. Peripheral Modules (Right Column)
  avg_kml_label     = createDataBlock(scr, right_x, y1, col_w, row_h, "AVG KML", lv_color_hex(0x00FF00));
  map_val_label     = createDataBlock(scr, right_x, y2, col_w, row_h, "MAP", lv_color_hex(0x00FFFF));
  bat_val_label     = createDataBlock(scr, right_x, y3, col_w, row_h, "BAT V", lv_color_hex(0xFF8800));
  
  // 5. Bottom Load Bar & Status
  lv_obj_t* bottom_cont = lv_obj_create(scr);
  lv_obj_set_size(bottom_cont, 310, 50);
  lv_obj_set_pos(bottom_cont, 5, 175);
  lv_obj_set_style_bg_color(bottom_cont, lv_color_hex(0x111111), 0);
  lv_obj_set_style_border_color(bottom_cont, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(bottom_cont, 2, 0);
  lv_obj_clear_flag(bottom_cont, LV_OBJ_FLAG_SCROLLABLE);

  load_bar = lv_bar_create(bottom_cont);
  lv_obj_set_size(load_bar, 200, 20);
  lv_obj_align(load_bar, LV_ALIGN_LEFT_MID, 0, 0);
  lv_bar_set_range(load_bar, 0, 100);
  lv_obj_set_style_bg_color(load_bar, lv_color_hex(0x222222), LV_PART_MAIN);
  lv_obj_set_style_bg_color(load_bar, lv_color_hex(0xFFFF00), LV_PART_INDICATOR); 

  status_label = lv_label_create(bottom_cont);
  lv_label_set_text(status_label, "BT Disc");
  lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), 0);
  lv_obj_align(status_label, LV_ALIGN_RIGHT_MID, 0, 0);
}

void updateUI() {
  char buf[32];

  // 1. Update Shift Lights (Calibrated for Diesel)
  int shift_start = 1800; 
  int shift_end = 3500;   
  float step = (float)(shift_end - shift_start) / 10.0;
  
  for(int i = 0; i < 10; i++) {
    int threshold = shift_start + (i * step);
    if (currentRpm >= threshold) {
      lv_led_on(shift_leds[i]);
    } else {
      lv_led_off(shift_leds[i]);
    }
  }

  // Flash Red LEDs if over shift_end
  if (currentRpm >= shift_end && (millis() / 100) % 2 == 0) {
    for(int i=7; i<10; i++) lv_led_off(shift_leds[i]);
  }

  // 2. Update RPM
  sprintf(buf, "%d", currentRpm);
  lv_label_set_text(rpm_val_label, buf);

  // Update Trip Average KM/L
  if (currentAvgKml > 0.0) {
    sprintf(buf, "%.1f", currentAvgKml);
  } else {
    sprintf(buf, "--");
  }
  lv_label_set_text(avg_kml_label, buf);

  // Update Battery
  sprintf(buf, "%.1f V", currentBat);
  lv_label_set_text(bat_val_label, buf);

  // Update Coolant/Peripheral Data
  sprintf(buf, "%d", currentCoolantTemp);
  lv_label_set_text(coolant_val_label, buf);

  sprintf(buf, "%d", currentIat);
  lv_label_set_text(iat_val_label, buf);

  sprintf(buf, "%d", currentLoad);
  lv_label_set_text(load_val_label, buf);

  sprintf(buf, "%.1f", currentBat);
  lv_label_set_text(bat_val_label, buf);

  sprintf(buf, "%d", currentMap);
  lv_label_set_text(map_val_label, buf);

  // 4. Update Bottom Bar
  lv_bar_set_value(load_bar, currentLoad, LV_ANIM_ON);
}

void updateBTStatus() {
  if (connectedToELM) {
    lv_label_set_text(status_label, "BT ON");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF00), 0);
  } else if (btConnecting) {
    lv_label_set_text(status_label, "BT Wait");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFFA500), 0);
  } else {
    lv_label_set_text(status_label, "BT Disc");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF0000), 0);
  }
}

void connectBluetooth() {
  btConnecting = true;
  updateBTStatus();
  lv_task_handler(); // Force UI update
  
  Serial.println("Scanning the air for OBDII...");
  BTScanResults* results = ELM_PORT.discover(5000); // 5 sec scan

  uint8_t obdMac[6] = {0};
  bool found = false;

  if (results) {
    for (int i = 0; i < results->getCount(); i++) {
      BTAdvertisedDevice* device = results->getDevice(i);
      Serial.printf("Found: %s (MAC: %s)\n", device->getName().c_str(), device->getAddress().toString().c_str());
      
      String name = device->getName().c_str();
      if (name == "OBDII" || name == "obdii") {
        Serial.println("Found match!");
        memcpy(obdMac, (uint8_t*)device->getAddress().getNative(), 6);
        found = true;
        break;
      }
    }
  }

  if (!found) {
    Serial.println("Couldn't find OBDII in the air! Make sure phone is disconnected.");
    connectedToELM = false;
    btConnecting = false;
    updateBTStatus();
    return;
  }

  esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE; 
  esp_spp_role_t role = ESP_SPP_ROLE_SLAVE; 

  Serial.println("Attempting to connect to OBDII via scanned MAC...");
  if (!ELM_PORT.connect(obdMac, 0, sec_mask, role)) {
    Serial.println("Couldn't connect to OBD scanner");
    connectedToELM = false;
    btConnecting = false;
    updateBTStatus();
    return;
  }

  Serial.println("Connected to Bluetooth OBDII scanner!");
  
  // DISABLE '1' SUFFIX (byteExpected=0) to fix clone timeouts!
  if (!myELM327.begin(ELM_PORT, true, 2000, '0', 20, 0)) {
    Serial.println("Couldn't initialize ELM327");
    connectedToELM = false;
  } else {
    Serial.println("Connected to ELM327!");
    connectedToELM = true;
  }
  
  btConnecting = false;
  updateBTStatus();
}

void setup() {
  Serial.begin(115200);
  
  // Turn on CYD backlight
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  // Init TFT
  tft.init();
  tft.setRotation(1); // Landscape
  
  // Disable hardware inversion so pitch black actually renders as black!
  tft.invertDisplay(false);

  // Init LVGL
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 20);

  // Init LVGL display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Build the F1 UI
  buildUI();

  // Initialize BT
  ELM_PORT.begin("ESP32_CYD_Gauge", true); 
  connectBluetooth();
}

void loop() {
  // LVGL tick & task handler
  static unsigned long lastTick = 0;
  unsigned long now = millis();
  lv_tick_inc(now - lastTick);
  lastTick = now;
  lv_task_handler();

  if (!connectedToELM) {
    // Retry connection every 5 seconds
    static unsigned long lastRetry = 0;
    if (now - lastRetry > 5000) {
      lastRetry = now;
      connectBluetooth();
    }
    delay(5);
    return;
  }

  // Poll OBD in a highly optimized staggered non-blocking loop
  if (now - lastElmUpdate > elmInterval) {
    lastElmUpdate = now;

    static int pollState = 0;
    switch(pollState) {
      case 0: {
        float val = myELM327.rpm();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentRpm = (int)val;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) pollState++;
        break;
      }
      case 1: {
        float val = myELM327.kph();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentKph = (int)val;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) pollState++;
        break;
      }
      case 2: {
        if (now - lastCoolantQuery < 5000) { // Poll coolant every 5 seconds
          pollState++;
          break;
        }
        float val = myELM327.engineCoolantTemp();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentCoolantTemp = (int)val;
          lastCoolantQuery = now;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
          lastCoolantQuery = now;
          pollState++;
        }
        break;
      }
      case 3: {
        float val = myELM327.engineLoad();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentLoad = (int)val;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) pollState++;
        break;
      }
      case 4: {
        if (now - lastIatQuery < 5000) { // Poll intake air temp every 5 seconds
          pollState++;
          break;
        }
        float val = myELM327.intakeAirTemp();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentIat = (int)val;
          lastIatQuery = now;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
          lastIatQuery = now;
          pollState++;
        }
        break;
      }
      case 5: {
        float val = myELM327.manifoldPressure();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentMap = (int)val;
          pollState++;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) pollState++;
        break;
      }
      case 6: {
        if (now - lastBatQuery < 5000) { // Poll battery voltage every 5 seconds
          pollState = 0;
          break;
        }
        float val = myELM327.batteryVoltage();
        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          currentBat = val;
          lastBatQuery = now;
          pollState = 0; // Loop back to start
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
          lastBatQuery = now;
          pollState = 0;
        }
        break;
      }
    }

    // --- Trip Fuel & Distance Integration ---
    if (currentRpm > 200) {
      if (lastCalcTime == 0) {
        lastCalcTime = now;
      } else {
        float dt = (float)(now - lastCalcTime) / 1000.0;
        lastCalcTime = now;

        if (dt > 0.0 && dt < 2.0) { // Prevent anomalies
          // 1. Calculate synthetic MAF (g/s)
          float iatK = (float)currentIat + 273.15;
          float maf = 0.0;
          if (iatK > 0) {
            maf = (float)currentRpm * (float)currentMap / iatK * 0.0308028;
          }

          // 2. Calculate dynamic AFR
          float afr = 65.0 - ((float)currentLoad * 0.47);
          if (afr < 18.0) afr = 18.0;
          if (afr > 65.0) afr = 65.0;

          // 3. Fuel Flow in Liters per second
          float fuelFlowLps = maf / (afr * DIESEL_DENSITY);

          // 4. Integrate
          tripFuel += fuelFlowLps * dt;
          tripDistance += ((float)currentKph / 3600.0) * dt;

          // 5. Update Average KM/L
          if (tripFuel > 0.0001) {
            currentAvgKml = tripDistance / tripFuel;
          }
        }
      }
    } else {
      lastCalcTime = 0; // Reset timer when engine is off
    }

    updateUI();
  }
  
  delay(5);
}
