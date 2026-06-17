#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <Preferences.h>

// --- Hardware ---
TFT_eSPI tft = TFT_eSPI();

// --- Bluetooth & OBD ---
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
ELM327 myELM327;

const char* elmName = "OBDII"; 
bool connectedToELM = false;
bool btConnecting = false;

// --- Preferences & Fast Connect ---
Preferences preferences;
bool hasStoredMac = false;
uint8_t storedMac[6] = {0};
int failedDirectConnects = 0;

// --- LDR Backlight Control ---
#define LDR_PIN 36
#define BACKLIGHT_PIN 21
#define PWM_CHANNEL 0
float smoothLdrVal = 2000.0;
unsigned long lastLdrRead = 0;

// --- LVGL Display Buffer ---
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 20]; 

// --- UI Elements ---
lv_obj_t * shift_leds[10];
lv_obj_t * rpm_val_label;
lv_obj_t * bat_val_label;
lv_obj_t * coolant_val_label;
lv_obj_t * iat_val_label;
lv_obj_t * load_val_label;
lv_obj_t * boost_val_label;
lv_obj_t * avg_kml_label;
lv_obj_t * load_bar;
lv_obj_t * status_label;

// --- Data Variables ---
int currentRpm = 0;
float currentBat = 0.0;
int currentCoolantTemp = 0;
int currentLoad = 0;
int currentIat = 0;
int currentMap = 0;
float currentBoost = 0.0;
int currentKph = 0;
float tripDistance = 0.0;
float tripFuel = 0.0;
float currentAvgKml = 0.0;
unsigned long lastCalcTime = 0;

// --- Diesel Constants (Ritz 1.3 DDiS) ---
const float DIESEL_DENSITY = 835.0; // g/L

// --- Polling Timers ---
unsigned long lastElmUpdate = 0;
const unsigned long elmInterval = 20; // Reverted back to 20 for stable clone processing
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
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t* val_label = lv_label_create(cont);
  lv_label_set_text(val_label, "--");
  lv_obj_set_style_text_font(val_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(val_label, valColor, 0);
  lv_obj_align(val_label, LV_ALIGN_BOTTOM_MID, 0, -2);

  return val_label;
}

void buildUI() {
  lv_obj_t * scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);

  // 1. Shift Lights (Top Row)
  int led_start_x = 10;
  int led_spacing = 30;
  for(int i=0; i<10; i++) {
    shift_leds[i] = lv_led_create(scr);
    lv_obj_set_size(shift_leds[i], 24, 12);
    lv_obj_set_pos(shift_leds[i], led_start_x + (i * led_spacing), 5);
    if (i < 4) lv_led_set_color(shift_leds[i], lv_color_hex(0x00FF00));
    else if (i < 7) lv_led_set_color(shift_leds[i], lv_color_hex(0xFFFF00));
    else lv_led_set_color(shift_leds[i], lv_color_hex(0xFF0000));
    lv_led_off(shift_leds[i]);
  }

  // 2. Central RPM Module
  lv_obj_t* center_cont = lv_obj_create(scr);
  lv_obj_set_size(center_cont, 140, 172);
  lv_obj_set_pos(center_cont, 90, 24);
  lv_obj_set_style_bg_color(center_cont, lv_color_hex(0x111111), 0);
  lv_obj_set_style_border_color(center_cont, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(center_cont, 2, 0);
  lv_obj_set_style_radius(center_cont, 8, 0);
  lv_obj_clear_flag(center_cont, LV_OBJ_FLAG_SCROLLABLE);

  rpm_val_label = lv_label_create(center_cont);
  lv_label_set_text(rpm_val_label, "0");
  lv_obj_set_style_text_font(rpm_val_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(rpm_val_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(rpm_val_label, LV_ALIGN_CENTER, 0, -15);

  lv_obj_t* center_title = lv_label_create(center_cont);
  lv_label_set_text(center_title, "RPM");
  lv_obj_set_style_text_color(center_title, lv_color_hex(0x00FFFF), 0);
  lv_obj_align(center_title, LV_ALIGN_BOTTOM_MID, 0, -15);

  // 3. Peripheral Modules (optimized heights and positions)
  int col_w = 80, row_h = 54, left_x = 5, right_x = 235;
  coolant_val_label = createDataBlock(scr, left_x, 24, col_w, row_h, "TEMP C", lv_color_hex(0xFF8800));
  iat_val_label     = createDataBlock(scr, left_x, 83, col_w, row_h, "IAT C", lv_color_hex(0xFF8800));
  load_val_label    = createDataBlock(scr, left_x, 142, col_w, row_h, "LOAD %", lv_color_hex(0xBF00FF));
  avg_kml_label     = createDataBlock(scr, right_x, 24, col_w, row_h, "AVG KML", lv_color_hex(0x00FF00));
  boost_val_label   = createDataBlock(scr, right_x, 83, col_w, row_h, "BOOST", lv_color_hex(0x00FFFF));
  bat_val_label     = createDataBlock(scr, right_x, 142, col_w, row_h, "BAT V", lv_color_hex(0xFF8800));
  
  // 4. Bottom Bar (thinned out)
  lv_obj_t* bottom_cont = lv_obj_create(scr);
  lv_obj_set_size(bottom_cont, 310, 28);
  lv_obj_set_pos(bottom_cont, 5, 205);
  lv_obj_set_style_bg_color(bottom_cont, lv_color_hex(0x111111), 0);
  lv_obj_set_style_border_color(bottom_cont, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(bottom_cont, 2, 0);
  lv_obj_set_style_pad_all(bottom_cont, 4, 0);
  lv_obj_clear_flag(bottom_cont, LV_OBJ_FLAG_SCROLLABLE);

  load_bar = lv_bar_create(bottom_cont);
  lv_obj_set_size(load_bar, 180, 10);
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

  // 1. Shift Lights
  int shift_start = 1800, shift_end = 3500; 
  float step = (float)(shift_end - shift_start) / 10.0;
  for(int i = 0; i < 10; i++) {
    int threshold = shift_start + (i * step);
    if (currentRpm >= threshold) lv_led_on(shift_leds[i]);
    else lv_led_off(shift_leds[i]);
  }
  if (currentRpm >= shift_end && (millis() / 100) % 2 == 0) {
    for(int i=7; i<10; i++) lv_led_off(shift_leds[i]);
  }

  // 2. Labels
  sprintf(buf, "%d", currentRpm);
  lv_label_set_text(rpm_val_label, buf);

  if (currentAvgKml > 0.0) sprintf(buf, "%.1f", currentAvgKml);
  else sprintf(buf, "--");
  lv_label_set_text(avg_kml_label, buf);

  sprintf(buf, "%.1f", currentBat);
  lv_label_set_text(bat_val_label, buf);

  sprintf(buf, "%d", currentCoolantTemp);
  lv_label_set_text(coolant_val_label, buf);

  sprintf(buf, "%d", currentIat);
  lv_label_set_text(iat_val_label, buf);

  sprintf(buf, "%d", currentLoad);
  lv_label_set_text(load_val_label, buf);

  sprintf(buf, "%.2f", currentBoost);
  lv_label_set_text(boost_val_label, buf);

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

// --- Restored Your Original Robust Scanning Method ---
void connectBluetooth() {
  btConnecting = true;
  updateBTStatus();
  lv_task_handler(); 
  
  esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE; 
  esp_spp_role_t role = ESP_SPP_ROLE_SLAVE; 
  uint8_t obdMac[6] = {0};
  bool found = false;

  if (hasStoredMac && failedDirectConnects < 3) {
    Serial.print("Attempting direct connection to saved MAC: ");
    for(int i=0; i<6; i++) Serial.printf("%02X%s", storedMac[i], (i<5)?":":"\n");
    memcpy(obdMac, storedMac, 6);
    found = true;
  } else {
    if (hasStoredMac) {
      Serial.println("Failed direct connection 3 times. Resetting MAC and scanning...");
    }
    Serial.println("Scanning the air for OBDII...");
    BTScanResults* results = ELM_PORT.discover(5000); // 5 sec scan

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
  }

  if (!found) {
    Serial.println("Couldn't find OBDII in the air! Make sure phone is disconnected.");
    connectedToELM = false;
    btConnecting = false;
    updateBTStatus();
    return;
  }

  Serial.println("Attempting to connect to OBDII via MAC...");
  if (!ELM_PORT.connect(obdMac, 0, sec_mask, role)) {
    Serial.println("Couldn't connect to OBD scanner");
    if (hasStoredMac && failedDirectConnects < 3) {
      failedDirectConnects++;
    } else {
      failedDirectConnects = 0;
    }
    connectedToELM = false;
    btConnecting = false;
    updateBTStatus();
    return;
  }

  Serial.println("Connected to Bluetooth OBDII scanner!");
  
  // Restored original safe clone adapter flags ('0', 20, 0)
  if (!myELM327.begin(ELM_PORT, true, 2000, '0', 20, 0)) {
    Serial.println("Couldn't initialize ELM327");
    connectedToELM = false;
    if (hasStoredMac && failedDirectConnects < 3) {
      failedDirectConnects++;
    }
  } else {
    Serial.println("Connected to ELM327!");
    connectedToELM = true;
    failedDirectConnects = 0;
    if (!hasStoredMac || memcmp(storedMac, obdMac, 6) != 0) {
      preferences.begin("obd_config", false);
      preferences.putBytes("mac", obdMac, 6);
      preferences.putBool("has_mac", true);
      preferences.end();
      memcpy(storedMac, obdMac, 6);
      hasStoredMac = true;
      Serial.println("Saved MAC address to NVS.");
    }
  }
  
  btConnecting = false;
  updateBTStatus();
}

void setup() {
  Serial.begin(115200);
  
  // Setup PWM backlight control
  ledcSetup(PWM_CHANNEL, 5000, 8);
  ledcAttachPin(BACKLIGHT_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 255); // Default to full brightness on boot

  tft.init();
  tft.setRotation(1); 
  tft.invertDisplay(false);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  buildUI();

  // Load Bluetooth MAC address from preferences
  preferences.begin("obd_config", true);
  hasStoredMac = preferences.getBool("has_mac", false);
  if (hasStoredMac) {
    preferences.getBytes("mac", storedMac, 6);
  }
  preferences.end();

  ELM_PORT.begin("ESP32_CYD_Gauge", true); 
  connectBluetooth();
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastTick = 0;
  lv_tick_inc(now - lastTick);
  lastTick = now;
  lv_task_handler();

  // Auto-dimming screen backlight using LDR
  if (now - lastLdrRead > 100) {
    lastLdrRead = now;
    int rawLdr = analogRead(LDR_PIN);
    smoothLdrVal = (0.05f * rawLdr) + (0.95f * smoothLdrVal);
    int duty = map((int)smoothLdrVal, 0, 4095, 15, 255);
    if (duty < 15) duty = 15;
    if (duty > 255) duty = 255;
    ledcWrite(PWM_CHANNEL, duty);
  }

  if (!connectedToELM) {
    static unsigned long lastRetry = 0;
    if (now - lastRetry > 5000) {
      lastRetry = now;
      connectBluetooth();
    }
    return;
  }

  if (now - lastElmUpdate > elmInterval) {
    lastElmUpdate = now;
    
    // Prioritized polling scheduling slots (length 8)
    // Slots 0, 2, 4, 6: RPM (50%)
    // Slots 1, 5: MAP/Boost (25%)
    // Slot 3: Engine Load (12.5%)
    // Slot 7: Slow PIDs (Coolant/IAT/Battery) or KPH (12.5%)
    static int scheduleIndex = 0;
    int currentSlotType = 0; // 0=RPM, 1=MAP, 2=Load, 3=KPH, 4=Coolant, 5=IAT, 6=Battery

    switch(scheduleIndex) {
      case 0: case 2: case 4: case 6:
        currentSlotType = 0; // RPM
        break;
      case 1: case 5:
        currentSlotType = 1; // MAP
        break;
      case 3:
        currentSlotType = 2; // Load
        break;
      case 7: {
        if (now - lastCoolantQuery >= 5000) {
          currentSlotType = 4; // Coolant
        } else if (now - lastIatQuery >= 5000) {
          currentSlotType = 5; // IAT
        } else if (now - lastBatQuery >= 5000) {
          currentSlotType = 6; // Battery
        } else {
          currentSlotType = 3; // KPH
        }
        break;
      }
    }

    bool advancedSlot = false;
    switch(currentSlotType) {
      case 0: {
        float val = myELM327.rpm();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentRpm = (int)val; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) advancedSlot = true;
        break;
      }
      case 1: {
        float val = myELM327.manifoldPressure();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentMap = (int)val; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) advancedSlot = true;
        break;
      }
      case 2: {
        float val = myELM327.engineLoad();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentLoad = (int)val; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) advancedSlot = true;
        break;
      }
      case 3: {
        float val = myELM327.kph();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentKph = (int)val; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) advancedSlot = true;
        break;
      }
      case 4: {
        float val = myELM327.engineCoolantTemp();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentCoolantTemp = (int)val; lastCoolantQuery = now; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { lastCoolantQuery = now; advancedSlot = true; }
        break;
      }
      case 5: {
        float val = myELM327.intakeAirTemp();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentIat = (int)val; lastIatQuery = now; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { lastIatQuery = now; advancedSlot = true; }
        break;
      }
      case 6: {
        float val = myELM327.batteryVoltage();
        if (myELM327.nb_rx_state == ELM_SUCCESS) { currentBat = val; lastBatQuery = now; advancedSlot = true; }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG) { lastBatQuery = now; advancedSlot = true; }
        break;
      }
    }

    if (advancedSlot) {
      scheduleIndex = (scheduleIndex + 1) % 8;
    }

    // --- Diesel Fuel & Boost Calculations ---
    if (currentRpm > 400) { 
      if (lastCalcTime == 0) lastCalcTime = now;
      else {
        float dt = (float)(now - lastCalcTime) / 1000.0;
        lastCalcTime = now;
        if (dt > 0.0 && dt < 2.0) {
          float iatK = (float)currentIat + 273.15;
          
          float maf = ((float)currentRpm * (float)currentMap / iatK) * 0.00185;
          float afr = 50.0 - ((float)currentLoad * 0.32); 
          if (afr < 16.0) afr = 16.0; 
          
          float fuelFlowLps = maf / (afr * DIESEL_DENSITY);
          tripFuel += fuelFlowLps * dt;
          tripDistance += ((float)currentKph / 3600.0) * dt;
          if (tripFuel > 0.0001) currentAvgKml = tripDistance / tripFuel;

          // Convert MAP to Relative Boost (Bar)
          currentBoost = ((float)currentMap - 101.3) / 100.0;
          if (currentBoost < 0.0) currentBoost = 0.0; 
        }
      }
    } else { 
      lastCalcTime = 0; 
      currentBoost = 0.0;
    }

    updateUI();
  }
}