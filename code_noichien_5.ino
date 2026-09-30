
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

// Khởi tạo LCD 20x04 (Địa chỉ I2C: 0x27 hoặc 0x3F)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ==================== KHAI BÁO CHÂN KẾT NỐI (GIỮ NGUYÊN) ====================
const int PWM_PIN     = 6;   // PWM LMD18200
const int DIR_PIN     = 4;   // DIR LMD18200
const int BRAKE_PIN   = 5;   // BRAKE LMD18200

const int ENCODER_A   = 2;   // Kênh A Encoder (INT0)
const int ENCODER_B   = 3;   // Kênh B Encoder (INT1)

const int POT_PIN     = A0;  // Biến trở chỉnh tốc độ
const int LM35_PIN    = A1;  // Cảm biến nhiệt độ LM35
const int LED_PIN     = 11;  // LED/Relay điều khiển nhiệt độ
const int LED12_PIN   = 12;  // LED báo trạng thái STOP / DONE / PAUSE / DWELL / WAIT

// 4 Nút nhấn cứng (sử dụng INPUT_PULLUP nội)
const int BT1_PIN     = 10;  // BT1: START / RESUME / NEXT / SAVE / CHỌN
const int BT2_PIN     = 9;   // BT2: PAUSE / STOP / BACK
const int BT3_PIN     = 8;   // BT3: UP (Tăng / Lên)
const int BT4_PIN     = 7;   // BT4: DOWN (Giảm / Xuống)

// ==================== CẤU HÌNH ĐỘNG CƠ & PID ====================
const int PPR = 200;             // Số xung/vòng Encoder
const float MAX_RPM = 1500.0;    // Tốc độ tối đa thực tế
const float ACCEL_STEP = 20.0;   // Dốc tăng tốc (Soft Start)

float Kp = 0.3;  
float Ki = 0.1;  
float Kd = 0.0;  

volatile long pulseCount = 0;
bool targetDirection = true;     // true = THUẬN, false = NGƯỢC
bool isRunning = false;          
bool isPaused = false;           // Cờ báo trạng thái Tạm dừng do người dùng
unsigned long pauseStartTime = 0;// Thời điểm bắt đầu tạm dừng

// ==================== CẤU HÌNH THỜI GIAN NGHỈ ĐẢO CHIỀU ====================
const unsigned long DWELL_TIME_MS = 5000; // Thời gian nghỉ 5 giây giữa các lần đảo chiều
bool isDwelling = false;                  // Cờ báo đang trong 5s nghỉ
unsigned long dwellStartTime = 0;         // Mốc thời gian bắt đầu nghỉ

// ==================== ĐỊNH NGHĨA MÁY TRẠNG THÁI ====================
enum ProcessState { 
  STATE_IDLE, 
  STATE_CHIEN_1, 
  STATE_WAIT_CHIEN_2, 
  STATE_CHIEN_2, 
  STATE_WAIT_LY_TAM, 
  STATE_LY_TAM, 
  STATE_DONE 
};
ProcessState currentState = STATE_IDLE;

enum UiPage { 
  PAGE_RUN, 
  PAGE_CHOOSE_MODE,      // Chọn AUTO / MANUAL
  PAGE_SELECT_PRODUCT,   // Chọn Sản phẩm (AUTO SET)
  PAGE_SELECT_WEIGHT,    // Chọn Khối lượng (AUTO SET)
  PAGE_SET_TEMP1,        // MANUAL SET: Cài nhiệt độ 1
  PAGE_SET_TEMP2,        // MANUAL SET: Cài nhiệt độ 2
  PAGE_SET_CHIEN1,       // MANUAL SET: Cài thời gian chiên 1
  PAGE_SET_CHIEN2,       // MANUAL SET: Cài thời gian chiên 2
  PAGE_SET_LYTAM         // MANUAL SET: Cài thời gian ly tâm
};

UiPage currentPage = PAGE_CHOOSE_MODE; 

// ==================== CẤU HÌNH AUTO SET & CƠ SỞ DỮ LIỆU PROGMEM ====================
const char prod0[] PROGMEM = "Khoai tay";
const char prod1[] PROGMEM = "Bi do";
const char prod2[] PROGMEM = "Hat sen";
const char prod3[] PROGMEM = "Dau bap";
const char* const PROD_NAMES[4] PROGMEM = {prod0, prod1, prod2, prod3};

const char w0[] PROGMEM = "50g";
const char w1[] PROGMEM = "150g";
const char w2[] PROGMEM = "200g";
const char w3[] PROGMEM = "300g";
const char w4[] PROGMEM = "500g";
const char* const WEIGHT_NAMES[5] PROGMEM = {w0, w1, w2, w3, w4};

struct AutoProfile {
  float temp1;
  float temp2;
  unsigned long timeChien1Sec;
  unsigned long timeChien2Sec;
  unsigned long timeLyTamSec;
};

// Đưa mảng AUTO_DATABASE vào bộ nhớ PROGMEM để giải phóng RAM
const AutoProfile AUTO_DATABASE[4][5] PROGMEM = {
  { {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60} },
  { {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60} },
  { {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60} },
  { {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60}, {50.0, 50.0, 60, 60, 60} }
};

uint8_t modeSelection = 0;    // 0: AUTO SET, 1: MANUAL SET
uint8_t productSelection = 0; // 0: Khoai tay, 1: Bi do, 2: Hat sen, 3: Dau bap
uint8_t weightSelection = 0;  // 0: 50g, 1: 150g, 2: 200g, 3: 300g, 4: 500g

// ==================== THAM SỐ CÀI ĐẶT & EEPROM ====================
float tempSet1 = 50.0;               
float tempSet2 = 50.0;               
unsigned long timeChien1SetSec = 60; 
unsigned long timeChien2SetSec = 60; 
unsigned long timeLyTamSetSec  = 60; 

unsigned long timeChien1SetMs  = 60000; 
unsigned long timeChien2SetMs  = 60000; 
unsigned long timeLyTamSetMs   = 60000; 

float tempSet1_tmp, tempSet2_tmp;
unsigned long timeChien1_tmp, timeChien2_tmp, timeLyTam_tmp;

uint8_t timeCursorPos = 1; 

bool blinkState = true;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 50;

const uint16_t EEPROM_MAGIC = 0xCAFE; 
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_DATA  = 2;

struct Settings {
  float temp1;
  float temp2;
  unsigned long timeChien1;
  unsigned long timeChien2;
  unsigned long timeLyTam;
};

// ==================== THỜI GIAN ĐẢO CHIỀU & BỘ ĐẾM ====================
const unsigned long CYCLE_DIR_CHIEN = 3000;  // 3 giây đảo chiều
const unsigned long CYCLE_DIR_LYTAM = 10000; // 10 giây đảo chiều

unsigned long phaseStartTime = 0; 
unsigned long dirSwitchTimer = 0; 

unsigned long lastTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastExcelTime = 0; 

const int SAMPLE_TIME = 50;       // Chu kỳ PID (ms)
const int LCD_REFRESH = 250;      // Chu kỳ làm mới LCD (ms)
const int EXCEL_REFRESH = 5000;   // Chu kỳ xuất dữ liệu Excel (5 giây)

float targetRPM = 0;         
float rampedSetpoint = 0;        
float currentRPM = 0;
float rawRPM = 0;                
float error = 0, lastError = 0;
float integral = 0, derivative = 0;
float pidOutput = 0;
int currentPWM = 0;
float tempC = 0.0;               

// ==================== QUẢN LÝ NÚT NHẤN ====================
unsigned long lastButtonTouch = 0;
const unsigned long TIMEOUT_MENU_MS = 10000; 

struct Button {
  int pin;
  bool lastState;
  bool currentState;
  unsigned long lastDebounceTime;
};

Button bt1 = {BT1_PIN, HIGH, HIGH, 0};
Button bt2 = {BT2_PIN, HIGH, HIGH, 0};
Button bt3 = {BT3_PIN, HIGH, HIGH, 0};
Button bt4 = {BT4_PIN, HIGH, HIGH, 0};

const unsigned long DEBOUNCE_DELAY    = 200;  
const unsigned long AUTO_REPEAT_DELAY = 1000; 
const unsigned long AUTO_REPEAT_RATE  = 100; 

unsigned long comboHoldStartTime = 0;
bool comboHoldTriggered = false;

unsigned long bt1PressStartTime = 0;
bool bt1Holding = false;
bool bt1LongPressTriggered = false;

unsigned long bt3HoldStartTime = 0;
unsigned long bt3LastRepeatTime = 0;
unsigned long bt4HoldStartTime = 0;
unsigned long bt4LastRepeatTime = 0;

// ==================== HÀM QUY ĐỔI VÀ TÍNH THỜI GIAN ====================
void breakTime(unsigned long totalSec, int &h, int &m, int &s) {
  h = totalSec / 3600;
  m = (totalSec % 3600) / 60;
  s = totalSec % 60;
}

unsigned long makeTime(int h, int m, int s) {
  return ((unsigned long)h * 3600UL) + ((unsigned long)m * 60UL) + (unsigned long)s;
}

void lcdPrintTime(unsigned long totalSec) {
  int h, m, s;
  breakTime(totalSec, h, m, s);

  
  if (h < 10) lcd.print(F("0"));
  lcd.print(h);
  lcd.print(F(":"));
  if (m < 10) lcd.print(F("0"));
  lcd.print(m);
  lcd.print(F(":"));
  if (s < 10) lcd.print(F("0"));
  lcd.print(s);
}

void lcdPrintTimeBlink(unsigned long totalSec, uint8_t cursorPos, bool blink) {
  int h, m, s;
  breakTime(totalSec, h, m, s);

  if (cursorPos == 0 && !blink) lcd.print(F("  "));
  else { if (h < 10) lcd.print(F("0")); lcd.print(h); }
  lcd.print(F(":"));

  if (cursorPos == 1 && !blink) lcd.print(F("  "));
  else { if (m < 10) lcd.print(F("0")); lcd.print(m); }
  lcd.print(F(":"));

  if (cursorPos == 2 && !blink) lcd.print(F("  "));
  else { if (s < 10) lcd.print(F("0")); lcd.print(s); }
}

void printProgmemString(const char* const strArray[], uint8_t index) {
  char buffer[16];
  strcpy_P(buffer, (char*)pgm_read_word(&(strArray[index])));
  lcd.print(buffer);
}

// ==================== HÀM TRUYỀN DỮ LIỆU SANG PLX-DAQ ====================
void sendPlxDaqData() {
  Serial.print(F("DATA,TIME,")); 
  
  switch(currentState) {
    case STATE_IDLE:         Serial.print(F("IDLE")); break;
    case STATE_CHIEN_1:      Serial.print(F("CHIEN_1")); break;
    case STATE_WAIT_CHIEN_2: Serial.print(F("WAIT_CHIEN_2")); break;
    case STATE_CHIEN_2:      Serial.print(F("CHIEN_2")); break;
    case STATE_WAIT_LY_TAM:  Serial.print(F("WAIT_LY_TAM")); break;
    case STATE_LY_TAM:       Serial.print(F("LY_TAM")); break;
    case STATE_DONE:         Serial.print(F("HOAN_THANH")); break;
  }
  if (isPaused) Serial.print(F(" (TAM DUNG)"));
  if (isDwelling) Serial.print(F(" (DWELL)"));
  Serial.print(F(","));

  Serial.print(tempC, 1);
  Serial.print(F(","));
  if (currentState == STATE_CHIEN_1) Serial.print(tempSet1, 0);
  else if (currentState == STATE_CHIEN_2) Serial.print(tempSet2, 0);
  else Serial.print(0);
  Serial.print(F(","));

  Serial.print((int)currentRPM);
  Serial.print(F(","));
  Serial.print((int)targetRPM);
  Serial.print(F(","));

  Serial.print(targetDirection ? F("THUAN") : F("NGUOC"));
  Serial.print(F(","));

  Serial.print(currentPWM);
  Serial.print(F(","));

  Serial.println(digitalRead(LED_PIN) ? F("ON") : F("OFF"));
}

// ==================== HÀM XỬ LÝ PHẦN CỨNG & EEPROM ====================
void readEncoderA() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) pulseCount++;
  else pulseCount--;
}

float readLM35() {
  long sumADC = 0;
  for (int i = 0; i < 20; i++) {
    sumADC += analogRead(LM35_PIN);
    delayMicroseconds(100);
  }
  float avgADC = (float)sumADC / 20.0;
  return (avgADC * 500.0) / 1023.0; 
}

void saveSettings() {
  Settings s = { tempSet1, tempSet2, timeChien1SetSec, timeChien2SetSec, timeLyTamSetSec };
  EEPROM.put(EEPROM_ADDR_DATA, s);
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  timeChien1SetMs = timeChien1SetSec * 1000UL;
  timeChien2SetMs = timeChien2SetSec * 1000UL;
  timeLyTamSetMs  = timeLyTamSetSec * 1000UL;
}

void loadSettings() {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    Settings s;
    EEPROM.get(EEPROM_ADDR_DATA, s);
    tempSet1 = s.temp1;
    tempSet2 = s.temp2;
    timeChien1SetSec = s.timeChien1;
    timeChien2SetSec = s.timeChien2;
    timeLyTamSetSec  = s.timeLyTam;
  } else {
    saveSettings();
  }
  timeChien1SetMs = timeChien1SetSec * 1000UL;
  timeChien2SetMs = timeChien2SetSec * 1000UL;
  timeLyTamSetMs  = timeLyTamSetSec * 1000UL;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);

  Serial.println(F("CLEARDATA")); 
  Serial.println(F("LABEL,Thoi Gian,Trang Thai,Nhiet Do (C),Nhiet Do SET (C),RPM Thuc,RPM SET,Chieu Quay,PWM,Gia Nhiet"));

  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  digitalWrite(BRAKE_PIN, HIGH); 

  pinMode(LED_PIN, OUTPUT);     
  digitalWrite(LED_PIN, LOW);   

  pinMode(LED12_PIN, OUTPUT);    
  digitalWrite(LED12_PIN, LOW);  

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  pinMode(BT1_PIN, INPUT_PULLUP);
  pinMode(BT2_PIN, INPUT_PULLUP);
  pinMode(BT3_PIN, INPUT_PULLUP);
  pinMode(BT4_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, RISING);

  loadSettings();
}

// ==================== QUẢN LÝ NÚT NHẤN ====================
bool isButtonPressed(Button &btn) {
  bool reading = digitalRead(btn.pin);

  if (reading != btn.lastState) {
    btn.lastDebounceTime = millis();
    btn.lastState = reading;
  }

  if ((millis() - btn.lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != btn.currentState) {
      btn.currentState = reading;
      if (btn.currentState == LOW) return true;
    }
  }
  return false;
}

void processButtons() {
  unsigned long now = millis();

  bool p2 = isButtonPressed(bt2);
  bool p3 = isButtonPressed(bt3);
  bool p4 = isButtonPressed(bt4);

  bool bt1State = (digitalRead(BT1_PIN) == LOW);
  bool bt3State = (digitalRead(BT3_PIN) == LOW);
  bool bt4State = (digitalRead(BT4_PIN) == LOW);

  // 1. NHẤN GIỮ BT3 + BT4 TRONG 1.5S ĐỂ VÀO MENU CHỌN CHẾ ĐỘ
  if (currentPage == PAGE_RUN && !isRunning && (currentState == STATE_IDLE || currentState == STATE_DONE)) {
    if (bt3State && bt4State) {
      if (comboHoldStartTime == 0) {
        comboHoldStartTime = now;
      } else if (now - comboHoldStartTime >= 1500 && !comboHoldTriggered) {
        comboHoldTriggered = true;
        currentPage = PAGE_CHOOSE_MODE;
        modeSelection = 0; 
        lastButtonTouch = now;
        lcd.clear();
        return;
      }
    } else {
      comboHoldStartTime = 0;
      comboHoldTriggered = false;
    }
  }

  // 2. XỬ LÝ BT1 (NHẤN NGẮN VÀ NHẤN GIỮ)
  if (bt1State && !bt1Holding) {
    bt1Holding = true;
    bt1PressStartTime = now;
    bt1LongPressTriggered = false;
  }

  if (bt1Holding && bt1State) {
    if ((now - bt1PressStartTime >= 1000) && !bt1LongPressTriggered) {
      bt1LongPressTriggered = true;
      if (currentPage == PAGE_SET_CHIEN1 || currentPage == PAGE_SET_CHIEN2 || currentPage == PAGE_SET_LYTAM) {
        timeCursorPos = (timeCursorPos + 1) % 3;
        lastButtonTouch = now;
        lastLcdTime = 0;
      }
    }
  }

  if (!bt1State && bt1Holding) {
    bt1Holding = false;
    if (!bt1LongPressTriggered) { 
      if (currentPage == PAGE_RUN) {
        if (!isRunning && currentState == STATE_IDLE) { 
          isRunning = true;
          isPaused = false;
          isDwelling = false;
          currentState = STATE_CHIEN_1;
          phaseStartTime = now;
          dirSwitchTimer = now;
          targetDirection = true;
          rampedSetpoint = 0;
          digitalWrite(BRAKE_PIN, LOW);
          integral = 0;
          lcd.clear();
        } 
        else if (currentState == STATE_WAIT_CHIEN_2) { 
          currentState = STATE_CHIEN_2;
          isRunning = true;
          isPaused = false;
          isDwelling = false;
          phaseStartTime = now;
          dirSwitchTimer = now;
          targetDirection = true;
          rampedSetpoint = 0;
          digitalWrite(BRAKE_PIN, LOW);
          integral = 0;
          lcd.clear();
        }
        else if (currentState == STATE_WAIT_LY_TAM) { 
          currentState = STATE_LY_TAM;
          isRunning = true;
          isPaused = false;
          isDwelling = false;
          phaseStartTime = now;
          dirSwitchTimer = now;
          targetDirection = true;
          rampedSetpoint = 0;
          digitalWrite(BRAKE_PIN, LOW);
          integral = 0;
          lcd.clear();
        }
        else if (isPaused) { 
          isPaused = false;
          unsigned long pauseDuration = now - pauseStartTime;
          phaseStartTime += pauseDuration; 
          dirSwitchTimer += pauseDuration;
          if (isDwelling) dwellStartTime += pauseDuration;
          digitalWrite(BRAKE_PIN, LOW);
          lcd.clear();
        }
      } 
      else { 
        lastButtonTouch = now;

        if (currentPage == PAGE_CHOOSE_MODE) {
          if (modeSelection == 0) { 
            currentPage = PAGE_SELECT_PRODUCT;
            productSelection = 0;
          } else { 
            currentPage = PAGE_SET_TEMP1;
            tempSet1_tmp = tempSet1;
            tempSet2_tmp = tempSet2;
            timeChien1_tmp = timeChien1SetSec;
            timeChien2_tmp = timeChien2SetSec;
            timeLyTam_tmp  = timeLyTamSetSec;
            timeCursorPos = 1;
          }
        }
        else if (currentPage == PAGE_SELECT_PRODUCT) {
          currentPage = PAGE_SELECT_WEIGHT;
          weightSelection = 0;
        }
        else if (currentPage == PAGE_SELECT_WEIGHT) {
          AutoProfile prof;
          memcpy_P(&prof, &AUTO_DATABASE[productSelection][weightSelection], sizeof(AutoProfile));
          tempSet1 = prof.temp1;
          tempSet2 = prof.temp2;
          timeChien1SetSec = prof.timeChien1Sec;
          timeChien2SetSec = prof.timeChien2Sec;
          timeLyTamSetSec  = prof.timeLyTamSec;
          saveSettings();
          
          currentState = STATE_IDLE;
          currentPage = PAGE_RUN; 
        }
        else if (currentPage == PAGE_SET_TEMP1) {
          currentPage = PAGE_SET_TEMP2;
        } 
        else if (currentPage == PAGE_SET_TEMP2) {
          currentPage = PAGE_SET_CHIEN1;
          timeCursorPos = 1;
        } 
        else if (currentPage == PAGE_SET_CHIEN1) {
          currentPage = PAGE_SET_CHIEN2;
          timeCursorPos = 1;
        } 
        else if (currentPage == PAGE_SET_CHIEN2) {
          currentPage = PAGE_SET_LYTAM;
          timeCursorPos = 1;
        } 
        else if (currentPage == PAGE_SET_LYTAM) {
          tempSet1 = tempSet1_tmp;
          tempSet2 = tempSet2_tmp;
          timeChien1SetSec = timeChien1_tmp;
          timeChien2SetSec = timeChien2_tmp;
          timeLyTamSetSec  = timeLyTam_tmp;
          saveSettings();
          
          currentState = STATE_IDLE;
          currentPage = PAGE_RUN;
        }
        lcd.clear();
      }
    }
  }

  // 3. XỬ LÝ BT2: PAUSE / STOP HOẶC BACK TRANG TRƯỚC ĐÓ
  if (p2) {
    if (currentPage == PAGE_RUN) {
      if (isRunning || currentState == STATE_WAIT_CHIEN_2 || currentState == STATE_WAIT_LY_TAM) {
        if (!isPaused && currentState != STATE_WAIT_CHIEN_2 && currentState != STATE_WAIT_LY_TAM) {
          isPaused = true;
          pauseStartTime = now;
          analogWrite(PWM_PIN, 0);
          digitalWrite(BRAKE_PIN, HIGH);
          currentPWM = 0;
          rampedSetpoint = 0;
          lcd.clear();
        } else {
          isRunning = false;
          isPaused = false;
          isDwelling = false;
          currentState = STATE_IDLE;
          analogWrite(PWM_PIN, 0);
          digitalWrite(BRAKE_PIN, HIGH);
          digitalWrite(LED_PIN, LOW);
          currentPWM = 0;
          rampedSetpoint = 0;
          integral = 0;
          lcd.clear();
        }
      }
    } else { 
      lastButtonTouch = now;
      if (currentPage == PAGE_CHOOSE_MODE) {
        currentState = STATE_IDLE;
        currentPage = PAGE_RUN;
      }
      else if (currentPage == PAGE_SELECT_PRODUCT) currentPage = PAGE_CHOOSE_MODE;
      else if (currentPage == PAGE_SELECT_WEIGHT)  currentPage = PAGE_SELECT_PRODUCT;
      else if (currentPage == PAGE_SET_TEMP1)       currentPage = PAGE_CHOOSE_MODE;
      else if (currentPage == PAGE_SET_TEMP2)       currentPage = PAGE_SET_TEMP1;
      else if (currentPage == PAGE_SET_CHIEN1)      currentPage = PAGE_SET_TEMP2;
      else if (currentPage == PAGE_SET_CHIEN2)      currentPage = PAGE_SET_CHIEN1;
      else if (currentPage == PAGE_SET_LYTAM)       currentPage = PAGE_SET_CHIEN2;

      lcd.clear();
      return;
    }
  }

  // 4. XỬ LÝ BT3 (UP) VÀ BT4 (DOWN) TRONG MENU CÀI ĐẶT
  if (currentPage != PAGE_RUN) {
    bool doIncrement = false;
    bool doDecrement = false;

    if (p3) {
      doIncrement = true;
      bt3HoldStartTime = now;
      bt3LastRepeatTime = now;
    } else if (bt3State && !bt4State) {
      if (now - bt3HoldStartTime >= AUTO_REPEAT_DELAY) {
        if (now - bt3LastRepeatTime >= AUTO_REPEAT_RATE) {
          doIncrement = true;
          bt3LastRepeatTime = now;
        }
      }
    } else {
      bt3HoldStartTime = 0;
    }

    if (p4) {
      doDecrement = true;
      bt4HoldStartTime = now;
      bt4LastRepeatTime = now;
    } else if (bt4State && !bt3State) {
      if (now - bt4HoldStartTime >= AUTO_REPEAT_DELAY) {
        if (now - bt4LastRepeatTime >= AUTO_REPEAT_RATE) {
          doDecrement = true;
          bt4LastRepeatTime = now;
        }
      }
    } else {
      bt4HoldStartTime = 0;
    }

    if (doIncrement || doDecrement) {
      lastButtonTouch = now; 

      switch (currentPage) {
        case PAGE_CHOOSE_MODE:
          if (doIncrement || doDecrement) modeSelection = (modeSelection == 0) ? 1 : 0;
          break;

        case PAGE_SELECT_PRODUCT:
          if (doIncrement) productSelection = (productSelection == 0) ? 3 : productSelection - 1;
          if (doDecrement) productSelection = (productSelection + 1) % 4;
          break;

        case PAGE_SELECT_WEIGHT:
          if (doIncrement) weightSelection = (weightSelection == 0) ? 4 : weightSelection - 1;
          if (doDecrement) weightSelection = (weightSelection + 1) % 5;
          break;

        case PAGE_SET_TEMP1:
          tempSet1_tmp = constrain(tempSet1_tmp + (doIncrement ? 1 : -1), 0.0, 150.0);
          break;

        case PAGE_SET_TEMP2:
          tempSet2_tmp = constrain(tempSet2_tmp + (doIncrement ? 1 : -1), 0.0, 150.0);
          break;

        case PAGE_SET_CHIEN1:
        case PAGE_SET_CHIEN2:
        case PAGE_SET_LYTAM: {
          unsigned long *targetTime = (currentPage == PAGE_SET_CHIEN1) ? &timeChien1_tmp : 
                                      ((currentPage == PAGE_SET_CHIEN2) ? &timeChien2_tmp : &timeLyTam_tmp);
          int delta = doIncrement ? 1 : -1;
          int h, m, s;
          breakTime(*targetTime, h, m, s);

          if (timeCursorPos == 0)      h = constrain(h + delta, 0, 23);
          else if (timeCursorPos == 1) m = constrain(m + delta, 0, 59);
          else if (timeCursorPos == 2) s = constrain(s + delta, 0, 59);

          *targetTime = makeTime(h, m, s);
          break;
        }
        default: break;
      }
      lastLcdTime = 0; 
    }

    if ((now - lastButtonTouch >= TIMEOUT_MENU_MS) && isRunning) {
      currentPage = PAGE_RUN; 
      lcd.clear();
    }
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();

  processButtons();

  // ĐIỀU KHIỂN LED D12 BÁO TRẠNG THÁI
  if (currentState == STATE_DONE || (currentState == STATE_IDLE && !isRunning) || 
      currentState == STATE_WAIT_CHIEN_2 || currentState == STATE_WAIT_LY_TAM || isPaused || isDwelling) {
    digitalWrite(LED12_PIN, HIGH);
  } else {
    digitalWrite(LED12_PIN, LOW);
  }

  // LOGIC CHUYỂN GIAI ĐOẠN & DWELLING 5S
  if (isRunning && !isPaused) {
    if (isDwelling) {
      if (currentTime - dwellStartTime >= DWELL_TIME_MS) {
        isDwelling = false;
        targetDirection = !targetDirection;
        dirSwitchTimer = currentTime;
        rampedSetpoint = 0; 
        integral = 0;
      }
    } else {
      if (currentState == STATE_CHIEN_1 || currentState == STATE_CHIEN_2) {
        if (currentTime - dirSwitchTimer >= CYCLE_DIR_CHIEN) {
          isDwelling = true;
          dwellStartTime = currentTime;
        }
      } else if (currentState == STATE_LY_TAM) {
        if (currentTime - dirSwitchTimer >= CYCLE_DIR_LYTAM) {
          isDwelling = true;
          dwellStartTime = currentTime;
        }
      }
    }

    // KIỂM TRA HẾT GIỜ TỪNG CÔNG ĐOẠN
    if (currentState == STATE_CHIEN_1) {
      if (currentTime - phaseStartTime >= timeChien1SetMs) {
        currentState = STATE_WAIT_CHIEN_2;
        isRunning = false;
        isDwelling = false;
        analogWrite(PWM_PIN, 0);
        digitalWrite(BRAKE_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
        currentPWM = 0;
        integral = 0;
        lcd.clear();
      }
    } 
    else if (currentState == STATE_CHIEN_2) {
      if (currentTime - phaseStartTime >= timeChien2SetMs) {
        currentState = STATE_WAIT_LY_TAM;
        isRunning = false;
        isDwelling = false;
        analogWrite(PWM_PIN, 0);
        digitalWrite(BRAKE_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
        currentPWM = 0;
        integral = 0;
        lcd.clear();
      }
    }
    else if (currentState == STATE_LY_TAM) {
      if (currentTime - phaseStartTime >= timeLyTamSetMs) {
        isRunning = false;
        currentState = STATE_DONE;
        isDwelling = false;
        analogWrite(PWM_PIN, 0);
        digitalWrite(BRAKE_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
        currentPWM = 0;
        integral = 0;
        lcd.clear();
      }
    }
  }

  // ĐỌC BIẾN TRỞ TỐC ĐỘ
  int potValue = analogRead(POT_PIN);
  if (potValue < 20) potValue = 0;
  else if (potValue > 1000) potValue = 1023;
  targetRPM = map(potValue, 0, 1023, 0, MAX_RPM);

  // ĐIỀU KHIỂN PID MOTOR (CHU KỲ 50MS)
  unsigned long elapsedTime = currentTime - lastTime;
  if (elapsedTime >= SAMPLE_TIME) {
    noInterrupts();
    long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float dt_sec = elapsedTime / 1000.0;
    rawRPM = (abs((float)pulses) / PPR) * (60.0 / dt_sec);
    currentRPM = currentRPM * 0.7 + rawRPM * 0.3;

    if (isRunning && !isPaused && !isDwelling) {
      digitalWrite(BRAKE_PIN, LOW);

      if (rampedSetpoint < targetRPM) {
        rampedSetpoint += ACCEL_STEP;
        if (rampedSetpoint > targetRPM) rampedSetpoint = targetRPM;
      } else if (rampedSetpoint > targetRPM) {
        rampedSetpoint -= ACCEL_STEP;
        if (rampedSetpoint < targetRPM) rampedSetpoint = targetRPM;
      }

      error = rampedSetpoint - currentRPM;
      integral += error * dt_sec;
      integral = constrain(integral, -100.0, 100.0); 
      derivative = (error - lastError) / dt_sec;

      float basePWM = (rampedSetpoint / MAX_RPM) * 255.0;
      pidOutput = basePWM + (Kp * error) + (Ki * integral) + (Kd * derivative);
      lastError = error;

      currentPWM = constrain((int)pidOutput, 0, 255);
      digitalWrite(DIR_PIN, targetDirection ? HIGH : LOW);
      analogWrite(PWM_PIN, currentPWM);
    } else {
      analogWrite(PWM_PIN, 0);
      digitalWrite(BRAKE_PIN, HIGH);
      currentPWM = 0;
      rampedSetpoint = 0;
    }
    lastTime = currentTime;
  }

  // GỬI DỮ LIỆU SANG PLX-DAQ CỐ ĐỊNH 5 GIÂY 1 LẦN
  if (currentTime - lastExcelTime >= EXCEL_REFRESH) {
    if (currentPage == PAGE_RUN) {
      sendPlxDaqData();
    }
    lastExcelTime = currentTime;
  }

  // HIỂN THỊ LCD 20X04 & GIA NHIỆT (CHU KỲ 250MS)
  if (currentTime - lastLcdTime >= LCD_REFRESH) {
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;
    }

    tempC = readLM35();

    // LOGIC GIA NHIỆT
    if (isRunning && currentState == STATE_CHIEN_1) {
      digitalWrite(LED_PIN, (tempC <= tempSet1) ? HIGH : LOW);
    } else if (isRunning && currentState == STATE_CHIEN_2) {
      digitalWrite(LED_PIN, (tempC <= tempSet2) ? HIGH : LOW);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    switch (currentPage) {
      case PAGE_RUN:
        if (isPaused) {
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print((char)223); lcd.print(F("C [TAM DUNG]   "));
          lcd.setCursor(0, 1); lcd.print(F("ST: DANG DUNG      "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  PWM:")); lcd.print(currentPWM); lcd.print(F("   "));
        } 
        else if (currentState == STATE_IDLE) {
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print(F("C S1:")); lcd.print((int)tempSet1); lcd.print(F("C S2:")); lcd.print((int)tempSet2); lcd.print(F("C"));
          lcd.setCursor(0, 1); lcd.print(F("TT: Ready Start     "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  PWM:")); lcd.print(currentPWM); lcd.print(F("   "));
        } 
        else if (currentState == STATE_CHIEN_1) {
          long remainSec = (long)(timeChien1SetMs - (currentTime - phaseStartTime)) / 1000;
          if (remainSec < 0) remainSec = 0;
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print((char)223); lcd.print(F("C  SET1: ")); lcd.print((int)tempSet1); lcd.print((char)223); lcd.print(F("C  "));
          lcd.setCursor(0, 1); lcd.print(F("CHIEN 1: ")); lcdPrintTime((unsigned long)remainSec); lcd.print(F("   "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  PWM:")); lcd.print(currentPWM); lcd.print(F("   "));
        } 
        else if (currentState == STATE_WAIT_CHIEN_2) {
          lcd.setCursor(0, 0); lcd.print(F("DA CHIEN 1 XONG!    "));
          lcd.setCursor(0, 1); lcd.print(F("Nhan BT1 -> Chien 2 "));
          lcd.setCursor(0, 2); lcd.print(F("Temp2 Set: ")); lcd.print((int)tempSet2); lcd.print((char)223); lcd.print(F("C     "));
          lcd.setCursor(0, 3); lcd.print(F("--------------------"));
        }
        else if (currentState == STATE_CHIEN_2) {
          long remainSec = (long)(timeChien2SetMs - (currentTime - phaseStartTime)) / 1000;
          if (remainSec < 0) remainSec = 0;
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print((char)223); lcd.print(F("C  SET2: ")); lcd.print((int)tempSet2); lcd.print((char)223); lcd.print(F("C  "));
          lcd.setCursor(0, 1); lcd.print(F("CHIEN 2: ")); lcdPrintTime((unsigned long)remainSec); lcd.print(F("   "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  PWM:")); lcd.print(currentPWM); lcd.print(F("   "));
        }
        else if (currentState == STATE_WAIT_LY_TAM) {
          lcd.setCursor(0, 0); lcd.print(F("DA CHIEN 2 XONG!    "));
          lcd.setCursor(0, 1); lcd.print(F("Nhan BT1 -> Ly Tam  "));
          lcd.setCursor(0, 2); lcd.print(F("Time Set: ")); lcdPrintTime(timeLyTamSetSec);
          lcd.setCursor(0, 3); lcd.print(F("--------------------"));
        }
        else if (currentState == STATE_LY_TAM) {
          long remainSec = (long)(timeLyTamSetMs - (currentTime - phaseStartTime)) / 1000;
          if (remainSec < 0) remainSec = 0;
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print((char)223); lcd.print(F("C  LY TAM     "));
          lcd.setCursor(0, 1); lcd.print(F("LYTAM  : ")); lcdPrintTime((unsigned long)remainSec); lcd.print(F("   "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  PWM:")); lcd.print(currentPWM); lcd.print(F("   "));
        } 
        else if (currentState == STATE_DONE) {
          lcd.setCursor(0, 0); lcd.print(F("T:")); lcd.print(tempC, 1); lcd.print((char)223); lcd.print(F("C  HOAN THANH!"));
          lcd.setCursor(0, 1); lcd.print(F("ST : HOAN THANH!    "));
          lcd.setCursor(0, 2); lcd.print(F("RPM: ")); lcd.print((int)currentRPM); lcd.print(F("/")); lcd.print((int)targetRPM); lcd.print(F(" RPM    "));
          lcd.setCursor(0, 3); lcd.print(F("DIR: ")); lcd.print(targetDirection ? F("THUAN") : F("NGUOC")); lcd.print(F("  P:")); lcd.print(currentPWM); lcd.print(F("   "));
        }
        break;

      case PAGE_CHOOSE_MODE:
        lcd.setCursor(0, 0); lcd.print(F("== CHON CHE DO CD =="));
        lcd.setCursor(0, 1); lcd.print(modeSelection == 0 ? F(" > AUTO SET         ") : F("   AUTO SET         "));
        lcd.setCursor(0, 2); lcd.print(modeSelection == 1 ? F(" > MANUAL SET       ") : F("   MANUAL SET       "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Exit BT1:Select]"));
        break;

      case PAGE_SELECT_PRODUCT:
        lcd.setCursor(0, 0); lcd.print(F("CHON SAN PHAM CHIEN:"));
        for (int i = 0; i < 3; i++) {
          uint8_t itemIdx = (productSelection < 3) ? i : (i + 1);
          lcd.setCursor(0, i + 1);
          if (itemIdx == productSelection) lcd.print(F(" > "));
          else lcd.print(F("   "));
          printProgmemString(PROD_NAMES, itemIdx);
          lcd.print(F("          "));
        }
        break;

      case PAGE_SELECT_WEIGHT:
        lcd.setCursor(0, 0); lcd.print(F("  CHON KHOI LUONG:  "));
        for (int i = 0; i < 3; i++) {
          uint8_t itemIdx = (weightSelection < 3) ? i : (i + 1);
          if (weightSelection == 4) itemIdx = i + 2;
          lcd.setCursor(0, i + 1);
          if (itemIdx == weightSelection) lcd.print(F(" > "));
          else lcd.print(F("   "));
          printProgmemString(WEIGHT_NAMES, itemIdx);
          lcd.print(F("          "));
        }
        break;

      case PAGE_SET_TEMP1:
        lcd.setCursor(0, 0); lcd.print(F("== NHIET DO SET 1 =="));
        lcd.setCursor(0, 1); lcd.print(F("                    "));
        lcd.setCursor(0, 2); lcd.print(F("Gia tri: > ")); lcd.print((int)tempSet1_tmp); lcd.print((char)223); lcd.print(F("C <  "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Back BT1:Next] "));
        break;

      case PAGE_SET_TEMP2:
        lcd.setCursor(0, 0); lcd.print(F("== NHIET DO SET 2 =="));
        lcd.setCursor(0, 1); lcd.print(F("                    "));
        lcd.setCursor(0, 2); lcd.print(F("Gia tri: > ")); lcd.print((int)tempSet2_tmp); lcd.print((char)223); lcd.print(F("C <  "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Back BT1:Next] "));
        break;

      case PAGE_SET_CHIEN1:
        lcd.setCursor(0, 0); lcd.print(F("==THOI GIAN CHIEN 1="));
        lcd.setCursor(0, 1); lcd.print(F(" Gio : Phut : Giay "));
        lcd.setCursor(0, 2); lcd.print(F("     "));
        lcdPrintTimeBlink(timeChien1_tmp, timeCursorPos, blinkState);
        lcd.print(F("       "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Back BT1:Next]"));
        break;

      case PAGE_SET_CHIEN2:
        lcd.setCursor(0, 0); lcd.print(F("==THOI GIAN CHIEN 2="));
        lcd.setCursor(0, 1); lcd.print(F(" Gio : Phut : Giay "));
        lcd.setCursor(0, 2); lcd.print(F("     "));
        lcdPrintTimeBlink(timeChien2_tmp, timeCursorPos, blinkState);
        lcd.print(F("       "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Back BT1:Next]"));
        break;

      case PAGE_SET_LYTAM:
        lcd.setCursor(0, 0); lcd.print(F("==THOI GIAN LY TAM=="));
        lcd.setCursor(0, 1); lcd.print(F(" Gio : Phut : Giay "));
        lcd.setCursor(0, 2); lcd.print(F("     "));
        lcdPrintTimeBlink(timeLyTam_tmp, timeCursorPos, blinkState);
        lcd.print(F("       "));
        lcd.setCursor(0, 3); lcd.print(F("[BT2:Back BT1:Save]"));
        break;
    }
    lastLcdTime = currentTime;
  }
}

//aaaaaaaaaaaaaaaaaaaaaaaaa