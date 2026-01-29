#include "Adafruit_SHTC3.h"
#include <Wire.h>

// ================= CONFIGURATION =================
#define OTAA_PERIOD   (5000)
#define OTAA_BAND     (RAK_REGION_AS923_2)
#define OTAA_DEVEUI   {0x01, 0xB1, 0x6C, 0x1D, 0x72, 0xF1, 0x1D, 0x0E}
#define OTAA_APPEUI   {0x01, 0xB1, 0x6C, 0x1D, 0x72, 0xF1, 0x1D, 0x0E}
#define OTAA_APPKEY   {0xB5, 0x25, 0x02, 0x1F, 0x03, 0x07, 0xCF, 0xD9, 0x60, 0x31, 0xBA, 0x5C, 0x53, 0xDA, 0xED, 0x28};
const int ZERO_OFFSET = 120; // Nhiễu nền
int ALARM_ON_THRESHOLD = 180; // Ngưỡng Khói


uint8_t TEMP_ALARM_CELSIUS = 50; // Mặc định 50 độ C
int16_t TEMP_THRESHOLD_RAW = TEMP_ALARM_CELSIUS * 100; // Quy đổi ra format x100 (5000)

#define MSG_TYPE_PERIODIC   0x01
#define MSG_TYPE_MANUAL     0x02
#define MSG_TYPE_ALARM      0x03

enum SystemState {
  MONITORING,
  CONFIRMING,
  ALARMING
};

SystemState currentState = MONITORING;

enum SmokeLevel { LEVEL_LOW = 1, LEVEL_MID = 2, LEVEL_HIGH = 3 };

const unsigned long CONFIRMATION_DURATION = 150; 
const unsigned long ALARM_PERIOD = 60000;
const unsigned long PERIODIC_SEND_INTERVAL = 60000; // 60s
const unsigned long HEARTBEAT_INTERVAL = 15000; 

unsigned long confirmation_start_time = 0;
unsigned long alarm_period_start_time = 0;
unsigned long last_periodic_send_time = 0;
unsigned long last_heartbeat_time = 0;

bool button_press_handled = false;
const int PIN_MAT_THU = PA10; 
const int BUZZER_PIN = PA1;
const int USER_BUTTON_PIN = PA8;
const int PIN_LED_PHAT = PA9;
const int LED_PIN = PB4;
const int BATTERY_PIN  = PA15;

uint32_t last_blink_time = 0;
const int BLINK_INTERVAL = 200;
bool is_snoozed = false; 
unsigned long snooze_start_time = 0;
const unsigned long SNOOZE_DURATION = 600000; 
volatile bool is_wakeup_by_button = false;
unsigned long last_join_try = 0;
int join_retry_count = 0;
#define BATVOLT_R1     1000.0f 
#define BATVOLT_R2     2000.0f 
#define REF_VOLTAGE    3.3f

//uint8_t collected_data[64] = { 0 };

// Khởi tạo SHTC3
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
int16_t temperature_val = 0; // Biến lưu nhiệt độ (x100)
uint8_t humidity_val = 0;    // Biến lưu độ ẩm
bool has_shtc3 = false;
bool fire_cause_smoke = false;
bool fire_cause_heat = false;

// ================= CALLBACKS =================
void recvCallback(SERVICE_LORA_RECEIVE_T * data) {
    if (data->BufferSize > 0) {
        Serial.printf("Downlink Received: Port %d, Size %d\r\n", data->Port, data->BufferSize);
        
        //  Command ID
        uint8_t cmd_id = data->Buffer[0];

        switch (cmd_id) {
            // CASE 1: CÀI ĐẶT NGƯỠNG KHÓI (Cần thêm 2 byte dữ liệu)
          
            case 0x01: 
                if (data->BufferSize == 3) {
                    uint16_t new_smoke = (data->Buffer[1] << 8) | data->Buffer[2];
                    ALARM_ON_THRESHOLD = new_smoke;
                    // Lưu vào Flash địa chỉ 0
                    api.system.flash.set(0, (uint8_t*)&new_smoke, 2);
                    Serial.printf(">>> SET SMOKE THRESHOLD: %d\r\n", new_smoke);
                }
                break;

            // CASE 2: CÀI ĐẶT NGƯỠNG NHIỆT ĐỘ (Cần thêm 1 byte dữ liệu)
           
            case 0x02:
                if (data->BufferSize == 2) {
                    uint8_t new_temp = data->Buffer[1];
                    TEMP_ALARM_CELSIUS = new_temp;
                    TEMP_THRESHOLD_RAW = new_temp * 100; // Cập nhật lại biến so sánh
                    // Lưu vào Flash địa chỉ 2
                    api.system.flash.set(2, &new_temp, 1);
                    Serial.printf(">>> SET TEMP THRESHOLD: %d C\r\n", new_temp);
                }
                break;

            // CASE 3: TẮT BÁO ĐỘNG TỪ XA (SILENCE / SNOOZE)
            
            case 0x03:
                Serial.println(">>> CMD: REMOTE SILENCE RECEIVED!");
                // Kích hoạt logic Snooze giống hệt nút bấm
                if (currentState == ALARMING || currentState == CONFIRMING) {
                    is_snoozed = true;
                    snooze_start_time = millis();
                    currentState = MONITORING; // Ép về trạng thái giám sát
                    
                    // Tắt ngay lập tức còi và đèn
                    digitalWrite(BUZZER_PIN, LOW);
                    digitalWrite(LED_PIN, LOW);
                    
                    Serial.println("Alarm Silenced via LoRaWAN!");
                }
                break;

            default:
                Serial.println("Unknown Command ID");
                break;
        }
    }
}

void joinCallback(int32_t status) { Serial.printf("Join status: %d\r\n", status); }
void sendCallback(int32_t status) { 
    if (status == RAK_LORAMAC_STATUS_OK) Serial.println("Sent OK"); 
    else Serial.println("Send Failed"); 
} 
void wakeup_callback() {}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("RAK3172 Dual-Sensor Fire Alarm (Smoke + Heat)");
    
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    pinMode(PIN_LED_PHAT, OUTPUT); digitalWrite(PIN_LED_PHAT, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
    
    // Load Threshold
    uint16_t saved_threshold;
    if (api.system.flash.get(0, (uint8_t*)&saved_threshold, 2)) {
        ALARM_ON_THRESHOLD = saved_threshold;
        Serial.printf("Loaded Smoke Threshold: %d%\r\n",  ALARM_ON_THRESHOLD/10);
    }
uint8_t saved_temp_threshold;
    if (api.system.flash.get(2, &saved_temp_threshold, 1)) {
        TEMP_ALARM_CELSIUS = saved_temp_threshold;
        TEMP_THRESHOLD_RAW = TEMP_ALARM_CELSIUS * 100; // Cập nhật biến phụ
        Serial.printf("Loaded Temp Threshold: %d C\r\n", TEMP_ALARM_CELSIUS);
    } else {
        Serial.printf("Default Temp Threshold: %d C\r\n", TEMP_ALARM_CELSIUS);
    }
    analogReadResolution(12);
    pinMode(USER_BUTTON_PIN, INPUT);
    api.system.sleep.setup(RUI_WAKEUP_RISING_EDGE, USER_BUTTON_PIN);
    api.system.sleep.registerWakeupCallback(wakeup_callback);

    // Init SHTC3
   Wire.begin();
if (!shtc3.begin()) {
    Serial.println("SHTC3 NOT FOUND! Running in SMOKE-ONLY mode.");
    has_shtc3 = false; // Đánh dấu là không có
} else {
    Serial.println("SHTC3 Found & Ready.");
    has_shtc3 = true;  // Đánh dấu là có
}

    // LoRaWAN Setup
    if(api.lorawan.nwm.get() != 1) {
        api.lorawan.nwm.set();
        api.system.reboot();
    }
    uint8_t node_device_eui[8] = OTAA_DEVEUI;
    uint8_t node_app_eui[8] = OTAA_APPEUI;
    uint8_t node_app_key[16] = OTAA_APPKEY;

    api.lorawan.deui.set(node_device_eui, 8);
    api.lorawan.appeui.set(node_app_eui, 8);
    api.lorawan.appkey.set(node_app_key, 16);
    api.lorawan.band.set(OTAA_BAND);
    api.lorawan.njm.set(RAK_LORA_OTAA);
    
    // Join
   Serial.println("Initiating Join Request...");
    api.lorawan.join();
    unsigned long start_wait = millis();
    bool joined_success = false;

    while (millis() - start_wait < 15000) { // Chờ tối đa 15s
        if (api.lorawan.njs.get() == 1) {
            joined_success = true;
            break; // Join thành công! Thoát ngay.
        }
        Serial.print(".");
        delay(1000);
    }

    if (joined_success) {
        Serial.println("\r\n>>> ONLINE MODE: Network Joined Successfully!");
        // Bíp 3 cái báo hiệu đã Online
        digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); delay(100);
        digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); delay(100);
        digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW);
        last_periodic_send_time = millis() - PERIODIC_SEND_INTERVAL - 1000;
    } else {
        Serial.println("\r\n>>> OFFLINE MODE: Gateway unreachable.");
        Serial.println(">>> Device will function as LOCAL ALARM and retry joining later.");
        // Bíp 1 cái dài báo hiệu vào chế độ Offline
        digitalWrite(BUZZER_PIN, HIGH); delay(1000); digitalWrite(BUZZER_PIN, LOW);
    }
    Serial.println("\r\nJoined!");
    
    api.lorawan.adr.set(true);
    api.lorawan.rety.set(1);
    api.lorawan.cfm.set(1);

    api.lorawan.registerRecvCallback(recvCallback);
    api.lorawan.registerJoinCallback(joinCallback);
    api.lorawan.registerSendCallback(sendCallback);
    delay(500);
}

// ================= FUNCTIONS =================

void readSHTC3_Data() {
  if (!has_shtc3) {
        temperature_val = 0;
        humidity_val = 0;
        return; 
    }
    sensors_event_t humidity, temp;
    shtc3.getEvent(&humidity, &temp); // Auto Wake -> Measure -> Sleep
    
    if (!isnan(temp.temperature)) {
        temperature_val = (int16_t)(temp.temperature * 100); 
    }
    if (!isnan(humidity.relative_humidity)) {
        humidity_val = (uint8_t)(humidity.relative_humidity); 
    }
    // Debug
    // Serial.printf("T: %d, H: %d\r\n", temperature_val, humidity_val);
}

int readSmokeSensor() {
    analogRead(PIN_MAT_THU); delayMicroseconds(10);
    int adc_off = analogRead(PIN_MAT_THU);
    digitalWrite(PIN_LED_PHAT, HIGH); delayMicroseconds(320);
    analogRead(PIN_MAT_THU); 
    int adc_on = analogRead(PIN_MAT_THU);
    digitalWrite(PIN_LED_PHAT, LOW);
    
    int raw_delta = adc_on - adc_off;
    int clean_signal = (raw_delta > ZERO_OFFSET) ? (raw_delta - ZERO_OFFSET) : 0;
    return clean_signal;
}

uint8_t getSmokePercentage(int clean_signal) {
    int percent = map(clean_signal, 0, 1000, 0, 100);
    return (percent > 100) ? 100 : percent;
}


uint8_t getSmokeLevel(uint8_t percent) {
    if (percent < 20) return LEVEL_LOW;
    if (percent < 60) return LEVEL_MID;
    return LEVEL_HIGH;
}

uint8_t getBatteryPercentage() {
    int adc_val = analogRead(BATTERY_PIN);
    float voltage = (adc_val / 4095.0f) * REF_VOLTAGE * ((BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2);
    float voltage_real = voltage + 0.30f;
    int percentage = map((long)(voltage_real * 100), 360, 500, 0, 100);///uowwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
    
    return (percentage > 100) ? 100 : (percentage < 0 ? 0 : percentage);
}

void buildAndSendPacket(uint8_t msgType, bool confirmed) {
    // Nếu chưa Join thì thôi
    if (api.lorawan.njs.get() != 1) return;

    // 1. Đọc dữ liệu
    int clean_signal = readSmokeSensor();
    readSHTC3_Data(); 

    uint8_t smoke_pct = getSmokePercentage(clean_signal); 
    uint8_t smoke_lvl = getSmokeLevel(smoke_pct);
    uint8_t bat_pct   = getBatteryPercentage();         

    // 2. KHAI BÁO BIẾN CỤC BỘ 
    uint8_t payload[10]; 
    uint8_t len = 0;

    // 3. Đóng gói
    payload[len++] = msgType;
    payload[len++] = smoke_pct;
    payload[len++] = smoke_lvl;
    payload[len++] = bat_pct;
    payload[len++] = (uint8_t)(temperature_val >> 8);
    payload[len++] = (uint8_t)(temperature_val & 0xFF);
    payload[len++] = humidity_val;

    // 4. Gửi tin và Kiểm tra kết quả
    if (api.lorawan.send(len, payload, 2, confirmed, 1)) {
        Serial.printf("PACKET SENT OK: Type %d, Smoke %d%%, Bat %d%%, Temp%d\r\n", msgType, smoke_pct, bat_pct,temperature_val);
    } else {
        Serial.println(">>> SEND FAILED! (Busy/Duty Cycle Limit or HW Error)");
    }
}
void buildAndSendPacket_Manual(uint8_t msgType, bool confirmed, int current_smoke, int16_t current_temp, uint8_t current_hum, uint8_t manual_bat_pct) {
    if (api.lorawan.njs.get() != 1) return;   
    uint8_t smoke_pct = getSmokePercentage(current_smoke);
    uint8_t smoke_lvl = getSmokeLevel(smoke_pct);
    // uint8_t bat_pct = getBatteryPercentage();
    
    uint8_t payload[10]; 
    uint8_t len = 0;

    payload[len++] = msgType;
    payload[len++] = smoke_pct;
    payload[len++] = smoke_lvl;
    payload[len++] = manual_bat_pct;
    payload[len++] = (uint8_t)(current_temp >> 8);
    payload[len++] = (uint8_t)(current_temp & 0xFF);
    payload[len++] = current_hum;

    api.lorawan.send(len, payload, 2, confirmed, 1);
    Serial.printf("MANUAL SENT: Bat %d%%\r\n", manual_bat_pct);
}
void handleAlarmVisuals() {
    if (currentState == ALARMING) {
        // 1. Còi hú
        digitalWrite(BUZZER_PIN, HIGH);

        // 2. Đèn nháy
        if (millis() - last_blink_time > BLINK_INTERVAL) {
            last_blink_time = millis(); // Cập nhật thời gian
            
            // Đảo trạng thái đèn: Đang Tắt -> Bật, Đang Bật -> Tắt
            int ledState = digitalRead(LED_PIN);
            digitalWrite(LED_PIN, !ledState); 
        }
    } 
    else {
        // Trạng thái bình thường: Tắt hết
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW); 
    }
}

// ================= LOOP =================
void loop() {
    // 1. XỬ LÝ NÚT BẤM
    bool wakeup_event = is_wakeup_by_button;
    is_wakeup_by_button = false;
    bool is_real_press = false;

    // Kiểm tra trạng thái nút vật lý
    if (digitalRead(USER_BUTTON_PIN) == HIGH || wakeup_event) {
        unsigned long press_start = millis();
        bool long_press_detected = false;
        while (digitalRead(USER_BUTTON_PIN) == HIGH) {
            delay(10); 
            
            // Nếu giữ quá 3000ms (3 giây)
            if (millis() - press_start > 3000) {
                Serial.println("\r\n>>> LONG PRESS (3s) DETECTED: SYSTEM REBOOTING...");
                
                // Báo hiệu bằng còi và đèn trước khi Reset
                digitalWrite(BUZZER_PIN, HIGH);
                digitalWrite(LED_PIN, HIGH);
                delay(1000); // Kêu dài 1s
                digitalWrite(BUZZER_PIN, LOW);
                digitalWrite(LED_PIN, LOW);

                // Thực hiện Reset
                api.system.reboot();
                
                long_press_detected = true;
                break; // Thoát vòng lặp
            }
        }

        // Nếu thả tay ra TRƯỚC 3 giây và thời gian nhấn đủ lâu 
        if (!long_press_detected && (millis() - press_start > 150)) {
            is_real_press = true;
        }
    }

    if (is_real_press && !button_press_handled) {
       button_press_handled = true;
       delay(50);
       
       if (currentState == ALARMING) {
           Serial.println("SNOOZE");
           is_snoozed = true;
           snooze_start_time = millis(); 
           currentState = MONITORING; 
           digitalWrite(BUZZER_PIN, LOW);
       } else {
           Serial.println("TEST SEND");
           int clean_smoke = readSmokeSensor();
           if (has_shtc3) readSHTC3_Data();
           uint8_t stable_bat_pct = getBatteryPercentage(); 
           
           // Bíp ngắn xác nhận Test
           digitalWrite(BUZZER_PIN, HIGH); 
           delay(100); 
           digitalWrite(BUZZER_PIN, LOW);
           delay(500);

           if (api.lorawan.njs.get() == 1) {
               buildAndSendPacket_Manual(MSG_TYPE_MANUAL, true, clean_smoke, temperature_val, humidity_val, stable_bat_pct);
           }
       }
    } else if (digitalRead(USER_BUTTON_PIN) == LOW) {
       button_press_handled = false;
    }
if (api.lorawan.njs.get() == 0) {
        unsigned long retry_interval;
        if (join_retry_count < 5) {
             retry_interval = 30000; // 5 lần đầu
        } else if (join_retry_count < 20) {
             retry_interval = 300000;
        } else {
             retry_interval = 3600000; 
        }

        // Kiểm tra đến giờ thử lại
        if (millis() - last_join_try > retry_interval) {
            last_join_try = millis();
            join_retry_count++;
            
            Serial.printf(">>> Background Join Attempt #%d (Interval: %lu ms)\r\n", join_retry_count, retry_interval);
            api.lorawan.join(); 
         
        }
    } else {
        // Nếu đã có mạng, reset biến đếm
        join_retry_count = 0;
    }
    // 2. LOGIC BÁO CHÁY KÉP (SMOKE + HEAT)
    switch (currentState) {
       case MONITORING:
       {
            if (is_snoozed && (millis() - snooze_start_time > SNOOZE_DURATION)) {
                is_snoozed = false;
            }
            int smoke_val = readSmokeSensor();
            bool cond_smoke = (smoke_val > ALARM_ON_THRESHOLD);
            bool cond_heat = false;
            
            // Logic đọc nhiệt độ
            if (has_shtc3) { 
                readSHTC3_Data(); 
                cond_heat = (temperature_val > TEMP_THRESHOLD_RAW);
            }else {
                    cond_heat = false;
                }
            
            // TRƯỜNG HỢP A: cháy hoặc nghi ngờ cháy
            if ((cond_smoke || cond_heat) && !is_snoozed) {
                // 1. Chuyển trạng thái ngay lập tức
                currentState = CONFIRMING;
                confirmation_start_time = millis();
                
                // Lưu nguyên nhân
                fire_cause_smoke = cond_smoke;
                fire_cause_heat = cond_heat;
                
                Serial.println(">>> DETECTED HAZARD! Skipping Heartbeat to verify Fire...");
                break; 
            }

            // TRƯỜNG HỢP B: AN TOÀN -> (Gửi tin)
            // ============================================================
            
            if (millis() - last_periodic_send_time > PERIODIC_SEND_INTERVAL) {
                last_periodic_send_time = millis();
                Serial.println("Safe status confirmed. Sending PAYLOAD...");
                if (api.lorawan.njs.get() == 1) {
                            delay(100); 
                            buildAndSendPacket(MSG_TYPE_PERIODIC, false);
                        }
                
            }

            // Xử lý hiển thị an toàn
            handleAlarmVisuals();
            
            // Ngủ
            if (!is_real_press && digitalRead(USER_BUTTON_PIN) == LOW) {
                if (api.lorawan.njs.get() == 0 && (millis() - last_join_try < 15000)) {
                    Serial.print("."); 
                    delay(1000); 
                } 
                else {
                    // Tình huống bình thường: Ngủ sâu tiết kiệm pin
                    api.system.sleep.all(OTAA_PERIOD); 
                }
            }
       }
            break;
       
        case CONFIRMING:
            {
                
                int smoke_val = readSmokeSensor();
                readSHTC3_Data();

                bool cond_smoke = (smoke_val > ALARM_ON_THRESHOLD);
                bool cond_heat  = (temperature_val > TEMP_THRESHOLD_RAW);

                // Nếu vẫn còn cháy (1 trong 2 điều kiện vẫn đúng)
                if (cond_smoke || cond_heat) {
                    if (millis() - confirmation_start_time > CONFIRMATION_DURATION) {
                        currentState = ALARMING;
                        alarm_period_start_time = millis();
                        
                        Serial.println("!!! FIRE CONFIRMED !!!");
                        if (cond_heat) Serial.println("Reason: HIGH TEMPERATURE");
                        if (cond_smoke) Serial.println("Reason: SMOKE DETECTED");

                        if (api.lorawan.njs.get() == 1) {
                            delay(100); 
                            buildAndSendPacket(MSG_TYPE_ALARM, true);
                        }
                    }
                } else {
                    // Nếu không còn tín hiệu -> Về lại Monitoring (Do nhiễu)
                    currentState = MONITORING;
                    Serial.println("False Alarm. Returning to Monitor.");
                }
            }
            break;

        case ALARMING:
            if (millis() - alarm_period_start_time > ALARM_PERIOD) {
                currentState = MONITORING;
            }
            // Logic gửi lại tin báo cháy (Resend)
            static unsigned long last_alarm_resend = 0;
            if (millis() - last_alarm_resend > 15000) { 
                last_alarm_resend = millis();
                if (api.lorawan.njs.get() == 1) {
                    buildAndSendPacket(MSG_TYPE_ALARM, true); 
                }
            }
            break;
    }

    handleAlarmVisuals();
    if (currentState != MONITORING) delay(200);
}