#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include "mbed.h"
#include "rtos.h"
#include <chrono>
#include "ukf.h"
#include "rtwtypes.h"

using namespace mbed;
using namespace rtos;
using namespace std::chrono;
kf Kalmanfilter;

//======= Data Structure=======
struct SensorData {
    uint8_t state=0xBA;//0xBA= Prelaunch, 0xBB= Launch, 0xBC= Deploy, 0xBD= Landed
    float time=0.0;
    float lat = 0.0;
    float lon = 0.0;
    float euler[3]= {0,0,0};
    float accel[3]= {0,0,0};
    float gyro[3]= {0,0,0};
    float pos[3]= {0,0,0};
    float altitude=0.0;
    float pressure = 0.0;
    float kalman[4]= {0,0,0,0};
};

struct __attribute__((packed)) PackedSensorData {
    uint8_t state=0xBA;
    float time=0.0;
    float lat=0.0;
    float lon=0.0;
    float euler[3]= {0,0,0};
    float accel[3]= {0,0,0};
    float gyro[3]= {0,0,0};
    float pos[3]= {0,0,0};
    float altitude=0.0;
    float pressure=0.0;
    float kalman[4]= {0,0,0,0};
};

//==== Sensor Mapping ==========
Adafruit_BMP3XX Barometer;
Servo           Parachute;
HardwareSerial& XBEE = Serial1;
UART            GNSS(PG_14, PG_9, NC, NC);
UART            IMU(PJ_8, PJ_9, NC, NC);

//======= Flight State ========
enum class  FlightState {CALIBRATION, PRELAUNCH, LAUNCH, DEPLOY, LANDED};
Mutex flightStateMutex;
FlightState flightState = FlightState::CALIBRATION;

//======= Parameter ===========
uint32_t        hold                    = 20;       // ms (1000/hold fps)
const float     launchThreshold         = 2;        // g
unsigned long   force_deploy_time       = 14000;    // ms
const float     fallingspeedThreshold   = -7;       // m/s
const float     tiltThreshold           = 45;       // degree



//======= variable ===========
uint32_t        Setup_Time              = 0; // ms
uint32_t        loop_start              = 0; // loop
uint32_t        loop_interval           = 0; // loop
float           basePressure            = 1013.0f;  // hPa

//=========== RTOS ================
Mail<SensorData, 4> xbeeMail;
Mail<SensorData, 4> decisionMail;
Thread              decisionThread;
Thread              xbeeThread;
SensorData          d{};

//=================================================
// Parsing tools
//=================================================
void parseGNSS(SensorData &d) {
  // 한 함수 안에 모든 로직(체크섬/토큰분리/좌표변환) 포함
  static char   nmeaBuf[128];
  static size_t nmeaPos = 0;

  // ---- 로컬 람다: 체크섬 ----
  auto checksumOK = [](const char* s)->bool {
    if (!s || s[0] != '$') return false;
    uint8_t cs = 0; 
    const char* p = s + 1;
    while (*p && *p != '*') cs ^= static_cast<uint8_t>(*p++);
    if (*p != '*') return false;
    auto hex = [](char c)->int{
      if ('0'<=c && c<='9') return c-'0';
      if ('A'<=c && c<='F') return 10 + (c-'A');
      if ('a'<=c && c<='f') return 10 + (c-'a');
      return -1;
    };
    if (!p[1] || !p[2]) return false;
    int hi = hex(p[1]), lo = hex(p[2]);
    if (hi<0 || lo<0) return false;
    return cs == static_cast<uint8_t>((hi<<4)|lo);
  };

  // ---- 로컬 람다: ',' 기준 토큰 분리 ----
  auto splitComma = [](char* line, char** fields, int maxF)->int {
    for (char* q=line; *q; ++q) if (*q=='\r' || *q=='\n') *q = '\0';
    int n = 0;
    char* p = line;
    if (*p=='$') ++p;
    fields[n++] = p;
    while (*p && n < maxF) {
      if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
      else if (*p == '*') { *p = '\0'; break; }
      ++p;
    }
    return n;
  };

  // ---- 로컬 람다: ddmm.mmmm -> degree ----
  auto toDeg = [](const char* v, const char* hemi, bool isLat, float& out)->bool {
    if (!v || !*v || !hemi || !*hemi) return false;
    double raw = strtod(v, nullptr);
    int degPart = static_cast<int>(floor(raw / 100.0));
    double minPart = raw - degPart * 100.0;
    double deg = degPart + (minPart / 60.0);
    char h = toupper(*hemi);
    if ((isLat && h=='S') || (!isLat && h=='W')) deg = -deg;
    if (isLat)  { if (deg < -90.0  || deg > 90.0)  return false; }
    else        { if (deg < -180.0 || deg > 180.0) return false; }
    out = static_cast<float>(deg);
    return true;
  };

  // ---- UART에서 한 줄 읽기 & 파싱 ----
  while (GNSS.available()) {
    char c = GNSS.read();
    //XBEE.write(0xAA);
    //XBEE.print(c);
    if (c == '\r') continue;

    if (c == '\n') {
      nmeaBuf[nmeaPos] = '\0';
      nmeaPos = 0;

      if (nmeaBuf[0] != '$') continue;
      if (!checksumOK(nmeaBuf)) continue;

      // 사본에서 토큰 분리
      char line[128];
      strncpy(line, nmeaBuf, sizeof(line));
      line[sizeof(line)-1] = '\0';

      char* f[24];
      int nf = splitComma(line, f, 24);
      if (nf < 6) continue;

      // ---- RMC: $GNRMC or $GPRMC ----
      if (!strcmp(f[0], "GNRMC") || !strcmp(f[0], "GPRMC")) {
        // $GNRMC, time, status(A/V), lat, NS, lon, EW, ...
        if (nf >= 7 && f[2] && f[2][0]=='A') {
          float lat, lon;
          bool ok1 = toDeg(f[3], f[4], true,  lat);
          bool ok2 = toDeg(f[5], f[6], false, lon);
          if (ok1) d.lat = lat;
          if (ok2) d.lon = lon;
        }
        continue;
      }

      // ---- GGA: $GNGGA or $GPGGA ----
      if (!strcmp(f[0], "GNGGA") || !strcmp(f[0], "GPGGA")) {
        // $GNGGA, time, lat, NS, lon, EW, fixQ, ...
        if (nf >= 6) {
          float lat, lon;
          bool ok1 = toDeg(f[2], f[3], true,  lat);
          bool ok2 = toDeg(f[4], f[5], false, lon);
          if (ok1) d.lat = lat;
          if (ok2) d.lon = lon;
        }
        continue;
      }

      // 다른 문장은 무시
      continue;
    }

    if (nmeaPos < sizeof(nmeaBuf) - 1) nmeaBuf[nmeaPos++] = c;
    else nmeaPos = 0; // 오버플로우 시 리셋
  }
}

void parseIMU(SensorData &d) {
    // 예: *-2.11,4.00,-176.67,-0.38,-1.81,10.21,0.014,-0.033,0.997,-6.827,-0.816,-0.112\n
    char     imuBuf[96];
    size_t   imuPos = 0; 
    while (IMU.available()) {
        char c = IMU.read();
        if (c == '\r') continue;
        if (c == '\n') {
            imuBuf[imuPos] = '\0';   // 문자열 종료
            imuPos = 0;
            if (imuBuf[0] != '*') continue;
            int commaCount = 0;
            for (char* p = imuBuf; *p; ++p) if (*p == ',') ++commaCount;
            if (commaCount != 11) continue;
            float vals[12];
            int   cnt = 0;
            char* s   = imuBuf + 1; // '*' 다음부터
            char* end = s;
            while (cnt < 12) {
                while (*s == ' ' || *s == '\t') ++s;
                vals[cnt] = strtof(s, &end);
                if (s == end) {cnt = -1; break;}
                ++cnt;
                if (*end == ',') {s = end + 1;}
                else if (*end == '\0') {
                    // 문자열 종료에 도달했는데 12개가 아니면 실패
                    if (cnt != 12) cnt = -1;
                    break;
                }
                else {
                    cnt = -1; break;
                }
            }

            if (cnt == 12) {
                d.euler[0] = vals[0];  d.euler[1] = vals[1];  d.euler[2] = vals[2];
                d.gyro[0]  = vals[3];  d.gyro[1]  = vals[4];  d.gyro[2]  = vals[5];
                d.accel[0] = vals[6];  d.accel[1] = vals[7];  d.accel[2] = vals[8];
                d.pos[0]   = vals[9];  d.pos[1]   = vals[10]; d.pos[2]   = vals[11];
                
            }
            continue; // 다음 문자 처리
        }
        if (imuPos < sizeof(imuBuf) - 1) imuBuf[imuPos++] = c;
        else imuPos = 0;
        }
}
 
void parseBaro(SensorData &d){
    if (Barometer.performReading()) {
            d.pressure = Barometer.pressure / 100.0;
            d.altitude = 44330.0f * (1.0f - pow(d.pressure / basePressure, 0.1903f));
    }
}

void parseKalman(SensorData &d){
    Kalmanfilter.rtU.accel_z_in = d.accel[2];
    Kalmanfilter.rtU.baro_altitude_in = d.altitude;
    Kalmanfilter.rtU.current_time_in = d.time / 1000.0;
    Kalmanfilter.step(); // dt=hold로 되어 있음
    d.kalman[0]=Kalmanfilter.rtY.kf_altitude_out;
    d.kalman[1]=Kalmanfilter.rtY.kf_velocity_out;
    d.kalman[2]=Kalmanfilter.rtY.apogee_out;
    d.kalman[3]=Kalmanfilter.rtY.r2_out;
}

//=================================================
// Decision Thread
//=================================================
void Decision() {
    uint32_t Decision_loop_start= 0;
    uint32_t Decision_interval  = 0;
    uint32_t Launch_Time         = 0;
    while(true){
        Decision_loop_start = millis();
        osEvent evt = decisionMail.get();  // mbed os5에서까지만 작동 os6으로 넘어가면 수정 필요
        if (evt.status == osEventMail) {
            SensorData* dmail = (SensorData*)evt.value.p;
            uint32_t Decision_time= dmail->time;
            float euler_x         = dmail->euler[0];
            float euler_y         = dmail->euler[1];
            float z_accel         = dmail->accel[2];
            float Decision_alt    = dmail->altitude;
            float alt_kalman      = dmail->kalman[0];
            float vz_kalman       = dmail->kalman[1];
            float pre_apogee      = dmail->kalman[2];
            float r2_apogee       = dmail->kalman[3];
            decisionMail.free(dmail);
            FlightState localState;
            {
                ScopedLock<Mutex> lock(flightStateMutex);
                localState = flightState;
            }
            switch (localState) {
                default:{
                    if (fabs(z_accel) > launchThreshold or Decision_alt>10.0f) {
                        Launch_Time = Decision_time;
                        flightStateMutex.lock();
                        flightState = FlightState::LAUNCH;
                        flightStateMutex.unlock();
                    }
                } break;
                case FlightState::LAUNCH:{
                    // LAUNCH 수정 필요

                    if (vz_kalman < fallingspeedThreshold) { // -7m/s
                        flightStateMutex.lock();
                        flightState = FlightState::DEPLOY;
                        flightStateMutex.unlock();
                        break;
                    }
                    
                    if (fabs(euler_x) < tiltThreshold || fabs(euler_y) > 180 - tiltThreshold){// euler angle >45'
                        flightStateMutex.lock();
                        flightState = FlightState::DEPLOY;
                        flightStateMutex.unlock();
                        break;
                    } 
                    if (Decision_time - Launch_Time > force_deploy_time){// timer 14s
                        flightStateMutex.lock();
                        flightState = FlightState::DEPLOY;
                        flightStateMutex.unlock();
                    } 
                } break;
                case FlightState::DEPLOY:{
                    Parachute.write(0);//닫힌거
                    ThisThread::sleep_for(milliseconds(500));
                    Parachute.write(180);//열린거
                    ThisThread::sleep_for(milliseconds(2000));
                    if (fabs(Decision_alt) < 2) {
                        flightStateMutex.lock();
                        flightState = FlightState::LANDED;
                        flightStateMutex.unlock();
                    }
                } break;
                case FlightState::LANDED: break;
                
            }
            Decision_interval = millis() - Decision_loop_start;
            if (Decision_interval < hold){
                ThisThread::sleep_for(milliseconds(hold - Decision_interval));
            }
        }
    }
}

//=================================================
// SENDXBEE Thread
//=================================================
void SENDXBEE() {
    PackedSensorData    packed{};
    uint32_t            xbee_loop_start = millis();
    uint32_t            xbee_interval=0;
    uint8_t             frame[1 + sizeof(PackedSensorData) + 1];
    while (true) {
        xbee_loop_start = millis();
        osEvent evt = xbeeMail.get();  // mbed os5에서까지만 작동 os6으로 넘어가면 수정 필요
        if (evt.status == osEventMail) {
            SensorData* xmail = (SensorData*)evt.value.p;
            packed.state    = xmail->state;
            packed.time     = xmail->time;
            packed.lat      = xmail->lat;
            packed.lon      = xmail->lon;
            packed.altitude = xmail->altitude;
            packed.pressure = xmail->pressure;
            for (int i = 0; i < 3; ++i) {
                packed.euler[i]  = xmail->euler[i];
                packed.accel[i]  = xmail->accel[i];
                packed.kalman[i] = xmail->kalman[i];
                packed.gyro[i] = xmail->gyro[i];
                packed.pos[i]  = xmail->pos[i];
            }
            packed.kalman[3] = xmail->kalman[3];
            xbeeMail.free(xmail);
            frame[0] = 0x7E;
            memcpy(&frame[1], &packed, sizeof(PackedSensorData));
            frame[1 + sizeof(PackedSensorData)] = 0x0A;
            XBEE.write(frame, sizeof(frame));
        }
        xbee_interval = millis() - xbee_loop_start;
        if (xbee_interval < hold){
            ThisThread::sleep_for(milliseconds(hold - xbee_interval));
        }
    }
}

//=================================================
// Calibration
//=================================================
void servo_cali(){
    Parachute.write(0);
    delay(1000);   
    Parachute.write(180);//열린거
    delay(1000);
    Parachute.write(0);//닫힌거
    delay(1000);   
}

void imu_cali(){
    XBEE.write(0xAA); 
    XBEE.println("[Calib] IMU accel calibration(10s)... !!DO NOT MOVE!!");
    delay(1000);
    IMU.print("<cg>");
    delay(3000);
    IMU.print("<posz>");
    delay(1000);
    XBEE.write(0xAA); 
    XBEE.println("[Calib] IMU accel calibration finished");

    delay(500);
    XBEE.write(0xAA); 
    XBEE.println("[Calib] Rotate the sensor in all directions for 15 seconds...");
    IMU.print("<cmf>");
    delay(15000);  // 회전 시간 확보 (예: 15초)
    IMU.print(">");
    uint32_t start = millis();
    while (millis() - start < 1000) {
        if (IMU.available()) {
        char c = IMU.read();
        XBEE.write(0xAA);
        XBEE.write(c);
        }
    }
    XBEE.write(0xAA); 
    XBEE.println("[Calib] IMU Cali Finished");
}

void baro_cali(){
    Barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    Barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    Barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    Barometer.setOutputDataRate(BMP3_ODR_200_HZ);
    XBEE.write(0xAA);
    XBEE.print("BAROMETER calibration(6s)...");
    delay(1000);
    float pressureSum = 0;
    float pressureCount = 0;
    float p=0;
    for (int i = 0; i <= 10; i++)   {
        if (Barometer.performReading()){
            p=0;
            p=Barometer.pressure/100;
            if( p>950){
                pressureSum += p;
                pressureCount+=1;
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%.2f hPa", p);
                XBEE.write(0xAA);
                XBEE.println(buffer);
            }
        }
        delay(500);
    }
    if (pressureCount > 0) {
        basePressure = pressureSum / pressureCount;
        XBEE.write(0xAA);
        XBEE.print("[CALI] Calibration complete. Base pressure: ");
        XBEE.write(0xAA);
        XBEE.print(basePressure);
        XBEE.write(0xAA);
        XBEE.println(" hPa");
    }
    else {
        XBEE.write(0xAA);
        XBEE.println("[CALI] Failed: No valid pressure readings");
        //while (1);  // 오류시 정지
    }
}

//=================================================
// Setup
//=================================================
void setup() {
    Parachute.attach(2);
    Serial.begin(115200);
    GNSS.begin(9600);
    IMU.begin(115200);
    XBEE.begin(115200);
    Wire2.begin();
    //pinMode(LED_BUILTIN, OUTPUT);
    delay(1000);
    XBEE.write(0xAA);
    XBEE.println("[IDLE] HELLO_GROUND");  // ping 전송
    Serial.println("[IDLE] HELLO_GROUND");
    if (!Barometer.begin_I2C(0x76, &Wire2)) {
        XBEE.println("[ERROR] Barometer390 not detected");
    }
    
    Kalmanfilter.initialize();
    flightStateMutex.lock();
    flightState = FlightState::CALIBRATION;
    flightStateMutex.unlock();

//============sensor check & cali==================
    servo_cali();
    delay(1000);
    imu_cali();
    delay(1000);
    baro_cali();
    delay(1000);
// ============== Master 명령 대기 루프 =============
    flightStateMutex.lock();
    flightState = FlightState::PRELAUNCH;
    flightStateMutex.unlock();
    XBEE.write(0xAA); 
    XBEE.println("PRELAUNCH");
    Serial.println("PRELAUNCH");
    decisionThread.start(Decision);
    xbeeThread.start(SENDXBEE);
    Setup_Time = millis();
}

//=================================================
// Main Loop
//=================================================
void loop() {
    loop_start = millis();
    parseGNSS(d);
    parseIMU(d);
    d.time = millis() - Setup_Time;
    parseBaro(d);
    parseKalman(d);
    FlightState currentState;
    flightStateMutex.lock();
    currentState = flightState;
    flightStateMutex.unlock();
    switch (currentState) {
                case FlightState::LAUNCH:    d.state = 0xBB; break;
                case FlightState::DEPLOY:    d.state = 0xBC; break;
                case FlightState::LANDED:    d.state = 0xBD; break;
                default:                     d.state = 0xBA; break;
            }


    if (SensorData* xbeedata = xbeeMail.try_alloc()) {
        *xbeedata = d;
        if (xbeeMail.put(xbeedata) != osOK){
                xbeeMail.free(xbeedata);
        }
    }
    if (SensorData* decisiondata = decisionMail.try_alloc()) {
        *decisiondata = d;   
        if (decisionMail.put(decisiondata) != osOK){
            decisionMail.free(decisiondata);
        }
    }
    
    loop_interval = millis() - loop_start;
    if (loop_interval < hold)
        ThisThread::sleep_for(milliseconds(hold - loop_interval));
}