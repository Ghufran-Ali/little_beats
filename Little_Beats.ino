#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_ADS1X15.h>

// =========================
// I2C / MUX / DEVICE SETUP
// =========================
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const uint8_t TCA_ADDR = 0x70;
static const uint8_t ADS_ADDR = 0x48;

// TCA9548A channels
static const uint8_t CH_ADXL0 = 0;
static const uint8_t CH_ADXL1 = 1;
static const uint8_t CH_ADXL2 = 2;
static const uint8_t CH_ADXL3 = 3;
static const uint8_t CH_ADS1  = 4;
static const uint8_t CH_ADS2  = 5;
static const uint8_t CH_ADS3  = 6;
static const uint8_t CH_ADS4  = 7;

// =========================
// SPEED / TIMING
// =========================
static const uint32_t I2C_FREQ_HZ       = 400000;
static const uint32_t SERIAL_BAUD       = 921600;
static const uint32_t ECG_INTERVAL_US   = 2000;   // 500 Hz target
static const uint32_t ACCEL_INTERVAL_US = 8000;   // 125 Hz target
static const float    ECG_BIAS          = 1.65f;
static const uint16_t SERIAL_QUEUE_LEN  = 512;

// =========================
// OBJECTS
// =========================
Adafruit_ADXL345_Unified accel0(1001);
Adafruit_ADXL345_Unified accel1(1002);
Adafruit_ADXL345_Unified accel2(1003);
Adafruit_ADXL345_Unified accel3(1004);
Adafruit_ADS1115 ads;

// =========================
// SERIAL PACKETS
// Same value format as your upper code:
// - accelerometer in m/s^2
// - ECG in volts after bias subtraction
// =========================
enum PacketType : uint8_t {
  PKT_ECG   = 1,
  PKT_ACCEL = 2
};

struct SerialPacket {
  uint8_t  type;
  uint32_t sampleIndex;
  uint32_t deviceTimeMs;
  float    v[12];
};

QueueHandle_t serialQueue;

// =========================
// HELPERS
// =========================
bool tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  return (Wire.endTransmission() == 0);
}

void initADSContinuous(uint8_t ch) {
  tcaSelect(ch);

  if (!ads.begin(ADS_ADDR)) {
    Serial.printf("ERROR,ADS_INIT_FAIL,CH,%u\n", ch);
    while (1) { delay(1000); }
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);
}

// Return ECG in VOLTS, same style as upper WiFi code
float readADS_V(uint8_t ch) {
  tcaSelect(ch);
  int16_t raw = ads.getLastConversionResults();
  return ads.computeVolts(raw) - ECG_BIAS;
}

// Return accelerometer in m/s^2, same style as upper WiFi code
void readAccelMS2(uint8_t ch,
                  Adafruit_ADXL345_Unified &accel,
                  float &x,
                  float &y,
                  float &z) {
  tcaSelect(ch);

  sensors_event_t event;
  accel.getEvent(&event);

  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
}

// =========================
// SERIAL TASK
// Output format kept separate for logger, but values now match upper code.
// ECG   : ECG,sample,device_time_ms,ecg1,ecg2,ecg3,ecg4
// ACCEL : ACC,sample,device_time_ms,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3
// =========================
void serialTask(void *p) {
  (void)p;

  char line[320];
  SerialPacket pkt;

  while (true) {
    if (xQueueReceive(serialQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      int len = 0;

      if (pkt.type == PKT_ECG) {
        len = snprintf(
          line, sizeof(line),
          "ECG,%lu,%lu,%.5f,%.5f,%.5f,%.5f\n",
          (unsigned long)pkt.sampleIndex,
          (unsigned long)pkt.deviceTimeMs,
          pkt.v[0], pkt.v[1], pkt.v[2], pkt.v[3]
        );
      } else if (pkt.type == PKT_ACCEL) {
        len = snprintf(
          line, sizeof(line),
          "ACC,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
          (unsigned long)pkt.sampleIndex,
          (unsigned long)pkt.deviceTimeMs,
          pkt.v[0],  pkt.v[1],  pkt.v[2],
          pkt.v[3],  pkt.v[4],  pkt.v[5],
          pkt.v[6],  pkt.v[7],  pkt.v[8],
          pkt.v[9],  pkt.v[10], pkt.v[11]
        );
      }

      if (len > 0) {
        Serial.write((const uint8_t *)line, len);
      }
    }
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);

  tcaSelect(CH_ADXL0); accel0.begin(); accel0.setRange(ADXL345_RANGE_2_G);
  tcaSelect(CH_ADXL1); accel1.begin(); accel1.setRange(ADXL345_RANGE_2_G);
  tcaSelect(CH_ADXL2); accel2.begin(); accel2.setRange(ADXL345_RANGE_2_G);
  tcaSelect(CH_ADXL3); accel3.begin(); accel3.setRange(ADXL345_RANGE_2_G);

  initADSContinuous(CH_ADS1);
  initADSContinuous(CH_ADS2);
  initADSContinuous(CH_ADS3);
  initADSContinuous(CH_ADS4);

  serialQueue = xQueueCreate(SERIAL_QUEUE_LEN, sizeof(SerialPacket));

  xTaskCreatePinnedToCore(
    serialTask,
    "serialTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("# ECG_HEADER,sample,device_time_ms,ecg1,ecg2,ecg3,ecg4");
  Serial.println("# ACC_HEADER,sample,device_time_ms,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3");
}

// =========================
// LOOP TIMERS / COUNTERS
// =========================
uint32_t lastECGUs = 0;
uint32_t lastAccelUs = 0;
uint32_t ecgSampleIndex = 0;
uint32_t accelSampleIndex = 0;

// =========================
// LOOP
// =========================
void loop() {
  uint32_t now = micros();

  while ((int32_t)(now - lastECGUs) >= (int32_t)ECG_INTERVAL_US) {
    lastECGUs += ECG_INTERVAL_US;

    SerialPacket pkt = {};
    pkt.type = PKT_ECG;
    pkt.sampleIndex = ecgSampleIndex;
    pkt.deviceTimeMs = ecgSampleIndex * 2UL;

    pkt.v[0] = readADS_V(CH_ADS1);
    pkt.v[1] = readADS_V(CH_ADS2);
    pkt.v[2] = readADS_V(CH_ADS3);
    pkt.v[3] = readADS_V(CH_ADS4);

    xQueueSend(serialQueue, &pkt, 0);
    ecgSampleIndex++;
    now = micros();
  }

  while ((int32_t)(now - lastAccelUs) >= (int32_t)ACCEL_INTERVAL_US) {
    lastAccelUs += ACCEL_INTERVAL_US;

    SerialPacket pkt = {};
    pkt.type = PKT_ACCEL;
    pkt.sampleIndex = accelSampleIndex;
    pkt.deviceTimeMs = accelSampleIndex * 8UL;

    readAccelMS2(CH_ADXL0, accel0, pkt.v[0],  pkt.v[1],  pkt.v[2]);
    readAccelMS2(CH_ADXL1, accel1, pkt.v[3],  pkt.v[4],  pkt.v[5]);
    readAccelMS2(CH_ADXL2, accel2, pkt.v[6],  pkt.v[7],  pkt.v[8]);
    readAccelMS2(CH_ADXL3, accel3, pkt.v[9],  pkt.v[10], pkt.v[11]);

    xQueueSend(serialQueue, &pkt, 0);
    accelSampleIndex++;
    now = micros();
  }
}