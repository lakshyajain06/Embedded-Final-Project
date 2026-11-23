#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

#include "MotionDetector.h"

namespace {
constexpr uint8_t imuAddress = 0x6A;
constexpr uint32_t samplePeriodUs = 1000000 / sampleRateHz;
constexpr uint32_t alertDurationMs = 10000;
constexpr size_t noDump = windowSamples + 1;

BLEUart bleUart;
MotionDetector detector;
MotionSample eventWindow[windowSamples];
uint32_t nextSampleUs;
uint32_t eventTimeMs;
uint32_t nextBleAttemptMs;
uint32_t eventNumber = 0;
size_t dumpRow = noDump;
bool pendingNotification = false;
bool alertActive = false;
bool imuReady = false;
bool bleReady = false;
unsigned invalidSamples = 0;
uint32_t validSamples = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(imuAddress);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
  Wire1.beginTransmission(imuAddress);
  Wire1.write(reg);
  if (Wire1.endTransmission(length == 1) != 0 ||
      Wire1.requestFrom(imuAddress, length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) data[i] = Wire1.read();
  return true;
}

bool startImu() {
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  // The Sense powers its IMU from a GPIO. Use the drive setting in Seeed's driver.
  nrf_gpio_cfg(g_ADigitalPinMap[PIN_LSM6DS3TR_C_POWER], NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(20);
  Wire1.begin();
  uint8_t id;
  return readRegisters(0x0F, &id, 1) && id == 0x6A &&
         writeRegister(0x12, 0x44) &&  // Block data updates, auto-increment.
         writeRegister(0x10, 0x4C) &&  // Accel: 104 Hz, ±8 g.
         writeRegister(0x11, 0x48);    // Gyro: 104 Hz, ±1000 degrees/s.
}

int16_t readLe16(const uint8_t* bytes) {
  return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
}

bool readMotion(MotionSample& sample) {
  uint8_t data[12];
  if (!readRegisters(0x22, data, sizeof(data))) return false;
  for (size_t axis = 0; axis < 3; ++axis) {
    sample.gyro[axis] = readLe16(data + axis * 2);
    sample.accel[axis] = readLe16(data + 6 + axis * 2);
    if (abs(sample.accel[axis]) >= 32760 || abs(sample.gyro[axis]) >= 32760) {
      return false;
    }
  }
  return true;
}

bool startBle() {
  if (!Bluefruit.begin()) return false;
  Bluefruit.setName("SynapSense");
  Bluefruit.autoConnLed(false);
  bleUart.begin();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  return Bluefruit.Advertising.start(0);
}

void recordEvent(uint32_t nowMs) {
  for (size_t i = 0; i < windowSamples; ++i) eventWindow[i] = detector.at(i);
  dumpRow = noDump;
  eventTimeMs = nowMs;
  ++eventNumber;
  pendingNotification = true;
  nextBleAttemptMs = nowMs;
  alertActive = true;
}

void serviceNotification(uint32_t nowMs) {
  if (!pendingNotification || !Bluefruit.connected() || !bleUart.notifyEnabled() ||
      static_cast<int32_t>(nowMs - nextBleAttemptMs) < 0) return;
  nextBleAttemptMs = nowMs + 250;
  char message[21];  // One notification even at the default 23-byte ATT MTU.
  const int length = snprintf(message, sizeof(message), "ALERT t=%lums",
                              static_cast<unsigned long>(eventTimeMs));
  if (bleUart.write(reinterpret_cast<const uint8_t*>(message), length) ==
      static_cast<size_t>(length)) {
    pendingNotification = false;
  }
}

void serviceSerial() {
  if (!Serial) {
    dumpRow = noDump;
    return;
  }
  const int command = Serial.read();
  if (command == 's') {
    Serial.print(imuReady ? "IMU OK" : "IMU init failed");
    Serial.println(bleReady ? ", BLE started" : ", BLE start failed");
  }
  if (command == 'm') {
    const MotionFeatures& motion = detector.features();
    Serial.print("samples=");
    Serial.print(validSamples);
    Serial.print(", accel_g=");
    Serial.print(motion.accelRms, 2);
    Serial.print(", gyro_dps=");
    Serial.print(motion.gyroRms, 1);
    Serial.print(", correlation=");
    Serial.println(motion.correlation, 2);
  }
  if (command == 't') {
    const char message[] = "TEST ALERT";
    if (Bluefruit.connected() && bleUart.notifyEnabled() &&
        bleUart.write(reinterpret_cast<const uint8_t*>(message), sizeof(message) - 1)) {
      Serial.println("BLE test sent");
    } else {
      Serial.println("BLE test pending: connect and enable TX notifications");
    }
  }
  if (command == 'd') dumpRow = 0;
  if (dumpRow == noDump || Serial.availableForWrite() < 64) return;
  if (eventNumber == 0) {
    Serial.println("No recorded event");
    dumpRow = noDump;
    return;
  }
  if (dumpRow == 0) {
    char header[64];
    snprintf(header, sizeof(header), "event=%lu,trigger_ms=%lu\n",
             static_cast<unsigned long>(eventNumber),
             static_cast<unsigned long>(eventTimeMs));
    Serial.print(header);
    Serial.println("offset_ms,ax,ay,az,gx,gy,gz");
  } else {
    const MotionSample& sample = eventWindow[dumpRow - 1];
    const int offsetMs = (static_cast<int>(dumpRow) - static_cast<int>(windowSamples)) *
                         static_cast<int>(samplePeriodUs / 1000);
    char line[64];
    snprintf(line, sizeof(line), "%d,%d,%d,%d,%d,%d,%d\n", offsetMs,
             sample.accel[0], sample.accel[1], sample.accel[2],
             sample.gyro[0], sample.gyro[1], sample.gyro[2]);
    Serial.print(line);
  }
  ++dumpRow;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  bleReady = startBle();
  imuReady = startImu();
  nextSampleUs = micros();
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  if (imuReady && static_cast<int32_t>(nowUs - nextSampleUs) >= 0) {
    if (nowUs - nextSampleUs >= samplePeriodUs) {
      detector.resetWindow();
      nextSampleUs = nowUs;
    }
    nextSampleUs += samplePeriodUs;

    MotionSample sample;
    if (readMotion(sample)) {
      invalidSamples = 0;
      ++validSamples;
      if (detector.add(sample)) recordEvent(nowMs);
    } else {
      if (invalidSamples < sampleRateHz) ++invalidSamples;
      detector.resetWindow();
    }
  }

  if (alertActive && nowMs - eventTimeMs >= alertDurationMs) alertActive = false;
  digitalWrite(LED_RED, alertActive ? LOW : HIGH);
  const bool sensorFault = !imuReady || invalidSamples >= sampleRateHz;
  digitalWrite(LED_BLUE, sensorFault && (nowMs / 250) % 2 ? LOW : HIGH);
  serviceNotification(nowMs);
  serviceSerial();
  delay(1);
}
