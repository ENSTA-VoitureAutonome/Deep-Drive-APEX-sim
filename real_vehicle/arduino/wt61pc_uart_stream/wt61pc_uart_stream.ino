/*
  APEX - DFRobot WT61PC / SEN0386 UART bridge for Arduino Nano 33 IoT
  (v2: 100 Hz, every sample stamped on the MCU)

  Wiring:
    WT61PC TXD -> Arduino D0 / RX1 (Serial1 RX, schematic net RX_MCU)
    WT61PC RXD -> Arduino D1 / TX1 (Serial1 TX, schematic net TX_MCU)
    WT61PC GND -> Arduino GND
    WT61PC VCC -> 3V3 or 5V according to the module label

  Nano 33 IoT uses 3.3 V logic and is not 5 V tolerant. If the WT61PC TXD line
  outputs 5 V, use a level shifter before connecting it to D0/RX1.

  USB Serial output to PC/Raspberry, one CSV line per sample:
    ax,ay,az,gx,gy,gz,t_us,seq

  Units:
    acceleration: m/s^2
    angular velocity: rad/s
    t_us: micros() when the first byte of the sample's acceleration frame
          arrived, wraps every 71.6 min; the host maps it to ROS time
          (apex_telemetry.timing.DeviceClockMapper)
    seq:  sample counter since power-up (lines lost on the link show as gaps)

  The first six fields are unchanged, so an older host node that reads
  "ax,ay,az,gx,gy,gz" still works (it ignores the rest and stamps on
  reception).

  v1 left the sensor at its factory setting, 9600 baud and 10 Hz output: a
  9600 baud link carries at most ~29 acceleration + gyroscope + angle
  triplets per second. v2 switches the sensor to 115200 baud, 100 Hz output
  and ~42 Hz bandwidth at every start (unlock 0x69, then registers 0x04 BAUD,
  0x03 RATE and 0x1F BANDWIDTH; nothing is saved in the sensor, so the
  factory setting stays for other readers) and configures it again if the
  frames stop (sensor power cycle). If the sensor does not take the
  commands, the bridge keeps streaming at whatever rate it finds and says
  so in a WARN line.

  This sketch parses WT61PC frames directly:
    0x55 0x51 ... acceleration
    0x55 0x52 ... gyroscope
    0x55 0x53 ... angle
*/

#include <Arduino.h>

static constexpr unsigned long DEBUG_BAUD = 115200;
static constexpr unsigned long SENSOR_FACTORY_BAUD = 9600;
static constexpr unsigned long SENSOR_FAST_BAUD = 115200;
static constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 2500;
static constexpr unsigned long STATUS_PERIOD_MS = 1000;
static constexpr unsigned long NO_FRAME_RECONFIGURE_MS = 1000;
static constexpr float GRAVITY_MPS2 = 9.80665f;
static constexpr float DEG2RAD = 0.017453292519943295f;

static constexpr uint8_t FRAME_START = 0x55;
static constexpr uint8_t FRAME_ACC = 0x51;
static constexpr uint8_t FRAME_GYRO = 0x52;
static constexpr uint8_t FRAME_ANGLE = 0x53;
static constexpr uint8_t FRAME_LEN = 11;

// WitMotion configuration command: 0xFF 0xAA <register> <data low> <data high>.
static constexpr uint8_t REG_RATE = 0x03;
static constexpr uint8_t REG_BAUD = 0x04;
static constexpr uint8_t REG_BANDWIDTH = 0x1F;
static constexpr uint8_t RATE_100HZ = 0x09;
static constexpr uint8_t BAUD_115200 = 0x06;
// Measured on the car (2026-10-09, 100 Hz output): the factory bandwidth 4
// (~20 Hz) computes only 50 new samples/s (every other line repeats) and
// holds gz at exactly 0 at rest; 3 (~42 Hz) gives 100 new samples/s and is
// the closest to the APEX_real filter (gyro 33 Hz, accel 50 Hz).
static constexpr uint8_t BANDWIDTH_42HZ = 0x03;

struct Vector3f {
  float x;
  float y;
  float z;
};

uint8_t frameBuffer[FRAME_LEN];
uint8_t frameIndex = 0;
uint32_t frameStartUs = 0;
uint32_t accStartUs = 0;
Vector3f acc = {0.0f, 0.0f, 0.0f};
Vector3f gyro = {0.0f, 0.0f, 0.0f};
Vector3f angle = {0.0f, 0.0f, 0.0f};
bool haveAcc = false;
bool haveGyro = false;
bool haveAngle = false;
unsigned long lastStatusMs = 0;
unsigned long lastFrameMs = 0;
unsigned long sensorBaud = 0;
uint32_t sampleSeq = 0;

bool isKnownFrameType(uint8_t value) {
  return value == FRAME_ACC || value == FRAME_GYRO || value == FRAME_ANGLE;
}

uint8_t checksum(const uint8_t *frame) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    sum += frame[i];
  }
  return sum;
}

int16_t readSigned16LE(const uint8_t *frame, uint8_t lowByteIndex) {
  const uint16_t raw = static_cast<uint16_t>(frame[lowByteIndex]) |
                       (static_cast<uint16_t>(frame[lowByteIndex + 1]) << 8);
  return static_cast<int16_t>(raw);
}

Vector3f decodeVector(const uint8_t *frame, float scale) {
  return {
    readSigned16LE(frame, 2) * scale,
    readSigned16LE(frame, 4) * scale,
    readSigned16LE(frame, 6) * scale,
  };
}

bool readWt61pcFrame(uint8_t *outFrame, uint32_t &outStartUs) {
  while (Serial1.available() > 0) {
    const uint8_t byteIn = static_cast<uint8_t>(Serial1.read());

    if (frameIndex == 0) {
      if (byteIn != FRAME_START) {
        continue;
      }
      frameStartUs = micros();
      frameBuffer[0] = byteIn;
      frameIndex = 1;
      continue;
    }

    if (frameIndex == 1) {
      if (!isKnownFrameType(byteIn)) {
        frameIndex = 0;
        if (byteIn == FRAME_START) {
          frameStartUs = micros();
          frameBuffer[0] = byteIn;
          frameIndex = 1;
        }
        continue;
      }
      frameBuffer[1] = byteIn;
      frameIndex = 2;
      continue;
    }

    frameBuffer[frameIndex] = byteIn;
    frameIndex++;
    if (frameIndex < FRAME_LEN) {
      continue;
    }

    frameIndex = 0;
    if (checksum(frameBuffer) != frameBuffer[10]) {
      continue;
    }
    memcpy(outFrame, frameBuffer, FRAME_LEN);
    outStartUs = frameStartUs;
    return true;
  }
  return false;
}

void processFrame(const uint8_t *frame, uint32_t startUs) {
  switch (frame[1]) {
    case FRAME_ACC:
      acc = decodeVector(frame, (16.0f * GRAVITY_MPS2) / 32768.0f);
      accStartUs = startUs;
      haveAcc = true;
      // A new sample starts with its acceleration frame.
      haveGyro = false;
      haveAngle = false;
      break;
    case FRAME_GYRO:
      gyro = decodeVector(frame, (2000.0f * DEG2RAD) / 32768.0f);
      haveGyro = true;
      break;
    case FRAME_ANGLE:
      angle = decodeVector(frame, 180.0f / 32768.0f);
      haveAngle = true;
      break;
    default:
      break;
  }
}

void writeConfig(uint8_t reg, uint8_t value) {
  const uint8_t cmd[5] = {0xFF, 0xAA, reg, value, 0x00};
  Serial1.write(cmd, sizeof(cmd));
  Serial1.flush();
  delay(50);
}

void openSensorPort(unsigned long baud) {
  Serial1.end();
  Serial1.begin(baud);
  frameIndex = 0;
  haveAcc = false;
  haveGyro = false;
  haveAngle = false;
}

// Valid frames per second seen at a baud rate over a window.
float frameRateAt(unsigned long baud, unsigned long windowMs) {
  openSensorPort(baud);
  delay(20);
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  uint8_t frame[FRAME_LEN];
  uint32_t startUs = 0;
  unsigned long frames = 0;
  const unsigned long startedMs = millis();
  while ((millis() - startedMs) < windowMs) {
    while (readWt61pcFrame(frame, startUs)) {
      frames++;
    }
  }
  return frames * 1000.0f / windowMs;
}

// Newer WitMotion firmware ignores register writes until it is unlocked.
void unlockConfig() {
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  Serial1.write(cmd, sizeof(cmd));
  Serial1.flush();
  delay(50);
}

// Older WitMotion protocol (JY61 / WT61C): 0xFF 0xAA 0x63 = 115200 baud, 100 Hz.
// Two zero bytes complete it as a 5-byte frame, so a newer parser does not
// take the next command's header as its data. Nothing is saved: a power
// cycle restores the sensor.
void legacyFastCommand() {
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x63, 0x00, 0x00};
  Serial1.write(cmd, sizeof(cmd));
  Serial1.flush();
  delay(50);
}

bool switchedToFastBaud() {
  openSensorPort(SENSOR_FAST_BAUD);
  delay(50);
  if (frameRateAt(SENSOR_FAST_BAUD, 300) > 0.0f) {
    return true;
  }
  openSensorPort(SENSOR_FACTORY_BAUD);
  return false;
}

void writeRateAndBandwidth() {
  unlockConfig();
  writeConfig(REG_RATE, RATE_100HZ);
  writeConfig(REG_BANDWIDTH, BANDWIDTH_42HZ);
}

// Sensor at 115200 baud and 100 Hz; returns the baud rate it ends up at (0: no frames).
unsigned long configureSensor() {
  if (frameRateAt(SENSOR_FAST_BAUD, 300) > 0.0f) {
    writeRateAndBandwidth();
    return SENSOR_FAST_BAUD;
  }
  if (frameRateAt(SENSOR_FACTORY_BAUD, 600) <= 0.0f) {
    return 0;
  }
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (attempt == 0) {
      writeConfig(REG_BAUD, BAUD_115200);
    } else if (attempt == 1) {
      unlockConfig();
      writeConfig(REG_BAUD, BAUD_115200);
    } else {
      legacyFastCommand();
    }
    if (switchedToFastBaud()) {
      writeRateAndBandwidth();
      Serial.print("INFO: WT61PC switched to 115200 baud by ");
      Serial.println(attempt == 0 ? "the BAUD register" : (attempt == 1 ? "the BAUD register after unlock" : "the legacy 0x63 command"));
      return SENSOR_FAST_BAUD;
    }
  }
  // The sensor did not take a baud command. 20 Hz still fits 9600 baud;
  // whether the rate changes tells if the sensor hears the Arduino at all.
  unlockConfig();
  writeConfig(REG_RATE, 0x07);  // 20 Hz
  openSensorPort(SENSOR_FACTORY_BAUD);
  return SENSOR_FACTORY_BAUD;
}

void startSensor() {
  sensorBaud = configureSensor();
  lastFrameMs = millis();
  if (sensorBaud == 0) {
    Serial.println("WARN: no WT61PC frames yet; check TX/RX crossed, GND, VCC");
    return;
  }
  const float rate = frameRateAt(sensorBaud, 1000);
  Serial.print("INFO: WT61PC at ");
  Serial.print(sensorBaud);
  Serial.print(" baud, frames/s ");
  Serial.println(rate, 1);
  if (sensorBaud != SENSOR_FAST_BAUD) {
    // 3 frames per sample: ~30 frames/s is 10 Hz, ~60 frames/s is 20 Hz.
    Serial.println(rate > 45.0f ? "WARN: WT61PC kept 9600 baud but took the rate command: 20 Hz"
                                : "WARN: WT61PC ignores commands (check WT61PC RXD <- D1/TX1): 10 Hz");
  }
  lastFrameMs = millis();
}

void printCsvSample() {
  Serial.print(acc.x, 6);
  Serial.print(",");
  Serial.print(acc.y, 6);
  Serial.print(",");
  Serial.print(acc.z, 6);
  Serial.print(",");
  Serial.print(gyro.x, 6);
  Serial.print(",");
  Serial.print(gyro.y, 6);
  Serial.print(",");
  Serial.print(gyro.z, 6);
  Serial.print(",");
  Serial.print(accStartUs);
  Serial.print(",");
  Serial.println(sampleSeq);
  ++sampleSeq;
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  const unsigned long waitStartedMs = millis();
  while (!Serial && (millis() - waitStartedMs) < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  Serial.println("INFO: WT61PC UART bridge v2 started");
  Serial.println("INFO: Board: Arduino Nano 33 IoT");
  Serial.println("INFO: Expected sensor wiring: TXD->D0/RX1, RXD->D1/TX1, GND->GND");
  Serial.println("INFO: Output CSV: ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps,t_us,seq");
  startSensor();
}

// Diagnostics from the host: a line "W rr ll hh" (hex) writes register rr of
// the sensor with data ll hh after the unlock, not saved. The APEX host node
// never writes to the port.
char commandLine[24];
uint8_t commandLength = 0;

void handleHostCommand() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c != '\n' && c != '\r') {
      if (commandLength < sizeof(commandLine) - 1) {
        commandLine[commandLength++] = c;
      }
      continue;
    }
    commandLine[commandLength] = '\0';
    commandLength = 0;
    unsigned int reg = 0;
    unsigned int lo = 0;
    unsigned int hi = 0;
    if (sscanf(commandLine, "W %x %x %x", &reg, &lo, &hi) != 3) {
      continue;
    }
    unlockConfig();
    const uint8_t cmd[5] = {0xFF, 0xAA, static_cast<uint8_t>(reg), static_cast<uint8_t>(lo), static_cast<uint8_t>(hi)};
    Serial1.write(cmd, sizeof(cmd));
    Serial1.flush();
    Serial.print("INFO: wrote WT61PC register 0x");
    Serial.println(reg, HEX);
  }
}

void loop() {
  handleHostCommand();
  uint8_t frame[FRAME_LEN];
  uint32_t startUs = 0;
  while (readWt61pcFrame(frame, startUs)) {
    lastFrameMs = millis();
    processFrame(frame, startUs);
    if (haveAcc && haveGyro && haveAngle) {
      printCsvSample();
      haveAcc = false;
      haveGyro = false;
      haveAngle = false;
    }
  }

  const unsigned long nowMs = millis();
  if ((nowMs - lastFrameMs) >= NO_FRAME_RECONFIGURE_MS) {
    // Sensor power cycle (back to its factory setting) or a loose wire.
    startSensor();
    return;
  }
  if ((nowMs - lastStatusMs) >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    if (sensorBaud == 0) {
      Serial.println("WARN: no WT61PC frames yet; check TX/RX crossed, GND, VCC, and 9600 baud");
    }
  }
}
