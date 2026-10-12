#include <Arduino_LSM6DS3.h>
#include <Wire.h>

/*
  APEX - Nano 33 IoT IMU stream (v2: every sample, stamped on the MCU)

  Streams every sample of the onboard LSM6DS3 (output data rate 104 Hz, the
  rate the learned odometry was trained for) over USB serial, one CSV line
  per sample:
      ax,ay,az,gx,gy,gz,t_us,seq

  ax..az  acceleration, m/s^2
  gx..gz  angular velocity, rad/s
  t_us    micros() when the sample became ready on the chip (data-ready),
          wraps every 71.6 min; the host maps it to ROS time
          (apex_telemetry.timing.DeviceClockMapper)
  seq     sample counter since power-up (lines lost on the link show as gaps)

  The first six fields are unchanged, so an older host node that reads
  "ax,ay,az,gx,gy,gz" still works (it ignores the rest and stamps on
  reception).

  v1 streamed at 55.6 Hz: loop() ended with delay(10) after ~8 ms of reads
  and prints, so every other chip sample was skipped. v2 has no delay: it
  polls the chip's data-ready flags and reads each sample once, as soon as
  it is ready (real_vehicle/docs/sensor_timing_requirements.md).

  Chip configuration: IMU.begin() of Arduino_LSM6DS3 (accelerometer 104 Hz,
  +-4 g; gyroscope 104 Hz, +-2000 dps), the one modelled by the simulation
  profile APEX_real. Do not change it without updating that profile.
*/

static constexpr float GRAVITY_MPS2 = 9.80665f;
static constexpr float DEG2RAD = 0.017453292519943295f;
static constexpr unsigned long SERIAL_WAIT_TIMEOUT_MS = 2500;
// 400 kHz I2C: a status poll takes ~0.1 ms instead of ~0.4 ms, which bounds
// the delay between data-ready and the micros() stamp.
static constexpr uint32_t I2C_CLOCK_HZ = 400000;

static uint32_t sample_seq = 0;

void setup() {
  Serial.begin(115200);
  const unsigned long serial_wait_started_ms = millis();
  while (!Serial && (millis() - serial_wait_started_ms) < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }

  if (!IMU.begin()) {
    Serial.println("ERROR: IMU initialization failed");
    while (true) {
      delay(1000);
    }
  }
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("INFO: Nano IMU stream v2 started (ax,ay,az,gx,gy,gz,t_us,seq)");
}

void loop() {
  // Both sensors run at the same output data rate; a sample is complete
  // when both data-ready flags are set.
  if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) {
    return;
  }
  const uint32_t t_us = micros();

  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  IMU.readAcceleration(ax_g, ay_g, az_g);
  IMU.readGyroscope(gx_dps, gy_dps, gz_dps);

  Serial.print(ax_g * GRAVITY_MPS2, 6);
  Serial.print(',');
  Serial.print(ay_g * GRAVITY_MPS2, 6);
  Serial.print(',');
  Serial.print(az_g * GRAVITY_MPS2, 6);
  Serial.print(',');
  Serial.print(gx_dps * DEG2RAD, 6);
  Serial.print(',');
  Serial.print(gy_dps * DEG2RAD, 6);
  Serial.print(',');
  Serial.print(gz_dps * DEG2RAD, 6);
  Serial.print(',');
  Serial.print(t_us);
  Serial.print(',');
  Serial.println(sample_seq);
  ++sample_seq;
}
