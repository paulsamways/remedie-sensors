#include <Arduino.h>
#include <math.h>
#include <CAN.h>
#include <arduino-timer.h>

#define PIN_ENGINE_OIL PIN_A2
#define PIN_TRANSMISSION_FLUID PIN_A2
#define PIN_DRIVER_SWITCH_A 2
#define PIN_DRIVER_SWITCH_B 3

#define CAN_ID_ENGINE_OIL 0x24
#define CAN_ID_TRANSMISSION_FLUID 0x25
#define CAN_ID_DRIVER_ID 0x26

auto timer = timer_create_default();

double read_bosch_ntc_thermistor(uint8_t pin, int r2) {
  const double vref = 5.0;

  const double A = 0.0013017331448854658;
  const double B = 0.00025873055639535946;
  const double C = 1.7259681220354165e-7;

  int value = analogRead(pin);

  double v = (vref / 1024) * value;
  double r1 = ((vref * r2) / v) - r2;

  // Steinhart and Hart Equation
  double logR1 = log(r1);
  double k = 1 / (A + B * logR1 + C * logR1 * logR1 * logR1);
  return k - 273.15;
}

void write_can_packet(int id, uint8_t v) {
  CAN.beginPacket(id);
  CAN.write(v);
  CAN.endPacket();

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

bool engine_oil_temperature(void *) {
  const double R2 = 10000;

  uint8_t c = round(read_bosch_ntc_thermistor(PIN_ENGINE_OIL, R2));

  write_can_packet(CAN_ID_ENGINE_OIL, c);

  return true;
}

bool transmission_fluid_temperature(void *) {
  const double R2 = 10000;

  uint8_t c = round(read_bosch_ntc_thermistor(PIN_TRANSMISSION_FLUID, R2));

  write_can_packet(CAN_ID_TRANSMISSION_FLUID, c);

  return true;
}

bool driver_id(void *) {
  static uint8_t last_driver_id = 0;

  uint8_t result = 0;

  if (digitalRead(PIN_DRIVER_SWITCH_A)) {
    result |= 1 << 0;
  }
  if (digitalRead(PIN_DRIVER_SWITCH_B)) {
    result |= 1 << 1;
  }

  for (uint8_t i = 0; i < 8; i += 2) {
    uint8_t p = (last_driver_id >> i) & 0b00000011;

    if (p != result) {
      last_driver_id |= result << i;
      return;
    }
  }

  write_can_packet(CAN_ID_DRIVER_ID, result);
  
  return true;
}

void setup() {
  Serial.begin(115200);

  // start the CAN bus at 500 kbps
  while (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    delay(1000);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PIN_ENGINE_OIL, INPUT);
  pinMode(PIN_TRANSMISSION_FLUID, INPUT);
  pinMode(PIN_DRIVER_SWITCH_A, INPUT);
  pinMode(PIN_DRIVER_SWITCH_B, INPUT);
  
  timer.every(1000, engine_oil_temperature);
  timer.every(1000, transmission_fluid_temperature);
  timer.every(250, driver_id);
}

void loop() {
  timer.tick();
}