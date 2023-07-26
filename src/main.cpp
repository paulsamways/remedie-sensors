#include <Arduino.h>
#include <math.h>
#include <CAN.h>
#include <arduino-timer.h>

#define PIN_ENGINE_OIL_TEMPERATURE PIN_A3
#define PIN_TRANSMISSION_FLUID_TEMPERATURE PIN_A2
#define PIN_ENGINE_OIL_PRESSURE PIN_A4
#define PIN_DRIVER_SWITCH_A 5
#define PIN_DRIVER_SWITCH_B 6

#define CAN_ID_ENGINE_OIL_TEMPERATURE 0x24
#define CAN_ID_TRANSMISSION_FLUID_TEMPERATURE 0x25
#define CAN_ID_ENGINE_OIL_PRESSURE 0x26
#define CAN_ID_DRIVER_ID 0x27

auto timer = timer_create_default();


/*

                 ┌┐
                 ││
                 ││ Bosch NTC M12
               ┌─┴┴─┐
               │ R1 │
               └┬──┬┘
                │  │
                │  │
                │  │
                │  │
                │  │
                │ ┌┼──────┐
                │ ││R2    │
                │ └┤      │
                │  │      │
            ┌───┴──┴──────┴────┐
            │   V  G      A0   │
            │                  │
            └──────────────────┘
            
*/

double read_bosch_ntc_thermistor(uint8_t pin, int r2) {
  const double vref = 3.3;

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

double read_bosch_pst_f1_ntc(uint8_t pin, int r2) {
  const double vref = 3.3;

  const double A = 1.288158971e-3;
  const double B = 2.617143934e-4;
  const double C = 1.624978584e-7;

  int value = 1024 - analogRead(pin);

  double v = (vref / 1024) * value;
  double r1 = ((vref * r2) / v) - r2;

  // Steinhart and Hart Equation
  double logR1 = log(r1);
  double k = 1 / (A + B * logR1 + C * logR1 * logR1 * logR1);
  return k - 273.15;
}
double read_bosch_pst_f1_pressure(uint8_t pin) {
  const double vref = 3.3;

  int value = analogRead(pin);

  double v = ((vref / 1024) * value) - 0.33;
  double p = (145 / 3) * v;
  return p;
}
double read_sandwhich_plate_sensor(uint8_t pin, int r2) {
  const double vref = 3.3;

  const double A = 0.0006212298585387718;
  const double B = 0.00025538988541031865;
  const double C = -2.5963688685835344e-8;

  int value = 1024 - analogRead(pin);

  double v = (vref / 1024) * value;
  double r1 = ((vref * r2) / v) - r2;

  // Steinhart and Hart Equation
  double logR1 = log(r1);
  double k = 1 / (A + B * logR1 + C * logR1 * logR1 * logR1);
  return k - 273.15;
}

void write_can_packet(int id, uint8_t v)
{
  CAN.beginPacket(id);
  CAN.write(v);
  CAN.endPacket();

  if (Serial.availableForWrite())
  {
    Serial.print(id);
    Serial.print(": ");
    Serial.println(v);
  }

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

bool engine_oil_temperature_pressure(void *) {
  const double R2 = 10000;

  uint8_t c = round(read_sandwhich_plate_sensor(PIN_ENGINE_OIL_TEMPERATURE, R2));

  write_can_packet(CAN_ID_ENGINE_OIL_TEMPERATURE, c);

  double p = round(read_bosch_pst_f1_pressure(PIN_ENGINE_OIL_PRESSURE));
  
  write_can_packet(CAN_ID_ENGINE_OIL_PRESSURE, p);

  return true;
}

bool transmission_fluid_temperature(void *) {
  const double R2 = 10000;

  uint8_t c = round(read_bosch_ntc_thermistor(PIN_TRANSMISSION_FLUID_TEMPERATURE, R2));

  write_can_packet(CAN_ID_TRANSMISSION_FLUID_TEMPERATURE, c);

  return true;
}

bool driver_id(void *)
{
  static uint8_t last_driver_id = 0;

  uint8_t result = 0;

  if (!digitalRead(PIN_DRIVER_SWITCH_A))
  {
    result |= 1 << 0;
  }
  if (!digitalRead(PIN_DRIVER_SWITCH_B))
  {
    result |= 1 << 1;
  }

  if (last_driver_id != result)
  {
    last_driver_id = result;
  }
  else
  {
    write_can_packet(CAN_ID_DRIVER_ID, result);
  }

  return true;
}

void setup()
{
  Serial.begin(115200);

  // start the CAN bus at 500 kbps
  uint8_t attempts = 0;
  while (!CAN.begin(500E3) && attempts++ < 60)
  {
    Serial.println("Starting CAN failed!");
    delay(1000);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PIN_ENGINE_OIL_TEMPERATURE, INPUT);
  pinMode(PIN_TRANSMISSION_FLUID_TEMPERATURE, INPUT);
  pinMode(PIN_ENGINE_OIL_PRESSURE, INPUT);
  pinMode(PIN_DRIVER_SWITCH_A, INPUT_PULLUP);
  pinMode(PIN_DRIVER_SWITCH_B, INPUT_PULLUP);
  
  timer.every(1000, engine_oil_temperature_pressure);
  timer.every(1000, transmission_fluid_temperature);
  timer.every(1000, driver_id);
}

void loop() {
  timer.tick();
}