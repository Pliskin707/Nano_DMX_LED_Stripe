#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define DEBUG_MODE  // uses the Serial port for debugging messsages. Cannot be used together with DMXSerial as both use the same hardware serial port.

#ifdef DEBUG_MODE
#include <HardwareSerial.h>
#else
#include <DMXSerial.h>
#endif

#define DMX_ADDR (200u)
#define LED_STRIPE_PIN (9)  // Pin D9/PB1/OC1A/LED_BUILTIN (make sure this is a hardware PWM pin which does not use Timer0 as this one is requried for millis() and delay() functions)


#define FONT_SMALL              u8g2_font_resoledmedium_tr
#define FONT_SMALL_MONO         u8g2_font_profont12_mf
#define FONT_LARGE              u8g2_font_chargen_92_tf
#define FONT_LARGE_MONO         u8g2_font_chargen_92_mf

auto oled = U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C(U8G2_R0);

void dim(__typeof__((oled))& instance, const bool dim)
{
    const uint8_t v = 3;    // value from 0 to 7, higher values more brighter
    const uint8_t p1 = 3;   // p1: 1..15, higher values, more darker, however almost no difference with 3 or more
    const uint8_t p2 = 1;   // p2: 1..15, higher values, more brighter

    if (dim)
    {
        instance.setContrast(0);
        instance.sendF("ca", 0x0db, v << 4);
        instance.sendF("ca", 0x0d9, (p2 << 4) | p1 );
    }
}

void setup() {
  pinMode(LED_STRIPE_PIN, OUTPUT);

  #ifdef DEBUG_MODE
  Serial.begin(115200);
  #endif
  
  // Use Timer1 hardware to produce a stable fast PWM output
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  // setup the OLED display
  oled.initDisplay();
  oled.setPowerSave(0);
  oled.clearDisplay();
  oled.setFont(FONT_SMALL);
  oled.setDrawColor(1);
  oled.drawStr(0, 16, "DMX");
  oled.setFont(FONT_LARGE_MONO);
  oled.drawStr(40, 16, "100");
  oled.setFont(FONT_SMALL);
  oled.drawStr(76, 16, "%");
  oled.drawFrame(40, 18, (128 - 40), (32 - 18));
  oled.sendBuffer();

  #ifndef DEBUG_MODE
  DMXSerial.init(DMXReceiver);
  
  // default/start brightness
  DMXSerial.write(DMX_ADDR, 200);
  #endif
}

void loop() {
  auto toggleBuiltinLED = []() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); };

  static uint32_t runningIndicator = 0uL;
  if ((millis() - runningIndicator) > 1000uL)
  {
    runningIndicator = millis();
    toggleBuiltinLED();
  }

  static bool dimmed = false;
  if ((millis() > 3000uL) && !dimmed)
  {
    dimmed = true;
    dim(oled, true);
  }

  #ifdef DmxSerial_h
  if (DMXSerial.dataUpdated())
  {
    DMXSerial.resetUpdated();
    toggleBuiltinLED(); // also visualizes communication

    // Update the LED stripe brightness based on the DMX value
    uint8_t brightness = DMXSerial.read(DMX_ADDR);
    analogWrite(LED_STRIPE_PIN, brightness);
  }
  #endif
}
