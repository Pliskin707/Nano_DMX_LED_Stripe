#include <Arduino.h>
#include <DMXSerial.h>
#include <U8g2lib.h>
#include <Wire.h>

#define DMX_ADDR (200u)
#define LED_STRIPE_PIN (13)  // Pin D9/PB1/OC1A/LED_BUILTIN (make sure this is a hardware PWM pin which does not use Timer0 as this one is requried for millis() and delay() functions)

#define FONT_SMALL              u8g2_font_resoledmedium_tr
#define FONT_SMALL_MONO         u8g2_font_profont12_mf
#define FONT_LARGE              u8g2_font_chargen_92_tf
#define FONT_LARGE_MONO         u8g2_font_chargen_92_mf

auto oled = U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C(U8G2_R0);

void setup() {
  pinMode(LED_STRIPE_PIN, OUTPUT);

  // setup timer1 for the highest possible PWM frequency for the LED stripe to avoid flickering.
  // this is required for the LED stripe to work properly, otherwise it will flicker at low brightness levels.
  // the default PWM frequency for pins D5 and D6 is around 980 Hz, which may cause the recorded image of the camera to flicker at low brightness levels.
  // the following code sets the PWM frequency for pins D5 and D6 to around 31 kHz.
  // this is done by setting the timer1 prescaler to 1 and the timer1 mode to Fast PWM with TOP at OCR1A (mode 15).
  // for more information on how to set the PWM frequency for the LED stripe, see the following links:
  // - https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  
  // Set Timer1 to Fast PWM mode (WGM1 = 15) with TOP at OCR1A
  TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  OCR1A = 255; // Set TOP value for 8-bit PWM

  // setup the OLED display
  if (oled.begin())
  {
      oled.setPowerSave(false);
      oled.clearBuffer();
      oled.setFont(FONT_LARGE);
      oled.setDrawColor(1);

      oled.setCursor(0, 16);
      oled.print("DMX");
      oled.display();
  }

  DMXSerial.init(DMXReceiver);

  // default/start brightness
  DMXSerial.write(DMX_ADDR, 255);
}

void loop() {
  if (DMXSerial.dataUpdated())
  {
    DMXSerial.resetUpdated();

    // Update the LED stripe brightness based on the DMX value
    uint8_t brightness = DMXSerial.read(DMX_ADDR);
    analogWrite(LED_STRIPE_PIN, brightness);
  }
}
