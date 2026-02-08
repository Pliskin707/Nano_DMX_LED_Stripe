#include <Arduino.h>
#include <U8g2lib.h>  // dynamic buffer allocation stays disabled, since only one display is used (this µC doesn't have enough RAM for two displays anyways)
#include <Wire.h>

// #define DEBUG_MODE  // uses the Serial port for debugging messsages. Cannot be used together with DMXSerial as both use the same hardware serial port.

#ifdef DEBUG_MODE
#include <HardwareSerial.h>
#else
#include <DMXSerial.h>
#endif

#define DMX_ADDR (200u)
#define LED_STRIPE_PIN (9)  // Pin D9/PB1/OC1A (make sure this is a hardware PWM pin which does not use Timer0 as this one is requried for millis() and delay() functions)


#define FONT_SMALL              u8g2_font_resoledmedium_tr
#define FONT_SMALL_MONO         u8g2_font_profont12_mf
#define FONT_LARGE              u8g2_font_chargen_92_tf
#define FONT_LARGE_MONO         u8g2_font_chargen_92_mf

static auto oled = U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C(U8G2_R0);
static uint8_t prevBrightness = 205u; // 80%

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

void printBrightnessBar(__typeof__((oled))& instance, const uint8_t brightness)
{
  const u8g2_uint_t 
    xStart       = 40,
    maxBarLen    = instance.getWidth() - xStart - 2 * 2, // frame + 1 pixel for the bar, each left and right
    barLen       = static_cast<u8g2_uint_t>((static_cast<uint16_t>(brightness) * static_cast<uint16_t>(maxBarLen)) / UINT16_C(255)),
    textBaseLine = (instance.getHeight() + 16) / 2;

  // clear previous content
  instance.setDrawColor(0);
  instance.drawBox(xStart, 0, instance.getWidth() - xStart, instance.getHeight());
  instance.setDrawColor(1);

  // draw bar
  instance.drawFrame(xStart, 0, instance.getWidth() - xStart, instance.getHeight());
  instance.drawBox(xStart + 2 + (maxBarLen - barLen), 2, barLen, instance.getHeight() - 2 * 2);

  // add text
  instance.setFont(FONT_LARGE_MONO);
  instance.setFontDirection(0);
  instance.setFontMode(true);
  instance.setDrawColor(2); // exor
  char buf[5];
  snprintf(buf, sizeof(buf), "%3d", (brightness * 100) / 255);
  buf[sizeof(buf) - 1] = 0;
  instance.drawStr(60, textBaseLine, buf);
  instance.setFont(FONT_SMALL);
  instance.drawStr(96, textBaseLine, "%");

  // revert "special" font modes
  instance.setFontMode(false);
  instance.setDrawColor(1);

  instance.sendBuffer();
}

void setup() {
  pinMode(LED_STRIPE_PIN, OUTPUT);

  #ifdef DEBUG_MODE
  Serial.begin(115200);
  #endif
  
  // Use Timer1 hardware to produce a stable fast PWM output (62.5 kHz)
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  // setup the OLED display
  oled.initDisplay();
  oled.setPowerSave(0);
  oled.clearDisplay();
  oled.setDrawColor(1);

  oled.setFont(FONT_SMALL);
  oled.setFontDirection(3);
  oled.drawRFrame(0, 0, 22, oled.getHeight(), 5);
  char buf[10];
  oled.drawStr(10, oled.getHeight() - 7, "DMX:");
  snprintf(buf, sizeof(buf), "%3ud", DMX_ADDR);
  oled.drawStr(19, oled.getHeight() - 6, buf);

  printBrightnessBar(oled, prevBrightness);

  #ifndef DEBUG_MODE
  DMXSerial.init(DMXReceiver);
  
  // default/start brightness
  // for now this does not start at 100% since the arduino lib would disconnect the output pin at set it to "permanently high"
  // since I want to check if the camera image flickers with the chosen frequency, it is set to a lower value until the check is done
  DMXSerial.write(DMX_ADDR, prevBrightness);
  #endif
}

void loop() {
  auto toggleBuiltinLED = []() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); };
  static uint32_t nextOledUpdate = 0uL;

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

    if ((brightness != prevBrightness) && !nextOledUpdate)
      nextOledUpdate = millis() + 100uL; // update OLED with some delay to avoid too many updates

    prevBrightness = brightness;
  }
  #endif

  if (millis() >= nextOledUpdate)
  {
    nextOledUpdate = 0uL;
    printBrightnessBar(oled, prevBrightness);
  }
}
