// ??
// #undef min
// #undef max
#include <etl/profiles/armv7.h>
#include <etl/vector.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Scales.h>
#include <Arduino.h>

#define TFT_CS         -1
#define TFT_RST        PA0
#define TFT_DC         PA1
#define TFT_BACKLIGHT PA2

const uint8_t LOADCELL_CLOCK = PB12;
const uint8_t LOADCELL_DATA[] = { PB13, PB14, PB15, PA8 };
const double SCALE = 5974.7895468853;
// 816216 / x = 136.61
Scales lt(LOADCELL_CLOCK, LOADCELL_DATA[0], SCALE);
Scales rt(LOADCELL_CLOCK, LOADCELL_DATA[1], SCALE);
Scales lb(LOADCELL_CLOCK, LOADCELL_DATA[2], SCALE);
Scales rb(LOADCELL_CLOCK, LOADCELL_DATA[3], SCALE);

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

struct element
{
  int16_t x;
  int16_t y;
  uint16_t color;
  uint8_t size;
};

element elements[] = {
  { 0, 0, ST77XX_RED, static_cast<uint8_t>(3u) },
  { 125, 0, ST77XX_GREEN, static_cast<uint8_t>(3u) },
  { 0, 125, ST77XX_BLUE, static_cast<uint8_t>(3u) },
  { 125, 125, ST77XX_ORANGE, static_cast<uint8_t>(3u) }
};

void setup() {
  Scales::InitPins();
  pinMode(PB9, INPUT_PULLUP);
  tft.init(240, 240, SPI_MODE3);
  delay(500);
  tft.fillScreen(ST77XX_BLACK);
}
uint32_t counter = 0;

void loop() {
  while (!Scales::IsAllReady()) {
    delay(10);
  }

  if (digitalRead(PB9) == LOW) {
    Scales::SetTareForAll();
  }

  Scales::ReadAll();
  tft.setCursor(elements[0].x, elements[0].y);
  tft.setTextColor(elements[0].color, ST77XX_BLACK);
  tft.setTextSize(elements[0].size);
  tft.println(lt.GetLastUnits());

  tft.setCursor(elements[1].x, elements[1].y);
  tft.setTextColor(elements[1].color, ST77XX_BLACK);
  tft.setTextSize(elements[1].size);
  tft.println(rt.GetLastUnits());

  tft.setCursor(elements[2].x, elements[2].y);
  tft.setTextColor(elements[2].color, ST77XX_BLACK);
  tft.setTextSize(elements[2].size);
  tft.println(lb.GetLastUnits());

  tft.setCursor(elements[3].x, elements[3].y);
  tft.setTextColor(elements[3].color, ST77XX_BLACK);
  tft.setTextSize(elements[3].size);
  tft.println(rb.GetLastUnits());
  counter++;
}
