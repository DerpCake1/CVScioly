#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Define the pins for the display
// #define TFT_MOSI 23  // SDA Pin on ESP32
// #define TFT_SCLK 18  // SCL Pin on ESP32
#define TFT_CS 15 // Chip select control pin
#define TFT_DC 2  // Data Command control pin
#define TFT_RST 4 // Reset pin (could connect to RST pin)

// Create an instance of the display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void initDisplay() {

  // plug-in FPC (if you see the display is inverted!)
  tft.initR(INITR_MINI160x80_PLUGIN);  // Init ST7735S mini display
  tft.setRotation(3);
  tft.setTextWrap(true);
}

void display(String txt, uint16_t color = ST77XX_WHITE, uint8_t size = 3) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.print(txt);
}

void setup()
{
  initDisplay();
}

void loop()
{
  display("Hello!");
  delay(3000);
  display("World!", ST77XX_RED, 4);
  delay(3000);
}