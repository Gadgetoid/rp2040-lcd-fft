
#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

#include "libraries/generic_st7789/generic_st7789.hpp"
#include "drivers/button/button.hpp"

#include "fft.hpp"

using namespace pimoroni;

enum DisplayStyle {
    WATERFALL,  // Waterfall style false colour, good view of frequencies, shows harmonics really nicely
    BAR         // Bar graph shows transients and gives a better visual idea of amplitude at frequencies
};

// False colour map, generated with "palette.py" using matplotlib
// default is nipy_spectral
// See: https://matplotlib.org/stable/gallery/color/colormap_reference.html
const Pen FALSE_COLOR_MAP[] = {
    0x0000, 0x0108, 0x0210, 0x0318, 0x0520, 0x0628, 0x0738, 0x0940, 0x0a48, 0x0b50, 0x0d58, 0x0e60, 0x0f70, 0x1170, 0x1178, 0x1178,
    0x1178, 0x1178, 0x1178, 0x1278, 0x1280, 0x1280, 0x1280, 0x1280, 0x1280, 0x1380, 0x1380, 0x1370, 0x1368, 0x1360, 0x1350, 0x1448,
    0x1440, 0x1430, 0x1428, 0x1420, 0x1410, 0x1508, 0x1500, 0x1500, 0x1600, 0x1600, 0x1700, 0x1700, 0x1800, 0x1800, 0x1900, 0x1900,
    0x1a00, 0x1a00, 0x1b00, 0x1b00, 0x5b00, 0x9b00, 0xfb00, 0x3b01, 0x7b01, 0xdb01, 0x1b02, 0x5b02, 0xbb02, 0xfb02, 0x3b03, 0x9b03,
    0xbb03, 0xdb03, 0xfb03, 0xfb03, 0x1b04, 0x3b04, 0x3b04, 0x5b04, 0x7b04, 0x7b04, 0x9b04, 0xbb04, 0xbb04, 0xdb04, 0xda04, 0xfa04,
    0xf904, 0xf904, 0x1805, 0x1805, 0x1705, 0x3705, 0x3605, 0x3605, 0x5505, 0x5505, 0x5505, 0x5405, 0x5405, 0x5405, 0x5305, 0x5305,
    0x5205, 0x5205, 0x5205, 0x5105, 0x5105, 0x5105, 0x5005, 0x4f05, 0x2e05, 0x2c05, 0x2b05, 0x0a05, 0x0805, 0x0705, 0xe604, 0xe404,
    0xe304, 0xc204, 0xc004, 0xc004, 0xe004, 0xe004, 0x0005, 0x2005, 0x2005, 0x4005, 0x6005, 0x6005, 0x8005, 0xa005, 0xa005, 0xc005,
    0xe005, 0xe005, 0x0006, 0x2006, 0x2006, 0x4006, 0x6006, 0x6006, 0x8006, 0xa006, 0xa006, 0xc006, 0xe006, 0xe006, 0x0007, 0x2007,
    0x2007, 0x4007, 0x6007, 0x6007, 0x8007, 0xa007, 0xa007, 0xc007, 0xe007, 0xe007, 0xe00f, 0xe01f, 0xe02f, 0xe03f, 0xe04f, 0xe057,
    0xe067, 0xe077, 0xe087, 0xe097, 0xe0a7, 0xe0af, 0xe0bf, 0xe0bf, 0xc0c7, 0xc0c7, 0xc0cf, 0xa0cf, 0xa0d7, 0xa0d7, 0x80df, 0x80df,
    0x80e7, 0x60e7, 0x60ef, 0x60ef, 0x40ef, 0x20f7, 0x20f7, 0x00f7, 0xe0f6, 0xe0f6, 0xc0f6, 0xa0fe, 0xa0fe, 0x80fe, 0x60fe, 0x60fe,
    0x40fe, 0x20fe, 0x00fe, 0xe0fd, 0xc0fd, 0xa0fd, 0x80fd, 0x60fd, 0x40fd, 0x20fd, 0x00fd, 0xe0fc, 0xc0fc, 0x60fc, 0x00fc, 0xa0fb,
    0x40fb, 0xe0fa, 0x80fa, 0x20fa, 0xc0f9, 0x60f9, 0x00f9, 0xa0f8, 0x40f8, 0x00f8, 0x00f8, 0x00f8, 0x00f0, 0x00f0, 0x00f0, 0x00e8,
    0x00e8, 0x00e8, 0x00e0, 0x00e0, 0x00e0, 0x00d8, 0x00d8, 0x00d8, 0x00d8, 0x00d8, 0x00d0, 0x00d0, 0x00d0, 0x00d0, 0x00d0, 0x00d0,
    0x00c8, 0x00c8, 0x00c8, 0x61c8, 0xc3c8, 0x65c9, 0xe7c9, 0x69ca, 0xebca, 0x6dcb, 0xefcb, 0x71cc, 0xf3cc, 0x75cd, 0xf7cd, 0x79ce
};

// Assorted config
const unsigned int min_sample = 5u; // Low-frequency samples have erroneous peaks
const unsigned int waterfall_start = 20u; // Starting Y position for the waterfall

// LCD config
const int WIDTH = 240;
const int HEIGHT = 240;
ST7789Generic lcd(WIDTH, HEIGHT, false, nullptr, BG_SPI_FRONT);

// Button config
const unsigned int BUTTON_A = 12u;
Button button_a(BUTTON_A);

// FFT config
const unsigned int adc_channel = 0;
const unsigned int adc_pin = 26;
const unsigned int sample_count = 512u;
const float sample_rate = 10000.0f;
FFT fft(adc_channel, adc_pin, sample_count, sample_rate);

int main() {
    // Waterfall vs Bar
    DisplayStyle display_style = WATERFALL;

    // Rolling y for waterfall mode
    unsigned int y = waterfall_start;

    // Handy colour constants
    const Pen WHITE = lcd.create_pen(255, 255, 255);
    const Pen BLACK = lcd.create_pen(0, 0, 0);

    stdio_init_all();

    lcd.set_backlight(255);
    lcd.set_pen(BLACK);
    lcd.clear();

    while(true) {
        lcd.set_pen(BLACK);
    
        if(button_a.read()) {
            display_style = display_style == WATERFALL ? BAR : WATERFALL;
            y = waterfall_start;
            lcd.clear();
        }

        if(display_style == WATERFALL) {
            lcd.rectangle(Rect(0, 0, WIDTH, 20));
        } else {
            lcd.clear();
        }

        // Write some text
        lcd.set_pen(WHITE);
        lcd.text("Max freq:", Point(0, 0), WIDTH);
    
        fft.update();

        float max_frequency = fft.max_frequency();

        char freqtext[40];
        snprintf(freqtext, 40, "%d", (int)max_frequency);
        lcd.text(freqtext, Point(WIDTH / 2, 0), WIDTH);

        for (auto i = min_sample; i < (sample_count / 2u); i++) {
            unsigned int x = i - min_sample;
            if(x >= WIDTH) break; // Don't draw off the right edge of the screen
            unsigned int height = fft.get_scaled(i, 144u);
            uint8_t c = std::min(height, 255u);
            lcd.set_pen(FALSE_COLOR_MAP[c]);
            if(display_style == WATERFALL) {
                lcd.pixel(Point(x, y));
            }
            else {
                lcd.line(Point(x, HEIGHT), Point(x, HEIGHT - height));
            }
        }

        y++;
        if(y >= HEIGHT) y = waterfall_start;

        lcd.update();
    }
}
