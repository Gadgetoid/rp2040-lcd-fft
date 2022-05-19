/**
 * Hunter Adams (vha3@cornell.edu)
 * Reproduced and modified with explicit permission
 * 
 * Original code in action:
 * https://www.youtube.com/watch?v=8aibPy4yzCk
 *
 * Resources Used
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - ADC channel 0
 *
 */
//#include "vga_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "libraries/generic_st7789/generic_st7789.hpp"

#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

// Helpers for 16.15 fixed-point arithmetic
typedef signed int fix15;
constexpr __always_inline fix15 multiply_fix15(fix15 a, fix15 b) {return (fix15)(((signed long long)(a) * (signed long long)(b)) >> 15);}
constexpr __always_inline fix15 float_to_fix15(float a) {return (fix15)(a * 32768.0f);}
constexpr __always_inline float fix15_to_float(fix15 a) {return (float)(a) / 32768.0f;}
constexpr __always_inline fix15 int_to_fix15(int a) {return (fix15)(a << 15);}
constexpr __always_inline int fix15_to_int(fix15 a) {return (int)(a >> 15);}

// ADC
const uint ADC_CHAN = 0;
const uint ADC_PIN = 26;

// Sample rate and depth
const uint NUM_SAMPLES = 512u; // Must be a power of 2
const uint MIN_SAMPLE = 5u;    // The very low frequencies contain erroneous high-amplitude values
const unsigned int LOG2_NUM_SAMPLES = log2(NUM_SAMPLES);  // log2 non constexpr :/
const unsigned int SHIFT_AMOUNT = (16u - LOG2_NUM_SAMPLES);
constexpr float Fs = 20000.0f;
constexpr float ADCCLK = 48000000.0f;
constexpr float PI_X2 = M_PI * 2.0f;

constexpr fix15 zero_point_4 = float_to_fix15(0.4f);

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES];

// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES];
fix15 fi[NUM_SAMPLES];

fix15 SINE_TABLE[NUM_SAMPLES]; // a table of sines for the FFT
fix15 FILTER_WINDOW[NUM_SAMPLES]; // a table of window values for the FFT


uint8_t *sample_address_pointer = &sample_array[0];

using namespace pimoroni;

const int WIDTH = 240;
const int HEIGHT = 240;

ST7789Generic lcd(WIDTH, HEIGHT, false, nullptr, BG_SPI_FRONT);

const Pen WHITE = lcd.create_pen(255, 255, 255);
const Pen BLACK = lcd.create_pen(0, 0, 0);


// Adapted from https://github.com/raspberrypi/pico-sdk/blob/master/src/host/pico_bit_ops/bit_ops.c
uint16_t __always_inline __revs(uint16_t v) {
    v = ((v & 0x5555u) << 1u) | ((v >> 1u) & 0x5555u);
    v = ((v & 0x3333u) << 2u) | ((v >> 2u) & 0x3333u);
    v = ((v & 0x0f0fu) << 4u) | ((v >> 4u) & 0x0f0fu);
    return ((v >> 8u) & 0x00ffu) | ((v & 0x00ffu) << 8u);
}

void FFT_fix15(fix15 fr[], fix15 fi[]) {

    // Bit Reversal Permutation
    // Bit reversal code below originally based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    // https://en.wikipedia.org/wiki/Bit-reversal_permutation
    // Detail here: https://vanhunteradams.com/FFT/FFT.html#Single-point-transforms-(reordering)
    //
    // PH: Converted to stdlib functions and __revs so it doesn't hurt my eyes
    for (auto m = 1u; m < NUM_SAMPLES - 1u; m++) {
        auto mr = __revs(m) >> SHIFT_AMOUNT;
        // don't swap that which has already been swapped
        if (mr <= m) continue;
        // swap the bit-reveresed indices
        std::swap(fr[m], fr[mr]);
        std::swap(fi[m], fi[mr]);
    }

    // Danielson-Lanczos
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Detail here: https://vanhunteradams.com/FFT/FFT.html#Two-point-transforms
    // Length of the FFT's being combined (starts at 1)
    //
    // PH: Moved variable declarations to first-use so types are visually explicit.
    // PH: Removed div 2 on sine table values, have computed the sine table pre-divided.
    int L = 1;
    int k = LOG2_NUM_SAMPLES - 1;

    // While the length of the FFT's being combined is less than the number of gathered samples
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        int istep = L << 1;
        // For each element in the FFT's that are being combined
        for (auto m = 0u; m < L; ++m) { 
            // Lookup the trig values for that element
            int j = m << k; // index into SINE_TABLE
            fix15 wr =  SINE_TABLE[j + NUM_SAMPLES / 4];
            fix15 wi = -SINE_TABLE[j];
            // i gets the index of one of the FFT elements being combined
            for (auto i = m; i < NUM_SAMPLES; i += istep) {
                // j gets the index of the FFT element being combined with i
                int j = i + L;
                // compute the trig terms (bottom half of the above matrix)
                fix15 tr = multiply_fix15(wr, fr[j]) - multiply_fix15(wi, fi[j]);
                fix15 ti = multiply_fix15(wr, fi[j]) + multiply_fix15(wi, fr[j]);
                // divide ith index elements by two (top half of above matrix)
                fix15 qr = fr[i] >> 1;
                fix15 qi = fi[i] >> 1;
                // compute the new values at each index
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }    
        }
        --k;
        L = istep;
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();

    lcd.set_backlight(255);

    // ADC Configuration

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_PIN);

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init();

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_CHAN);

    // Setup the FIFO
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 5kHz (48Mhz/5kHz - 1)
    adc_set_clkdiv(ADCCLK / Fs);

    // ADC DMA Configuration

    // Channels
    int sample_chan = 2;
    int control_chan = 3;

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);

    // ADC Sample Channel
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,    // transfer count
        false           // start immediately
    );

    // Control Channel
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);  // 32-bit txfers
    channel_config_set_read_increment(&c3, false);            // no read incrementing
    channel_config_set_write_increment(&c3, false);           // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);

    dma_channel_configure(
        control_chan,                         // Channel to be configured
        &c3,                                  // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
        &sample_address_pointer,              // Read address (POINTER TO AN ADDRESS)
        1,                                    // Number of transfers, in this case each is 4 byte
        false                                 // Don't start immediately.
    );

    // Filter and Sine tables

    for (auto ii = 0u; ii < NUM_SAMPLES; ii++) {
        // Full sine wave with period NUM_SAMPLES
        // Wolfram Alpha: Plot[(sin(2 * pi * (x / 1.0))), {x, 0, 1}]
        SINE_TABLE[ii] = float_to_fix15(0.5f * sin(PI_X2 * ((float) ii) / (float)NUM_SAMPLES));

        // This is a crude approximation of a Lanczos window.
        // Wolfram Alpha Comparison: Plot[0.5 * (1.0 - cos(2 * pi * (x / 1.0))), {x, 0, 1}], Plot[LanczosWindow[x - 0.5], {x, 0, 1}]
        FILTER_WINDOW[ii] = float_to_fix15(0.5f * (1.0f - cos(PI_X2 * ((float) ii) / ((float)NUM_SAMPLES))));
    }

    printf("Starting capture...\n");

    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan));

    // Start the ADC
    adc_run(true);

    while(true) {
        lcd.set_pen(BLACK);
        lcd.clear();

        // Write some text
        lcd.set_pen(WHITE);
        lcd.text("Max freqency:", Point(0, 0), 240);
    
        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer
        dma_channel_wait_for_finish_blocking(sample_chan);

        // Copy/window elements into a fixed-point array
        for (int i=0; i<NUM_SAMPLES; i++) {
            fr[i] = multiply_fix15(int_to_fix15((int)sample_array[i]), FILTER_WINDOW[i]);
            fi[i] = (fix15)0;
        }
        float max_fr = 0;
        int max_fr_dex = 0;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan);

        // Compute the FFT
        FFT_fix15(fr, fi);

        // Find the magnitudes
        for (auto i = 0u; i < (NUM_SAMPLES / 2u); i++) {  
            // get the approx magnitude
            fr[i] = abs(fr[i]); //>>9
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = std::max(fr[i], fi[i]) + 
                    multiply_fix15(std::min(fr[i], fi[i]), zero_point_4); 

            // Keep track of maximum
            if (fr[i] > max_fr && i >= MIN_SAMPLE) {
                max_fr = fr[i];
                max_fr_dex = i;
            }
        }

        float max_freqency = max_fr_dex * (Fs / NUM_SAMPLES);
        printf("%f\n", max_freqency);

        char freqtext[40];
        snprintf(freqtext, 40, "%d", (int)max_freqency);
        lcd.text(freqtext, Point(160, 0), 240);

        for (auto i = MIN_SAMPLE; i < (NUM_SAMPLES / 2u); i++) {
            if(i >= WIDTH) break; // Don't draw off the right edge of the screen
            int height = fix15_to_int(multiply_fix15(fr[i], int_to_fix15(144u)));
            lcd.line(Point(i, HEIGHT), Point(i, HEIGHT - height));
        }

        lcd.update();
    }
}
