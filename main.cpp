/*
PI PICO Demonstration of interface to an si5351

# Make sure that the PICO_SDK_PATH is set properly

cd /home/bruce/pico/hello-pico
# You need main.c and CMakeLists.txt
cp ../pico-sdk/external/pico_sdk_import.cmake .
mkdir build
cd build
cmake ..
make

# Connect the PI PICO to USB while pressing the BOOTSEL button.
# And then:
cp main.uf2 /media/bruce/RPI-RP2

# Make sure the SWD is connected properly:
# GPIO24 (Pin 18) to SWDIO
# GPIO25 (Pin 22) to SWCLK
# GND (Pin 20) to SWGND

# Use this command to flash:
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program main.elf verify reset exit"

# Looking at the serial port:
minicom -b 115200 -o -D /dev/ttyACM0
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "scamplib/fixed_fft.h"

#define LED_PIN (25)

static const uint16_t fftN = 512;
static q15 trigTable[fftN];
static const FixedFFT fft(fftN, trigTable);

static q15 fftWindow[fftN];
static q15 samples[fftN];

static const uint32_t adcClockHz = 48000000;
static const uint16_t sampleFreqHz = 2000;
static uint32_t adcSampleCount = 0;

// TODO: UNDERSTAND THIS DIRECTIVE
void __not_in_flash_func(adc_irq_handler) () {
    adcSampleCount++;
    if (adcSampleCount % 2000 == 0) {
        printf("TICK %ld\n", adcSampleCount);
    }
    adc_fifo_drain();
}

int main() {
 
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Startup ID
    for (uint i = 0; i < 4; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }    

    puts("ADC Demo");

    // Get the ADC iniialized
    uint8_t adcChannel = 0;
    adc_gpio_init(26 + adcChannel);
    adc_init();
    adc_select_input(adcChannel);
    adc_fifo_setup(
        true,   
        false,
        1,
        false,
        true
    );
    adc_set_clkdiv(adcClockHz / (uint32_t)sampleFreqHz);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);    
    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);
    adc_run(true);

    /*
    // Fill the samples with a tone
    float toneFreqHz = 100;
    float PI = 3.1415926;
    float omega = 2.0f * PI * ((float)toneFreqHz / (float)sampleFreqHz);
    float phi = 0;
    float amp = 0.5;
    for (uint16_t i = 0; i < fftN; i++ ) {
        float sig = std::cos(phi) * amp;
        phi += omega;
        samples[i] = f32_to_q15(sig);
    }
    uint16_t toneBin = (toneFreqHz / sampleFreqHz) * (float)fftN;
    */

    /*
    uint32_t startUs = time_us_32();
    // Make a complex series
    cq15 x[fftN];
    for (uint16_t i = 0; i < fftN; i++) {
        x[i].r = samples[i];
        x[i].i = 0;
    }
    // Do the transformation
    fft.transform(x);
    // Display a few buckets
    //printf("%d bin   %f\n", j, x[toneBin].mag_f32());
    //printf("%d bin-1 %f\n", j, x[toneBin - 1].mag_f32());
    //printf("%d bin+1 %f\n", j, x[toneBin + 1].mag_f32());
    uint32_t endUs = time_us_32();
    printf("Time elapsed per transform %ld\n", (endUs - startUs));
    */

    sleep_ms(100);
    printf("ADC count %ld\n", adcSampleCount);

    // Prevent the main fom exiting
    while (1) { }

    return 0;
}
