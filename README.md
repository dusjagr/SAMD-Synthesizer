# SAMD-Synthesizer
sound experiments with the XIAO SAMD chip

Thanks to Malte Steiner for the code! 

See the original repo here: https://github.com/HerrSteiner/SAMD-Synthesizer

After reproducing the Matle's code, I have been experimenting with adding the qTouch function to it. Designing first boards with 4 touchpads and 5 potentiometers, to hopefully make a stand-alone fun little instrument to play with.

## Adding q-touch

This library by adafruit seems to work nicely with the SAMD-Synthesizer:

https://github.com/adafruit/Adafruit_FreeTouch


## experimental DIY hardware

![](https://github.com/dusjagr/SAMD-Synthesizer/raw/main/hardware/Lakhosky_Synth/Lakhowksy_board_version1.jpg)

It's mapped like this:

    //QT1           QT8
    //   A5  A2  A9
    //    A5   A3
    // QT6      QT7

## More about q-touch

For oversample :

OVERSAMPLE_1
OVERSAMPLE_2
OVERSAMPLE_4
OVERSAMPLE_8
OVERSAMPLE_16
OVERSAMPLE_32
OVERSAMPLE_64

For series resistor :

RESISTOR_0
RESISTOR_20K
RESISTOR_50K
RESISTOR_100K

For freq mode :

FREQ_MODE_NONE
FREQ_MODE_HOP
FREQ_MODE_SPREAD
FREQ_MODE_SPREAD_MEDIAN
