#include <stdio.h>
#include "led_cube.h"

const RGBWLed red = {0xFF, 0x00, 0x00, 0x00};        //The color red
const RGBWLed green = {0x00, 0xFF, 0x00, 0x00};      //The color green
const RGBWLed blue = {0x00, 0x00, 0xFF, 0x00};       //The color blue
const RGBWLed white = {0x00, 0x00, 0x00, 0xFF};      //The color white
const RGBWLed pink = {0xFA, 0x00, 0x00, 0x0F};       //The color pink
const RGBWLed yellow = {0xFF, 0x8F, 0x00, 0x05};     //The color yellow


void set_led_strip_to_solid_color(RGBWLed color, uint8_t *led_strip_pixels, uint8_t leds){
    for(int i=0; i<leds; i++){
        led_strip_pixels[i * 4 + 0] = color.green;
        led_strip_pixels[i * 4 + 1] = color.red;
        led_strip_pixels[i * 4 + 2] = color.blue;
        led_strip_pixels[i * 4 + 3] = color.white;
    }
}
