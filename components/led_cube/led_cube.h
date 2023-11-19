
/// 32 Color for a RGBW led
typedef struct{
    uint8_t red;         //Holds the Red color
    uint8_t green;       //Holds the Green color
    uint8_t blue;        //Holds the Blue color
    uint8_t white;       //Holds the White color
}RGBWLed;

extern const RGBWLed red;        //The color red
extern const RGBWLed green;      //The color green
extern const RGBWLed blue;       //The color blue
extern const RGBWLed white;      //The color white
extern const RGBWLed pink;       //The color pink
extern const RGBWLed yellow;     //The color yellow

void set_led_strip_to_solid_color(RGBWLed color, uint8_t *led_strip_pixels, uint8_t leds);
