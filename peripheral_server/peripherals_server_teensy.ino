#include "WS2812Serial.h"
#include "Bitcraze_PMW3901.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <SPI.h>

// Underglow LEDs
#define NUM_LED 62
#define PIN_LED 1
byte drawing_memory[NUM_LED * 3];         // 3 bytes per LED
DMAMEM byte display_memory[NUM_LED * 12]; // 12 bytes per LED
WS2812Serial leds(NUM_LED, display_memory, drawing_memory, PIN_LED, WS2812_GRB);

// Back LEDs
#define NUM_LED_BACK 25
#define PIN_LED_BACK 8
byte drawing_memory_back[NUM_LED_BACK * 3];         // 3 bytes per LED
DMAMEM byte display_memory_back[NUM_LED_BACK * 12]; // 12 bytes per LED
WS2812Serial leds_back(NUM_LED_BACK, display_memory_back, drawing_memory_back, PIN_LED_BACK, WS2812_GRB);

// Flow sensors
#define PIN_CS_FLOW1 9
#define PIN_CS_FLOW2 10
Bitcraze_PMW3901 flow1(PIN_CS_FLOW1);
Bitcraze_PMW3901 flow2(PIN_CS_FLOW2);

// LED states and colors
#define LED_STATE_IDLE 0
#define LED_STATE_ACTIVE 1
#define LED_STATE_BOOT 2
#define LED_STATE_ERROR 3
#define LED_STATE_DISCO 4

#define COLOR_PURPLE 0x400040
#define COLOR_GRAY 0x222222
#define COLOR_BLACK 0x000000
#define COLOR_WHITE 0xffffff
#define COLOR_RED 0xff0000
#define COLOR_GREEN 0x00ff00
#define COLOR_YELLOW 0xaa8800

#define BOOT_ANIMATION_DURATION 8

char led_state = LED_STATE_IDLE;
float led_active_angle;
float led_active_velocity;

int status_led_power = COLOR_WHITE;
int status_led_battery = COLOR_WHITE;
int status_led_warning = COLOR_WHITE;
int status_led_arm = COLOR_WHITE;
int status_led_wheels = COLOR_WHITE;

// Variables for BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Time since last message received
uint time_last_receive = millis();
uint time_since_boot_receive = millis();

void setup() {
    Serial.begin(115200);

    // Setup SPI port
    SPI.begin();
    pinMode(PIN_CS_FLOW1, OUTPUT);
    pinMode(PIN_CS_FLOW2, OUTPUT);
    digitalWrite(PIN_CS_FLOW1, HIGH);
    digitalWrite(PIN_CS_FLOW2, HIGH);

    if (!flow1.begin()) {
        Serial.println("ERROR_FLOW1_INIT");
        while (1) {}
    }
    flow1.setLed(true);
    if (!flow2.begin()) {
        Serial.println("ERROR_FLOW2_INIT");
        while (1) {}
    }
    flow2.setLed(true);

    // Setup BNO055
    if (!bno.begin()) {
        Serial.println("ERROR_BNO_INIT");
        while (1) {}
    }
    bno.setExtCrystalUse(true);
    bno.setMode(OPERATION_MODE_COMPASS);

    leds.begin();
    leds_back.begin();
}

void loop() {
    check_serial();
    update_underglow();
    update_back();

    delay(1);
}

int16_t deltaX1, deltaY1, deltaX2, deltaY2;

void check_serial() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        time_last_receive = millis();

        if (command.equals("PING")) {
            Serial.println("PONG");
        } else if (command.equals("FLOW")) {
            // Get motion count since last call
            flow1.readMotionCount(&deltaX1, &deltaY1);
            flow2.readMotionCount(&deltaX2, &deltaY2);

            Serial.print(deltaX1);
            Serial.print(",");
            Serial.print(deltaY1);
            Serial.print(",");
            Serial.print(deltaX2);
            Serial.print(",");
            Serial.println(deltaY2);
        } else if (command.equals("BNO")) {
            // Get orientation
            sensors_event_t event;
            bno.getEvent(&event);

            Serial.print(event.orientation.x);
            Serial.print(",");
            Serial.print(event.orientation.y);
            Serial.print(",");
            Serial.println(event.orientation.z);
        } else if (command.equals("BNO MAG")) {
            // Get magnetometer
            sensors_event_t event;
            bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);

            Serial.print(event.magnetic.x);
            Serial.print(",");
            Serial.print(event.magnetic.y);
            Serial.print(",");
            Serial.println(event.magnetic.z);
        } else if (command.startsWith("LED ")) {
            command = command.substring(4);
            if (command.equals("IDLE")) {
                led_state = LED_STATE_IDLE;
            } else if (command.startsWith("ACTIVE ")) {
                led_state = LED_STATE_ACTIVE;
                command = command.substring(7);
                led_active_angle = command.substring(0, command.indexOf(" ")).toFloat();
                led_active_velocity = command.substring(command.indexOf(" ") + 1).toFloat();
            } else if (command.equals("BOOT")) {
                time_since_boot_receive = millis();
                led_state = LED_STATE_BOOT;
            } else if (command.equals("DISCO")) {
                led_state = LED_STATE_DISCO;
            } else if (command.startsWith("STATUS ")) {
                int led_status_index;
                int led_status_state;

                command = command.substring(7);
                led_status_index = command.substring(0, command.indexOf(" ")).toInt();
                led_status_state = command.substring(command.indexOf(" ") + 1).toInt();

                if (led_status_index == 0) {
                    status_led_power = led_status_state == 0 ? COLOR_RED : COLOR_GREEN;
                } else if (led_status_index == 1) {
                    status_led_battery = led_status_state == 0 ? COLOR_RED : COLOR_GREEN;
                } else if (led_status_index == 2) {
                    status_led_warning = led_status_state == 0 ? COLOR_RED : COLOR_GREEN;
                } else if (led_status_index == 3) {
                    status_led_arm = led_status_state == 0 ? COLOR_RED : COLOR_GREEN;
                } else if (led_status_index == 4) {
                    status_led_wheels = led_status_state == 0 ? COLOR_RED : COLOR_GREEN;
                }
            } else if (command.equals("ERROR")) {
                led_state = LED_STATE_ERROR;
            }

            Serial.println("OK");
        }
    }
}

void update_underglow() {
    if (led_state == LED_STATE_ACTIVE && millis() > time_last_receive + 1000) {
        led_state = LED_STATE_ERROR;
    }

    int color;
    int led;
    int num_led_bar;
    int num_lit_leds;
    switch (led_state) {
        case LED_STATE_IDLE:
            set_all_leds(COLOR_GRAY);
            break;
        case LED_STATE_ACTIVE:
            led = angle_to_led(led_active_angle);
            num_led_bar = led_active_velocity * 30;
            color = (millis() % 1000 < 500) ? COLOR_PURPLE : COLOR_BLACK;
            for (int i = 0; i < NUM_LED; i++) {
                leds.setPixel(i, color);
            }
            color = COLOR_WHITE;
            for (int i = 0; i < num_led_bar / 2; i++) {
                leds.setPixel((led - i) % NUM_LED, color);
                leds.setPixel((led + i) % NUM_LED, color);
            }
            leds.show();
            break;
        case LED_STATE_BOOT:
            // Circle animation. All LEDs start black, and gradually fill up over time.
            // The number of lit LEDs is determined by the time since boot.
            num_lit_leds = NUM_LED * ((millis() - time_since_boot_receive) / 1000) / BOOT_ANIMATION_DURATION;
            for (int i = 0; i < num_lit_leds; i++) {
                leds.setPixel(i, 0x0000ff);
            }
            for (int i = num_lit_leds; i < NUM_LED; i++) {
                leds.setPixel(i, COLOR_BLACK);
            }
            leds.show();

            // Switch to idle after the animation.
            if (num_lit_leds >= NUM_LED) {
                led_state = LED_STATE_IDLE;
            }

            break;
        case LED_STATE_DISCO:
            led_disco_mode();
            break;
        case LED_STATE_ERROR:
            color = (millis() % 500 < 200) ? COLOR_RED : COLOR_BLACK;
            set_all_leds(color);
            break;
    }
}

void update_back() {
    for (int i = 0; i < 20; i++) {
        leds_back.setPixel(i, 0x000000);
    }

    if (led_state == LED_STATE_ACTIVE) {
        if (led_active_angle < 6.28f - 0.35f && led_active_angle > 3.14f + 1.57f) {
            // Turning right
            int color = (millis() % 1000 < 500) ? COLOR_YELLOW : COLOR_BLACK;
            for (int i = 10; i < 20; i++) {
                leds_back.setPixel(i, color);
            }

        } else if (led_active_angle > 0.35f && led_active_angle < 1.57f) {
            // Turning left
            int color = (millis() % 1000 < 500) ? COLOR_YELLOW : COLOR_BLACK;
            for (int i = 0; i < 10; i++) {
                leds_back.setPixel(i, color);
            }
        }
    }

    // Status LEDs
    leds_back.setPixel(20, status_led_wheels); // right
    leds_back.setPixel(21, status_led_arm);
    leds_back.setPixel(22, status_led_warning);
    leds_back.setPixel(23, status_led_battery);
    leds_back.setPixel(24, status_led_power); // left
    leds_back.show();
}

#define NUM_DISCO_COLORS 6
const int disco_colors[NUM_DISCO_COLORS] = {
    0x0200E8, 0x9B00FF, 0xFF00B9, 0xFFF800, 0x18FF00, 0xFB0000
};
void led_disco_mode() {
    // Easter egg. Disco mode!
    int color_index = 0;

    // Back panel direction indicators.
    int color = (millis() % 1000 < 500) ? COLOR_YELLOW : COLOR_BLACK;
    for (int i = 10; i < 20; i++) {
        leds_back.setPixel(i, color);
    }
    for (int i = 0; i < 10; i++) {
        leds_back.setPixel(i, color);
    }
    leds_back.show();

    // Underglow LEDs.
    // Only update every 200ms.
    if (millis() % 200 != 0) {
        return;
    }

    color = 0;
    for (int i = 0; i < NUM_LED; i++) {
        color_index = random(NUM_DISCO_COLORS);
        color = disco_colors[color_index];
        leds.setPixel(i, color);
    }
    leds.show();
}

void set_all_leds(int color) {
    for (int i = 0; i < NUM_LED; i++) {
        leds.setPixel(i, color);
    }
    leds.show();
}

void set_all_back_leds(int color) {
    for (int i = 0; i < NUM_LED_BACK; i++) {
        leds_back.setPixel(i, color);
    }
    leds_back.show();
}

int angle_to_led(float angle) {
    angle = fmod(angle, 2 * 3.1415f);
    int led;
    if (angle < 0.111f) led = 6;
    else if (angle < 0.219f) led = 7;
    else if (angle < 0.322f) led = 8;
    else if (angle < 0.418f) led = 9;
    else if (angle < 0.507f) led = 10;
    else if (angle < 0.588f) led = 11;
    else if (angle < 0.644f) led = 12;
    else if (angle < 0.709f) led = 14;
    else if (angle < 0.785f) led = 15;
    else if (angle < 0.876f) led = 16;
    else if (angle < 0.983f) led = 17;
    else if (angle < 1.107f) led = 18;
    else if (angle < 1.249f) led = 19;
    else if (angle < 1.406f) led = 20;
    else if (angle < 1.571f) led = 21;
    else if (angle < 3.142f - 1.406f) led = 22;
    else if (angle < 3.142f - 1.249f) led = 23;
    else if (angle < 3.142f - 1.107f) led = 24;
    else if (angle < 3.142f - 0.983f) led = 25;
    else if (angle < 3.142f - 0.876f) led = 26;
    else if (angle < 3.142f - 0.785f) led = 27;
    else if (angle < 3.142f - 0.709f) led = 28;
    else if (angle < 3.142f - 0.644f) led = 29;
    else if (angle < 3.142f - 0.588f) led = 30;
    else if (angle < 3.142f - 0.507f) led = 31;
    else if (angle < 3.142f - 0.418f) led = 32;
    else if (angle < 3.142f - 0.322f) led = 33;
    else if (angle < 3.142f - 0.219f) led = 34;
    else if (angle < 3.142f - 0.111f) led = 35;
    else if (angle < 3.142f) led = 36;
    else if (angle < 3.142f + 0.111f) led = 37;
    else if (angle < 3.142f + 0.219f) led = 38;
    else if (angle < 3.142f + 0.322f) led = 39;
    else if (angle < 3.142f + 0.418f) led = 40;
    else if (angle < 3.142f + 0.507f) led = 41;
    else if (angle < 3.142f + 0.588f) led = 42;
    else if (angle < 3.142f + 0.644f) led = 43;
    else if (angle < 3.142f + 0.709f) led = 45;
    else if (angle < 3.142f + 0.785f) led = 46;
    else if (angle < 3.142f + 0.876f) led = 47;
    else if (angle < 3.142f + 0.983f) led = 48;
    else if (angle < 3.142f + 1.107f) led = 49;
    else if (angle < 3.142f + 1.249f) led = 50;
    else if (angle < 3.142f + 1.406f) led = 51;
    else if (angle < 3.142f + 1.571f) led = 52;
    else if (angle < 6.283f - 1.406f) led = 53;
    else if (angle < 6.283f - 1.249f) led = 54;
    else if (angle < 6.283f - 1.107f) led = 55;
    else if (angle < 6.283f - 0.983f) led = 56;
    else if (angle < 6.283f - 0.876f) led = 57;
    else if (angle < 6.283f - 0.785f) led = 58;
    else if (angle < 6.283f - 0.709f) led = 59;
    else if (angle < 6.283f - 0.644f) led = 60;
    else if (angle < 6.283f - 0.588f) led = 61;
    else if (angle < 6.283f - 0.507f) led = 0;
    else if (angle < 6.283f - 0.418f) led = 1;
    else if (angle < 6.283f - 0.322f) led = 2;
    else if (angle < 6.283f - 0.219f) led = 3;
    else if (angle < 6.283f - 0.111f) led = 4;
    else led = 5;
    return led;
}
