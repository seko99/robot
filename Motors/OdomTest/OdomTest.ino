#include <HardwareSerial.h>
#include <Arduino.h>

#define TICKS_IN_CYCLE 233
#define H1_PIN 2
#define H2_PIN 3
#define TICKS_IN_METER 0.94/1000

bool flag = false;
int ticks = 0;
bool last_h2_state = false;
float m = 0.0;

void setup(){
    Serial.begin(115200);
    last_h2_state = digitalRead(H2_PIN);
}

void loop(){
    bool state = digitalRead(H1_PIN);
    if (state && !flag) {   // фронт LOW-HIGH
        flag = true;
    }
    if (!state && flag) {   // фронт HIGH-LOW
        flag = false;
    }

    bool new_h2_state = digitalRead(H2_PIN);
    if (state && !last_h2_state && new_h2_state){
        ticks++;
    }

    if(state && last_h2_state && !new_h2_state){
        ticks--;
    }

    m = ticks*TICKS_IN_METER;

    last_h2_state = new_h2_state;

    Serial.print("ticks=");
    Serial.print(ticks);
    Serial.print(" m=");
    Serial.println(m);

    // 233 ticks == 1 оборот - замер
    // 69.60*3.14= 218.54 mm длина окружности - рассчёт
    // 1 tick == 218.54/2s33 = 0.94 mm
}