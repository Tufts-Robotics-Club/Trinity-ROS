#include "DualVNH5019MotorShield.h"

static const int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
const size_t data_size = 3;
const char IS_ENCODER = 0x80;
unsigned int motor_one = 400, motor_two = 400;
volatile long lenc_count = 0;
volatile long renc_count = 0;
unsigned char cmd[data_size];
unsigned char enc[4];

long old_cl = 0;
long old_cr = 0;
long old_t = 0;
long dt = 0;
double change_el = 0;
double change_er = 0;
double vel_l = 0;
double vel_r = 0;
int index = 0;
uint16_t temp;

DualVNH5019MotorShield md;

void setup() {
    Serial.begin(115200);
    md.init();
    attachInterrupt(digitalPinToInterrupt(18),lencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(19),lencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(20),rencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(21),rencoder,CHANGE);
}

void loop() {
    index++;
    if (index >= 35){
        change_el = lenc_count - old_cl;
        change_er = renc_count - old_cr;
        dt = micros() - old_t;
        old_cl = lenc_count;
        old_cr = renc_count;
        old_t = micros();
        vel_l = (change_el * 6413900.601)/dt;
        vel_r = (change_er * 6413900.601)/dt;
        index = 0;
    }
    if (Serial.available() > 0) {
        Serial.readBytes(cmd, data_size);
        if (cmd[0] & IS_ENCODER) {
            enc[0] = (lenc_count & 0xff);
            enc[1] = (lenc_count & 0xff00) >> 8;
            enc[2] = (lenc_count & 0xff0000) >> 16;
            enc[3] = (lenc_count & 0xff000000) >> 24;
            Serial.write(enc, 4);
            enc[0] = (renc_count & 0xff);
            enc[1] = (renc_count & 0xff00) >> 8;
            enc[2] = (renc_count & 0xff0000) >> 16;
            enc[3] = (renc_count & 0xff000000) >> 24;
            Serial.write(enc, 4);
            temp = vel_l;
            enc[0] = (temp & 0xff);
            enc[1] = (temp & 0xff00) >> 8;
            temp = vel_r;
            enc[2] = (temp & 0xff);
            enc[3] = (temp & 0xff00) >> 8;
            Serial.write(enc, 4);
        } else {
            motor_one = cmd[0] << 3;
            motor_one |= ((cmd[1] & 0xe0) >> 5);
            motor_two = (cmd[1] & 0x1f) << 5;
            motor_two |= (cmd[2] & 0xf8) >> 3;
        }
    }
    md.setM2Speed(motor_two - 400);
    md.setM1Speed(motor_one - 400);
}
void lencoder()
{
    static uint8_t enc_val = 0;
    enc_val = enc_val << 2;
    enc_val = enc_val | ((PIND & 0b1100) >> 2);
 
    lenc_count = lenc_count + lookup_table[enc_val & 0b1111];
}

void rencoder()
{
    static uint8_t enc_val = 0;
    enc_val = enc_val << 2;
    enc_val = enc_val | (PIND & 0b0011);
 
    renc_count = renc_count + lookup_table[enc_val & 0b1111];
}
