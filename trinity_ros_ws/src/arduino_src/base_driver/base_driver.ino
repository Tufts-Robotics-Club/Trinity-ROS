#include "DualVNH5019MotorShield.h"
#include <PID_v1.h>

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
double kp = 70 , ki = 15, kd = 10;
// double kp = 70 , ki = , kd = 10;

double inputr = 0, outputr = 0, setpointr = 0;
double inputl = 0, outputl = 0, setpointl = 0;

PID velRPid(&inputr, &outputr, &setpointr, kp, ki, kd, DIRECT);
PID velLPid(&inputl, &outputl, &setpointl, kp, ki, kd, DIRECT);

DualVNH5019MotorShield md;

void setup() {
    Serial.begin(115200);
    md.init();
    attachInterrupt(digitalPinToInterrupt(18),lencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(19),lencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(20),rencoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(21),rencoder,CHANGE);
    velRPid.SetMode(AUTOMATIC);
    velRPid.SetSampleTime(1);
    velRPid.SetOutputLimits(-400, 400);
    velLPid.SetMode(AUTOMATIC);
    velLPid.SetSampleTime(1);
    velLPid.SetOutputLimits(-400, 400);
}

void loop() {
    index++;
    if (index >= 40){
        change_el = lenc_count - old_cl;
        change_er = renc_count - old_cr;
        dt = micros() - old_t;
        old_cl = lenc_count;
        old_cr = renc_count;
        old_t = micros();
        vel_l = (change_el * 6413.900601)/dt;
        vel_r = (change_er * 6413.900601)/dt;
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
            temp = (double)1000 * vel_l;
            enc[0] = (temp & 0xff);
            enc[1] = (temp & 0xff00) >> 8;
            temp = (double)1000 * vel_r;
            enc[2] = (temp & 0xff);
            enc[3] = (temp & 0xff00) >> 8;
            Serial.write(enc, 4);
        } else {
            motor_two = cmd[0] << 4;
            motor_two |= ((cmd[1] & 0xf0) >> 4);
            motor_one = (cmd[1] & 0x0f) << 7;
            motor_one |= (cmd[2] & 0xfe) >> 1;
            setpointr = ((int)motor_one - 1024)/10;
            setpointl = ((int)motor_two - 1024)/10;
        }
    } 
    inputl = vel_l;
    inputr = vel_r;
    velLPid.Compute();
    velRPid.Compute();
    md.setM2Speed(outputl);
    md.setM1Speed(outputr);
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