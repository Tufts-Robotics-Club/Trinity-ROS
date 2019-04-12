/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include "arduinoFFT.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

bool fire_on = false;

void messageCb( const std_msgs::Empty& toggle_msg){
  fire_on = not fire_on;
}

ros::Subscriber<std_msgs::Empty> sub("servo_toggle", messageCb );

std_msgs::String str_msg;
std_msgs::Bool sound_start_msg;
ros::Publisher chatter("flame_sensors", &str_msg);
ros::Publisher start("start", &sound_start_msg);

arduinoFFT FFT = arduinoFFT(); 
#define CHANNEL A0
#define LOW_FREQ_THRESH 3900
#define HIGH_FREQ_THRESH 4600
#define samples 128
#define samplingFrequency 9500

void updateSoundStart(){
    unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
    unsigned long microseconds;
    double vReal[samples];
    double vImag[samples];
    for(int i=0; i<samples; i++)
    {
        microseconds = micros();
        vReal[i] = analogRead(CHANNEL);
        vImag[i] = 0;
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    if (x > LOW_FREQ_THRESH and x < HIGH_FREQ_THRESH) {
      sound_start_msg.data = true;
    }
}

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(start);
  nh.subscribe(sub);
  sound_start_msg.data = false;
}

void servo_move() {
  if (fire_on == true){
    
  } else {
    
  }
}


void update_firesensors(){
  char temp[50];
  sprintf(temp, "%04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\0", analogRead(A1),
  analogRead(A2),analogRead(A3),analogRead(A4),analogRead(A5),analogRead(A6),analogRead(A7),
  analogRead(A8),analogRead(A9),analogRead(A10));
  str_msg.data = temp;
}


void loop()
{
  if (sound_start_msg.data == false){
    updateSoundStart();
  }
  
  servo_move();
  update_firesensors();
  chatter.publish(&str_msg);
  start.publish(&sound_start_msg);
  nh.spinOnce();
}
