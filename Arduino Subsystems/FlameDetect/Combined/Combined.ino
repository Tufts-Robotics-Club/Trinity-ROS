#include "arduinoFFT.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

// ------------------------ MAIN CONSTANTS -----------------------------
const int LOW_FREQ_THRESH = 3900;
const int HIGH_FREQ_THRESH = 4600;

// ------------------------ FFT GLOBALS ---------------------------------
arduinoFFT FFT = arduinoFFT(); 
#define CHANNEL A0
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9500; //Hz, must be less than 10000 due to ADC

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// ------------------- ROS SERIAL GLOBALS AND FUNCTIONS---------------------------
ros::NodeHandle  nh;

std_msgs::Int32MultiArray flame_sensor_msg;
std_msgs::Bool sound_start_msg;

ros::Publisher pub_flame("flame_sensors", &flame_sensor_msg);
ros::Publisher pub_sound("sound_start", &sound_start_msg);

void extinguish_motor_callback(const std_msgs::Bool& msg) {
    if (msg.data == true) {
//digitalWrite(13, HIGH-digitalRead(13));
    }
    else {
//do something
    }
}

ros::Subscriber<std_msgs::Bool> sub1("extinguish_flame", &extinguish_motor_callback);

// ------------------ SOUND FUNCTIONS --------------------------------

//Record some sound to see if it is the right frequency
void updateSoundStart(){
    
    unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
    unsigned long microseconds;
    
    double vReal[samples];
    double vImag[samples];
/*SAMPLING*/
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
        Serial.println("Start");
        //Update sound msg
    }
}

//Check the sound message to see if it is on and turn on the LED
void checkSoundStart(){

}


// ------------------ FLAME FUNCTIONS --------------------------------

//Read pins to see what flame sensors are on
void updateFlameSensorMsg(){

}



//Check the flame message to see if any is on and turn on the LED
void checkFlameDetected(){

}


void setup()
{
//ROS SETUP
    nh.initNode();
    nh.advertise(pub_flame);  
    nh.advertise(pub_sound);
}

void loop()
{
    updateFlameSensorMsg();
    checkFlameDetected();
    
    updateSoundStart();
    checkSoundStart();
    
    pub_flame.publish(&flame_sensor_msg);
    pub_sound.publish(&sound_start_msg);
    
    nh.spinOnce(); 
    delay(50);
}
