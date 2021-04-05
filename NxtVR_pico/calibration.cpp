/*
 * This calibration program is based upon the Luis Ródenas <luisrodenaslorda@gmail.com> arduino sketch, so all of the logic and math behind the code 
 * belongs to him. This program has been modified to work with our own way of working with the mpu6050
 * TODO: Clean the code so it's easier to read!
*/

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "sensors/mpu6050/mpu6050.h"
#include "stdint.h"
#include "iostream"
#include "cmath"
using namespace std;
MPU6050 mpu6050;



int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int16_t bias[6] = {0,0,0,0,0,0};


void meansensors(){
   long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

   while (i<(buffersize+101)){
      // read raw accel/gyro measurements from device
      mpu6050.readAccel(mpu6050.accel);
      mpu6050.readGyro(mpu6050.gyro);

      if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
         buff_ax=buff_ax+mpu6050.accel[0];
         buff_ay=buff_ay+mpu6050.accel[1];
         buff_az=buff_az+mpu6050.accel[2];
         buff_gx=buff_gx+mpu6050.gyro[0];
         buff_gy=buff_gy+mpu6050.gyro[1];
         buff_gz=buff_gz+mpu6050.gyro[2];
      }
      if (i==(buffersize+100)){
         mean_ax=buff_ax/buffersize;
         mean_ay=buff_ay/buffersize;
         mean_az=buff_az/buffersize;
         mean_gx=buff_gx/buffersize;
         mean_gy=buff_gy/buffersize;
         mean_gz=buff_gz/buffersize;
      }
      i++;
      sleep_ms(2); //Needed so we don't get repeated measures
   }
}
void calibration(){
  bias[0]=-mean_ax/8;
  bias[1]=-mean_ay/8;
  bias[2]=(16384-mean_az)/8;

  bias[3]=-mean_gx/4;
  bias[4]=-mean_gy/4;
  bias[5]=-mean_gz/4;
  while (1){
    int ready=0;

    mpu6050.configure(bias);
    meansensors();
    cout << "..." << endl;
    cout << mean_ax << endl;
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else bias[0]=bias[0]-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else bias[1]=bias[1]-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else bias[2]=bias[2]+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else bias[3]=bias[3]-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else bias[4]=bias[4]-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else bias[5]=bias[5]-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

int main()
{
   char message;
   stdio_init_all();
   i2c_init(I2C_PORT, 400 * 1000);
   gpio_set_function(4, GPIO_FUNC_I2C);
   gpio_set_function(5, GPIO_FUNC_I2C);
   gpio_pull_up(4);
   gpio_pull_up(5); 
   mpu6050.begin();

   cout << "Send any character to start!" << endl;
   cout << "\nMPU6050 Calibration Sketch";
   sleep_ms(2000);
   cout << "\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n";
   sleep_ms(3000);

   mpu6050.configure(bias);

   while(1)
   { 
      if (state==0){
         cout << "\nReading sensors for first time...";
         meansensors();
         state++;
         sleep_ms(1000);
      }

      if (state==1) {
         cout << ("\nCalculating offsets...");
         calibration();
         state++;
         sleep_ms(1000);
      }

      if (state==2) {
         meansensors();
         cout << "\nFINISHED!";
         cout << "\nSensor readings with offsets:\t";
         cout << mean_ax; 
         cout << "\t";
         cout << mean_ay; 
         cout << "\t";
         cout << mean_az; 
         cout << "\t";
         cout << mean_gx; 
         cout << "\t";
         cout << mean_gy; 
         cout << "\t";
         cout << mean_gz << endl;
         cout << "Your offsets:\t";
         cout << bias[0]; 
         cout << "\t";
         cout << bias[1]; 
         cout << "\t";
         cout << bias[2]; 
         cout << "\t";
         cout << bias[3]; 
         cout << "\t";
         cout << bias[4]; 
         cout << "\t";
         cout << bias[5]; 
         cout << "\nData is printed as: acelX acelY acelZ giroX giroY giroZ" << endl;
         cout << "Check that your sensor readings are close to 0 0 16384 0 0 0" << endl;
         cout << "If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)" << endl;
         while (1);
      }

   }

   return 0;
}



