//LIBRARIES
#include "simpletools.h"                      // Include simple tools
#include "simplei2c.h"



//  PIN ASSIGNMENTS //
//IMU I2C
#define SCL 12
#define SDA 11

//H BRIDGE
#define PWML 6
#define PWMR 0
#define AIN1 2
#define AIN2 1
#define STBY 3
#define BIN1 4
#define BIN2 5

//ULTRASONIC SENSOR
#define ULT_ECHO 8





//I2C CONSTANTS
#define ACCEL_READ 0x3B
#define ACCEL_WRITE 0x3A
#define GYRO_READ 0xD7
#define GYRO_WRITE 0xD6

#define RAD2DEG (180./3.141592)

//GLOBAL ANGLE VARIABLES
volatile float accel_x_angle = 0;
volatile float accel_y_angle = 0;
volatile float accel_z_angle = 0;

//GLOBAL HEIGHT VARIABLE
volatile float ult_height = 0;

//MOTOR CONTROL VARIABLES
volatile float dutyCycleLeft = 0;
volatile float dutyCycleRight = 0;

//PID CONSTANTS
//ANGLE CONSTANTS
#define kpa 0.7
#define kia 0.3
#define kda 0.4
//#define kpa .7
//#define kia 0.3
//#define kda 0.4
//HEIGHT CONSTANTS
#define kph 5
#define kih 2.5
#define kdh 2


//Starting PWM Value (60%)
#define initialValue 600
//Starting Target Height
#define hover_height 90




volatile float deltaT = 0;

//KALMAN FILTER CONSTANTS
// Process noise covariance for the accelerometer - (w = process noise)
#define Q_angle .001
// Process noise covariance for the gyro bias - (w = process noise)
#define Q_gyroBias 0.003
// Measurement noise covariance - this is actually the variance of the measurement noise - (v = measurement noise)
#define R_angle .03

//KALMAN FILTER VARIABLES
volatile float Kalman_x = 0;
volatile float Kalman_y = 0;
volatile float Kalman_z = 0;
float P00 = 1; // Error covariance matrix - This is a 2x2 matrix
float P01 = 0;
float P10 = 0;
float P11 = 1;
// Since we don't know the initial angle and bias we set it like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
double K[2]; // Kalman gain - This is a 2x1 matrix
double y; // Angle error - 1x1 matrix
double S; // Estimate error - 1x1 matrix
double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

//KALMAN FILTER CALCULATION METHOD
double getAngle(double newAngle, double newRate, double dt) {            
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/08/a-practical-approach-to-kalman-filter-and-how-to-implement-it           
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        rate = newRate - bias;
        angle += dt * rate;        
        // Update estimation error covariance - Project the error covariance ahead
        P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
        P01 -= dt * P11;
        P10 -= dt * P11;
        P11 += Q_gyroBias * dt;        
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        S = P00 + R_angle;
        K[0] = P00 / S;
        K[1] = P10 / S;        
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;        
        // Calculate estimation error covariance - Update the error covariance
        P00 -= K[0] * P00;
        P01 -= K[0] * P01;
        P10 -= K[1] * P00;
        P11 -= K[1] * P01;
        
        return angle;
}

//I2C IMU COMMUNICATION & ANGLE CALCULATIONS
void readIMU() {
  int ax =0;
  int ay =0;
  int az =0;
  int wx = 0;
  int wy = 0;
  int wz = 0;
  
  int current = 0;
  
  i2c my_i2c;
  i2c_open(&my_i2c,SCL,SDA,0);
  // Start Accelerometer
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,ACCEL_WRITE);
  i2c_writeByte(&my_i2c, 0x20); // CTRL1 register
  i2c_writeByte(&my_i2c, 0x77);
  i2c_stop(&my_i2c);
  unsigned char accel_bytes[6];  
  // Start Gyroscope
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,GYRO_WRITE);
  i2c_writeByte(&my_i2c, 0x20); // CTRL1 register
  i2c_writeByte(&my_i2c, 0x3f); // page 36-37
  i2c_stop(&my_i2c);
  // Specify Gyro Scale
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,GYRO_WRITE);
  i2c_writeByte(&my_i2c, 0x23); // CTRL4 register
  i2c_writeByte(&my_i2c, 0x20); // page 39
  i2c_stop(&my_i2c);
  unsigned char gyro_bytes[6];
  
  //float err_integral = 0;
  float last_e = 0;
  //float e_height_integral = 0;
  float last_e_height = 0;
  
  unsigned int looptime = 0;
  unsigned int startTime = CNT;
  unsigned int endTime = 0;
 // volatile float deltaT = 0;
 
  float e_height_integral = 0;
  float e_height = 0;
  float err_integral = 0;
  float e = 0;
  
  while(1) { 
    // ACCELEROMETER
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,ACCEL_WRITE);
    i2c_writeByte(&my_i2c, 0x28 + 0x80); //OUT_X_L_A from page 29 of Accelerometer data sheet
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,ACCEL_READ);
    i2c_readData(&my_i2c,accel_bytes,6);
    i2c_stop(&my_i2c);
    
    //Transfer Values to Local Variables
    current = accel_bytes[0];
    ax = current;
    current = accel_bytes[1];
    ax += current << 8;
    if(ax & (1 << 15)) ax -= (1 << 16);    
    current = accel_bytes[2];
    ay = current;
    current = accel_bytes[3];
    ay += current << 8;
    if(ay & (1 << 15)) ay -= (1 << 16);    
    current = accel_bytes[4];
    az = current;
    current = accel_bytes[5];
    az += current << 8;
    if(az & (1 << 15)) az -= (1 << 16);
    
    //Convert to angles
    accel_x_angle = (atan2(((float)ay),((float)az)))*RAD2DEG;
    ////accel_y_angle = (atan2(((float)ax*-1),((float)az)))*RAD2DEG;
    //accel_z_angle = (atan2(((float)ax),((float)ay)))*RAD2DEG;
    
    // GYROSCOPE
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,GYRO_WRITE);
    i2c_writeByte(&my_i2c, 0x28 + 0x80); //OUT_X_L from page 34 of gyro data sheet
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,GYRO_READ);
    i2c_readData(&my_i2c,gyro_bytes,6);
    i2c_stop(&my_i2c);
    //Transfer Values to Local Variables
    wx = gyro_bytes[0];
    wx += gyro_bytes[1] << 8;
    if(wx & (1 << 15)) wx -= (1 << 16);    
/*    current = gyro_bytes[2];
    wy = current;
    current = gyro_bytes[3];
    wy += current << 8;
    if(wy & (1 << 15)) wy -= (1 << 16);    
    current = gyro_bytes[4];
    wz = current;
    current = gyro_bytes[5];
    wz += current << 8;
    if(wz & (1 << 15)) wz -= (1 << 16);
  */  
    //Integrate to calculated angles
    //gyro_x_angle += (float)wx*0.07 * deltaT;
    //gyro_y_angle += (float)wy*0.07 * deltaT;
    //gyro_z_angle += (float)wz*0.07 * deltaT;
    //Limit to within +/- 180 degrees
    //if(gyro_x_angle > 180) gyro_x_angle -= 360;
    //if(gyro_y_angle > 180) gyro_y_angle = -180;
    //if(gyro_z_angle > 180) gyro_z_angle = -180;
    //if(gyro_x_angle < -180) gyro_x_angle +=360;
    //if(gyro_y_angle < -180) gyro_y_angle = 180;
    //if(gyro_z_angle < -180) gyro_z_angle = 180;
    
    //Perform Kalman Calculations on angle values
    Kalman_x  = getAngle(accel_x_angle, wx * 0.07, deltaT);
    //Kalman_y  = getAngle(accel_y_angle, wy * 0.07, deltaT);
    //Kalman_z  = getAngle(accel_z_angle, wz * 0.07, deltaT);
    
    //Height PID
    e_height = hover_height - ult_height;
    e_height_integral += e_height*deltaT;
    if(e_height_integral > 100) e_height_integral = 100;
    if(e_height_integral < -100) e_height_integral = -100;
    float height_pid = kph*e_height + kih*e_height_integral + kdh*(e_height-last_e_height)/deltaT;
    last_e_height = e_height;
    
    //Angle PID
    e = Kalman_x;
    err_integral += e*deltaT;
    if(err_integral > 1000) err_integral -= 1000;
    if(err_integral < -1000) err_integral += 1000;
    float angle_pid = kpa*e + kia*err_integral + kda*(e-last_e)/deltaT;
    last_e = e;
    
    //Duty Cycle Calculations
    dutyCycleLeft = initialValue - height_pid - angle_pid;
    dutyCycleRight = initialValue + 10 - height_pid + angle_pid;
    
    //Duty Cycle capping
    if(dutyCycleLeft > 1000) dutyCycleLeft = 1000;
    if(dutyCycleLeft < 0) dutyCycleLeft = 0;
    if(dutyCycleRight > 1000) dutyCycleRight = 1000;
    if(dutyCycleRight < 0) dutyCycleRight = 0;
    
    
    endTime = CNT;
    looptime = endTime - startTime;
    startTime = endTime;
    deltaT = looptime / 100000000.;

    }  
}

void pwmLeft() {
  CTRA = 0x10000000;
  CTRA += PWML;
  FRQA = 1;
  low(PWML);
  unsigned endTime = CNT;
  while(1) {
    endTime += 10000;
    PHSA = - (int)(dutyCycleLeft * 10);
    waitcnt(endTime);
  }    
}

void pwmRight() {
  CTRA = 0x10000000;
  CTRA += PWMR;
  FRQA = 1;
  low(PWMR);
  unsigned endTime = CNT;
  while(1) {
    endTime += 10000;
    PHSA = - (int)(dutyCycleRight * 10);
    waitcnt(endTime);
  }    
}

void ultrasonic() {
  input(ULT_ECHO);
  unsigned timer = CNT;
  pause(175);
  while(1) {
    waitpeq(1 << ULT_ECHO, 1 << ULT_ECHO);
    timer = CNT;
    waitpeq(0x00, 1 << ULT_ECHO);
    ult_height = (CNT - timer) / 5800.;
    if(ult_height > 125) ult_height = 125;
    
  }    
}  


int main()                                    // Main function
{
  // Add startup code here.
  set_direction(STBY,1);
  set_direction(PWML,1);
  set_direction(PWMR,1);
  high(STBY);
  high(AIN1);
  low(AIN2);
  high(BIN1);
  low(BIN2);
  cog_run(ultrasonic, 128);
  
  pause(300);
  
  cog_run(readIMU, 256);
  cog_run(pwmLeft, 128);
  cog_run(pwmRight, 128);
 
  while(1)
  {
    // Add main loop code here.
    print("ULT:%2.6f, DCR:%2.1f, DCL:%2.1f\n", ult_height, dutyCycleRight, dutyCycleLeft);
    //print("%2.1f\n", ult_height);
    pause(200);
  }  
}
