#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial mySerial(2, 3); // RX, TX
 // RX, TX



#define pos_x    0
#define pos_y     1

#define PI 3.1415926535897932384626433832795





//Initialize global variables
String bluetoothRead;
unsigned short x, y, area;
unsigned short strLength;
int contador= 0;
int ledPin = 8;
double target_x;
double target_y;

float instruction[2];
float errors[2];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[2]      = {0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[2] = {0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float measures[2] = {0, 0};
float posx_pid;
float posy_pid;
double cosYaw;
double sinYaw;
double desiredRoll;
double desiredPitch;
double heading;
double angle;
double g=9.81;
double yaw=125;


float minMax(double value, double min_value, double max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

void calculateErrors() {
    errors[pos_x] = instruction[pos_x] - measures[pos_x];
    errors[pos_y]  = instruction[pos_y]  - measures[pos_y];
}

void resetPidController()
{
    errors[pos_x]   = 0;
    errors[pos_y] = 0;

    error_sum[pos_x]   = 0;
    error_sum[pos_y] = 0;
    
    previous_error[pos_x]   = 0;
    previous_error[pos_y] = 0;
}

void pidController() {
    float Kp[2]        = {1, 1};    // P coefficients in that order : x, y
    float Ki[2]        = {0.1, 0.1}; // I coefficients in that order : x, y
    float Kd[2]        = {3, 3};        // D coefficients in that order : x, y
    float delta_err[2] = {0, 0};          // Error deltas in that order   : x, y

    // Calculate sum of errors : Integral coefficients
    error_sum[pos_x]   += errors[pos_x];
    error_sum[pos_y] += errors[pos_y];

    // Calculate error delta : Derivative coefficients
    delta_err[pos_x]   = errors[pos_x]   - previous_error[pos_x];
    delta_err[pos_y] = errors[pos_y] - previous_error[pos_y];
    // Save current error as previous_error for next time
    previous_error[pos_x]   = errors[pos_x];
    previous_error[pos_y] = errors[pos_y];


    // PID = e.Kp + ∫e.Ki + Δe.Kd
    posx_pid  = (errors[pos_x]*Kp[pos_x]) + (error_sum[pos_x]*Ki[pos_x]) + (delta_err[pos_x]*Kd[pos_x]);
    posy_pid = (errors[pos_y]*Kp[pos_y]) + (error_sum[pos_y]*Ki[pos_y]) + (delta_err[pos_y]*Kd[pos_y]);

}



void setup() {

  Serial.begin(9600);
  Serial.println("OpenCV Bot");

  mySerial.begin(9600);
  resetPidController();
  pinMode(ledPin, OUTPUT); 
}

void loop() {
  //Serial.println(contador);
  register byte ndx = 0;
  char commandbuffer[50];

  if (mySerial.available() > 0) {
    if(contador==0){
      delay(3000);
    }
    delay(10);
    while ( mySerial.available() && ndx < 50) {
      commandbuffer[ndx++] = mySerial.read();
    }
    
    commandbuffer[ndx] = '\0';
    bluetoothRead = (char*)commandbuffer;
    strLength = bluetoothRead.length();

    if (bluetoothRead.substring(0, 1).equals("X")) {
      uint8_t pos, i = 1;

      while (bluetoothRead.substring(i, i + 1) != ("Y")) {
        i++;
      }

      x = bluetoothRead.substring(1, i).toInt();

      pos = i + 1;
      while (bluetoothRead.substring(i, i + 1) != ("A")) {
        i++;
      }

      y = bluetoothRead.substring(pos, i).toInt();

      area = bluetoothRead.substring(i + 1, strLength).toInt();
      if(contador == 0){
        instruction[pos_x]=x;
        instruction[pos_y]=y;
        contador=1;
      }
      else{
        measures[pos_x]=x;
        measures[pos_y]=y;
        //Serial.println(measures[pos_y]);
        
        calculateErrors();
        pidController();

        
//        angle= atan2(instruction[pos_x] - measures[pos_x],instruction[pos_y] - measures[pos_y])*180/PI;       
//        heading=angle-yaw;
//        heading=heading*PI/180; 

        heading=yaw*PI/180;
        //Serial.println(errors[pos_x]);
        //Serial.println(errors[pos_y]);
        cosYaw = cos(heading);
        sinYaw = sin(heading);
        desiredRoll  = (PI/180)*((posx_pid*sinYaw + posy_pid*cosYaw)*(1/g));
        desiredPitch  = (PI/180)*((posx_pid*cosYaw - posy_pid*sinYaw)*(1/g));
        desiredRoll = minMax(desiredRoll, -15, 15);
        desiredPitch = minMax(desiredPitch, -15, 15);
        Serial.print("Roll:");
        //Serial.println(desiredRoll);
        //Serial.print("Pitch:");
        //Serial.println(desiredPitch);
        delay(10);
               
      }
//      if (x < 750) {
//        Serial.println("Left");
//        //Left();
//      }
//      if (x >  1100 ) {
//        Serial.println("Right");
//        //Right();
//      }
//      if (x >= 750 && x <= 1100) {
//        if (area < 150) {
//          Serial.println("Forward");
//          //Forward();
//        }
//        if (area >= 150 && area <= 250) {
//          Serial.println("Stop");
//          //Stop();
//        }
//        if (area > 250) {
//          Serial.println("Back");
//          //Back();
//        }
//      }
    }
  }
  // all data has been sent, and the buffer is empty.
  Serial.flush();
}




