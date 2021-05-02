#include <Arduino.h>
#include <Wire.h>
#include <pwmstm.h>
#include <mpul.h>
#include <util.h>
#include "SPI.h"
//#include "HardwareSerial.h"
#include <VL53L0X.h>
#include <altitude_kf.h>

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging








int accel_range = 16;// ACC_FULL_SCALE_16_G;
int gyro_range = 500; //GYRO_FULL_SCALE_500_DPS;
int mag_bits = 16;

int accel_dlpf = 184; // 0, 41, 92, 184, 460
int gyro_dlpf = 184; // 0, 41, 92, 184, 460


// Linear position
float x[3]; // x, y, z
// Linear velocity
float v[3]; // vx, vy, vz
float euler_angles[3]; // yaw, Pi2tch, roll (3-2-1)
float a[3], g[3], m[3]; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method



// Specify sensor full scale
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float accel_res, gyro_res, mag_res;      // scale resolutions per LSB for the sensors

float magCalibration[3] = {0, 0, 0},magbias[3] = {154.62f,  318.39f,  -610.13f}, magbias2[3] = {1.0f, 1.0f, 1.0f};  // Factory mag calibration and mag bias
//magbias[0] = 144.96f;//+249.2; // +609.360; //+666.715; //+470.0;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//magbias[1] = 218.48f;//+83.7; // +447.380; //+452.795; //+120.0;  // User environmental y-axis correction in milliGauss
//magbias[2] = -337.47f;//-489.63; //-400.095; //-412.265; //+125.0;  // User environmental z-axis correction in milliGauss


float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = 3.14159f * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = 3.14159f * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a Pi2D control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError*1.2; //0.4
// compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Kp 25.0f //25
#define Ki 0.25f //0.25


float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0;//, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval




const unsigned long timeout_limit = 100000;
const float deg2rad = 3.14159f / 180.0f;


bool encendido=false;
char valserial=0;

float motores[4];

float maltura = 0.0f;
float altura= 0;
int set_altura = 0;
int condicion_altura = 1;
int set_alturaini = 0;
float alturapas=0;

#define larg_fil 20
float alturas[larg_fil];// = {0,0,0,0,0};
float altura_filt = 0.0f;



float error_p_a = 0;
float error_i_a = 0;
float error_d_a = 0;
float error_a = 0;
float error_pas_a = 0;

float kp_a = 1.5f;
float kd_a = 1.0f;
float ki_a = 0.0f;//0.0001f;

int set_angles[3] = {0, 0, 0};
float errores[3] = {0.0f, 0.0f, 0.0f};
float mot_angles[3] = {0.0f, 0.0f, 0.0f};

float error_p[3] = {0.0f, 0.0f, 0.0f};
float error_i[3] = {0.0f, 0.0f, 0.0f};
float error_d[3] = {0.0f, 0.0f, 0.0f};

float error_pas[3] = {0.0f, 0.0f, 0.0f};

//float kp[3] = {0.0f, 0.1f, 0.1f};
//float kd[3] = {0.0f, 0.05f, 0.008f};//original
//float ki[3] = {0.0f, 0.00012f, 0.0001f};


//float kp[3] = {0.0f, 0.01f, 0.15f};
//float kd[3] = {0.0f, 0.0f, 0.0001f};//mejorado antes de que se fuera
//float ki[3] = {0.0f, 0.000f, 0.0001f};


float kp[3] = {0.1f, 0.35f, 0.3f};
float kd[3] = {0.0f, 0.5f, 0.5f};//bajar el último
float ki[3] = {0.0f, 0.0001f, 0.0001f};

#define larg_filyaw 30
float yaw_arr[larg_filyaw];
float yaw_filt = 0;


int lalt = 0;
int escalon = -1;

VL53L0X sensor;
Altitude_KF alt_estimator(0.2, 0.1);//0.5 0.1/ 0.2

void setup() {
  //Wire.setClock(500000);
  Wire.begin();
  Wire.setClock(400000);
 
  Serial.begin(115200);

  Serial.println("Listo");

  while(valserial!='c' & valserial!='C'){
     valserial=Serial.read();
    }

  Serial.println("Calibrando");
  
  
  
   
  setup_mpu9250(accel_range, gyro_range, mag_bits, gyro_dlpf, accel_dlpf);
  if(valserial=='C'){
    Serial.println("Calibrando Magnetometro, hacer 8s");
    magcalMPU9250(magbias,magbias2);
  }
  
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  
  pinMode(PE0,INPUT_PULLUP); //interrupcion MPU
  pinMode(PB3,INPUT_PULLUP); //interrupcion VLX
  
  pinMode(PE1, OUTPUT); // motores
  
  pinMode(PB15, OUTPUT);//loop mpu
  pinMode(PD8, OUTPUT); //loop altura
  
  
  PWM_init(400);

  Serial.print("beta: ");
  Serial.println(beta);

  Serial.print("zeta: ");
  Serial.println(zeta);
  
  Serial.println("Esperando encendido");
  while(!encendido){
     valserial=Serial.read();
      if(valserial=='e'){
      encendido=1;
      Serial.println("Encendiendo"); 
    }
    delay(10);
    
    //Serial.println(valserial);
//    read_mpu9250();
//    updateState();
//    serialPrintArray3(euler_angles);
  }


  Serial.println("Realizando convergencia");

  encendido=0;
  while(!encendido){
     valserial=Serial.read();
      if(valserial=='e'){
      encendido=1;
      }
  for(int be=0;be<100;be++){
    while(!digitalRead(PE0)){}
      read_mpu9250();
      updateState();
  }
  Serial.println(euler_angles[0]);
  delay(100);
  }
  
  Serial.print("Yaw inicial: ");
   Serial.println(euler_angles[0]);
  set_angles[0] = euler_angles[0];

 delay(100);


//  int promedioalt=0;
//  for(uint8_t iter=0;iter<20;iter++){
//    promedioalt += sensor.readRangeContinuousMillimeters();
//    delay(100);
//  }
   set_alturaini = 420;// promedioalt/20;
   alturapas = set_alturaini;
   Serial.print("Altura inicial: ");
   Serial.println(set_alturaini);
  
 
// calibrar();
  armar();

  Serial.println("All systems on line");
  
//  enumerar();
  delay(1000);

encendido=0;
  while(!encendido){
     valserial=Serial.read();
      if(valserial=='l'){
      encendido=1;
      Serial.println("comenzando"); 
    }
    delay(10);
  }


delay(500);

  
for(int au=0;au<2160;au++){
  motoreson(au);
  delay(1);
}

Serial.println("Loop");
  
}

void loop() {

    while(lalt<10){
      if(digitalRead(PE0) & encendido){
        GPIOB->ODR ^= (1<<15);
        read_mpu9250();
        updateState();
        
        filtrar_yaw();
        
        PIDs();
      }
      lalt++;
    }

    lalt=0;
   
      if(!digitalRead(PB3)){
        GPIOD->ODR ^= (1<<8);
        alturapas = sensor.readRangeContinuousMillimeters();
        if(alturapas<1500 && alturapas>50){
          //altura = alturapas; 
          alt_estimator.update(alturapas);
          
          //Serial.println(alt_estimator.h);
          //Serial.println(error_p[0]);//altura_filt
          
        }
        else{
          //Serial.println(alturapas);
        }
      }
      altura = alt_estimator.h;
      filtrar();
      
      PIDa();

    
    if(set_altura<set_alturaini && condicion_altura){
      set_altura++;
      Serial.println(set_altura);
      //delay(2);
    }
    else{
      condicion_altura = 0;
    }
    
    
     

    
  //angulos();
  
  valserial=Serial.read();
  if(valserial=='a'){
    apagar();
    encendido=0;
    Serial.println("Apagando");
    }
  
  if(valserial=='+'){
    //kd[2]+=0.001f;
    set_altura+=20;
    Serial.println(error_a);
    
    }
    else if(valserial=='-'){
      //kd[2]-=0.001f;
      set_altura-=20;
   // Serial.println(error_a);
    }


//  serialPrintArray4(motores);
 // serialPrintArray3(a);
  
// Serial.println(errores[1]);
//Serial.println(euler_angles[0]);
  escmotores(); // escribir a los motores
  
}





void motoreson(int valor){
  m0(valor);
  m1(valor);
  m2(valor);
  m3(valor); 
}




void filtrar_yaw(){
 float promedioy = 0;
  for(int ip=0;ip<larg_filyaw-1;ip++){
    yaw_arr[ip+1] = yaw_arr[ip];
  }
  yaw_arr[0] = euler_angles[0];

  for(int ia=0;ia<larg_filyaw;ia++){
    promedioy += yaw_arr[ia];
  }

  yaw_filt = promedioy/larg_filyaw;
}


void filtrar(){
 float promedio = 0;
  for(int ip=0;ip<larg_fil-1;ip++){
    alturas[ip+1] = alturas[ip];
  }
  alturas[0] = altura;

  for(int ia=0;ia<larg_fil;ia++){
    promedio += alturas[ia];
  }

  altura_filt = promedio/larg_fil;
}

void angulos(){
  int maximo = 50;
  
  if (euler_angles[1]>maximo || euler_angles[1]<-maximo)
  {
    apagar();
    Serial.println(euler_angles[1]);
    serialPrintArray3(euler_angles);
  }

  if (euler_angles[2]>maximo || euler_angles[2]<-maximo)
  {
    apagar();
    Serial.println(euler_angles[2]);
    serialPrintArray3(euler_angles);
  }
  
}




void enumerar(){
  int en=0;
  delay(1000);
 m0(20);
 delay(1000);
 
   while(!en){
     valserial=Serial.read();
      if(valserial=='m'){
      en=1;
    }
    delay(10);
  }

 en=0;
 m0(0);
 m1(20);
delay(1000);
while(!en){
     valserial=Serial.read();
      if(valserial=='m'){
      en=1;
    }
    delay(10);
  }

 en=0;
 m1(0);
 m2(20);
 delay(1000);
 while(!en){
     valserial=Serial.read();
      if(valserial=='m'){
      en=1;
    }
    delay(10);
  }

 en=0;
 m2(0);
 m3(20);
 delay(1000);
 while(!en){
     valserial=Serial.read();
      if(valserial=='m'){
      en=1;
    }
    delay(10);
  }


m3(0);
 delay(1000);
  
}

void PIDa(){
  float Tdif=1.0f;
  
  error_p_a =  set_altura - altura_filt; //altura; 

  error_i_a += saturacion(error_p_a*Tdif,80);
  error_i_a = saturacion(error_i_a,80);
  

  error_d_a = (error_p_a - error_pas_a) / Tdif;

  error_a = saturacion(error_p_a*kp_a + error_d_a*kd_a + error_i_a*ki_a, 400 );

  maltura = error_a + 2000; //2160
  
  error_pas_a = error_p_a; 
}


void PIDs(){
  float Tdif=1.0f;

  error_p[0] = -set_angles[0] + yaw_filt; //yaw
  error_p[1] = set_angles[1] - euler_angles[1]; // roll
  error_p[2] = set_angles[2] - euler_angles[2]; // pitch

  error_i[0] += error_p[0]*Tdif;
  error_i[1] += error_p[1]*Tdif;
  error_i[2] += error_p[2]*Tdif;


  error_d[0] = (error_p[0] - error_pas[0]) / Tdif;
  error_d[1] = (error_p[1] - error_pas[1]) / Tdif;
  error_d[2] = (error_p[2] - error_pas[2]) / Tdif;

  errores[0] = saturacion(error_p[0]*kp[0] + error_d[0]*kd[0] + error_i[0]*ki[0], 8 );
  errores[1] = saturacion(error_p[1]*kp[1] + error_d[1]*kd[1] + error_i[1]*ki[1], 30 );
  errores[2] = saturacion(error_p[2]*kp[2] + error_d[2]*kd[2] + error_i[2]*ki[2], 30 );

  
  
  
  error_pas[0] = error_p[0];
  error_pas[1] = error_p[1];
  error_pas[2] = error_p[2];
  
}



float saturacion(float variable, int sat){
  if(variable>sat){
    variable = sat;
    //Serial.println('S');
  }
  else if(variable<-sat){
    variable = -sat;
    //Serial.println('S');    
  }
  return variable;
}







void apagar(){
  m0(0);
  m1(0);
  m2(0);
  m3(0);
  delay(500);
  GPIOE->ODR &= ~(1<<1); 
}

void escmotores(){
  float multia=30.0f;
  
  motores[0] = maltura +(- errores[2] - errores[1] + errores[0])*multia;
  motores[1] = maltura +(+ errores[2] - errores[1] - errores[0])*multia;
  motores[2] = maltura +(- errores[2] + errores[1] - errores[0])*multia;
  motores[3] = maltura +(+ errores[2] + errores[1] + errores[0])*multia;
  
  m0(motores[0]);
  m1(motores[1]);
  m2(motores[2]);
  m3(motores[3]); 
}

void updateState() //
{

  float pitch, yaw, roll;
  

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;



for(uint8_t it=0;it<15;it++){//20

//  MadgwickQuaternionUpdate(a[0]/9.81f, a[1]/9.81f, a[2]/9.81f, g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f,  m[1], m[0], m[2]);
  MadgwickQuaternionUpdate(a, g, m);
//  MahonyQuaternionUpdate(a[0], a[1], a[2], g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f, my, mx, mz);
//  MahonyQuaternionUpdate(a, g, m);
}


  
  adjustAccelData(a, q);
  alt_estimator.propagate(a[2], deltat);

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  yaw   -= 1.2; // Declination at Burbank, California is 12.2 degrees
////      yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll  *= 180.0f / PI;

  // Set gloabl variables
  euler_angles[0] = yaw;
  euler_angles[1] = roll;
  euler_angles[2] = pitch;

//  if ((roll<5)&&(roll>-5)&&(pitch<5)&&(pitch>-5)){
//    digitalWrite(LED_BUILTIN, HIGH);
//  } else {
//    digitalWrite(LED_BUILTIN, LOW);
//  }

  // float v_avg = 0;
  float cutoff = 0.1;

//if (Now>5200000){
  for (int j = 0; j<3; j++){
      //v_avg = 1.0/2.0*(v[j] + (v[j] + a[j]*deltat));
      if (abs(a[j])>cutoff){
         v[j] = v[j] + a[j]*deltat;
      }
      if (abs(v[j])>cutoff){
        x[j] = x[j] + v[j]*deltat;
      //x[j] += v_avg*deltat;
      }
  }

//  Serial.print(Now); Serial.print("\t");
//  serialPrintArray(a);
//  serialPrintArray(v);
//  serialPrintArrayLn(x);
//  serialPrintArray(q);
//  serialPrintArray(euler_angles);

}


void MadgwickQuaternionUpdate(float *a, float *g, float *m)
{
    float ax = a[0];
    float ay = a[1];
    float az = a[2];

    float gx = g[0]*3.14159f/180.0f; // Pass gyro rate as rad/s
    float gy = g[1]*3.14159f/180.0f;
    float gz = g[2]*3.14159f/180.0f;

    float mx = m[1]; //switch mx and my
    float my = m[0]; //switch mx and my
    float mz = m[2];

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}



 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.

//void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
void MahonyQuaternionUpdate(float *a, float *g, float *m)
{

//    MadgwickQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f,
//-gz*PI/180.0f,  mx,  my, mz);
//  if(passThru)MahonyQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f,
//gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz);

//    float ax = -a[1];
//    float ay = -a[0];
//    float az = a[2];
//
//    float gx = g[1]*PI/180.0f; // Pass gyro rate as rad/s
//    float gy = g[0]*PI/180.0f;
//    float gz = -g[2]*PI/180.0f;
//
//    float mx = m[0];
//    float my = m[1];
//    float mz = m[2];

    float ax = a[0];
    float ay = a[1];
    float az = a[2];

    float gx = g[0]*3.14159f/180.0f; // Pass gyro rate as rad/s
    float gy = g[1]*3.14159f/180.0f;
    float gz = g[2]*3.14159f/180.0f;

    float mx = m[1];
    float my = m[0];
    float mz = -m[2];

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;      // missing deltat? I guess thats fine as long as its taken into account in K_i
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

// ################################################################################
void read_mpu9250()
{
//  // If intPin goes high, all data registers have new data
//  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
//    readAccelData(a);  // Read the x/y/z adc values
//    readGyroData(g); // Read the x/y/z adc values
//  }

    readAccelData(a);  // Read the x/y/z adc values
    readGyroData(g); // Read the x/y/z adc values

  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readMagData(m);  // Read the x/y/z adc values
  }
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
}


// #######################################################################################################
void setup_mpu9250(int accel_range, int gyro_range, int mag_bits, int gyro_dlfp, int accel_dlpf)
{
  bool verbose = SerialDebug;
  char dummy_str[50];



  // Set the global variables for accel and gyro ranges
  uint8_t gyro_range_cmd;
  switch (gyro_range)  {
    case 2000:   gyro_range_cmd = 0x03;   gyro_res = 2000.0/32768.0;   break; // gyro_range_cmd = 0x18;
    case 1000:   gyro_range_cmd = 0x02;   gyro_res = 1000.0/32768.0;   break; // gyro_range_cmd = 0x10;
    case 500:    gyro_range_cmd = 0x01;   gyro_res = 500.0/32768.0;   break; // gyro_range_cmd = 0x08;
    case 250:    gyro_range_cmd = 0x00;   gyro_res = 250.0/32768.0;   break;
    default:
      Serial.print("[ERROR]: Bad Gyro range command: ");
      Serial.println(gyro_range);
  }
  if (verbose) {
    Serial.print("[Setup] Setting Gryo command:");
    sprintf(dummy_str, " %x (hex) =  %d (dec) \n", gyro_range_cmd, gyro_range_cmd);
    Serial.print(dummy_str);
  }

  // Set the accelrometer range
  uint8_t accel_range_cmd;
  switch (accel_range)  {
    case 16:   accel_range_cmd = 0x03;   accel_res = (16.0)/32768.0;   break; // accel_range = 0x18;
    case 8:    accel_range_cmd = 0x02;   accel_res = (8.0)/32768.0;   break; // accel_range = 0x10;
    case 4:    accel_range_cmd = 0x01;   accel_res = (4.0)/32768.0;   break; // accel_range = 0x08;
    case 2:    accel_range_cmd = 0x00;   accel_res = (2.0)/32768.0;   break;
    default:
      Serial.print("[ERROR]: Bad Accelerometer range command");
      Serial.println(accel_range_cmd);
  }
  if (verbose) {
    Serial.print("[Setup] Setting Accelerometer command:");
    sprintf(dummy_str, " %x (hex) =  %d (dec) \n", accel_range_cmd, accel_range_cmd);
    Serial.print(dummy_str);
  }

   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
  uint8_t mag_resolution_cmd;
  switch (mag_bits)  {
    case 14:    mag_res = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
                mag_resolution_cmd = 0x00;      break;
    case 16:    mag_res = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
                mag_resolution_cmd = 0x01;      break;
    default:
      Serial.println("[ERROR] Bad Mag Resolution command");
      Serial.println(mag_bits);
  }

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if (SerialDebug){
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  }

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    if (SerialDebug){
      Serial.println("MPU9250 is online...");

      MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    }

    calibrateMPU9250(gyroBias, accelBias);// Calibrate gyro and accelerometers, load biases in bias registers
//    Serial.print("GYRO BIASES: "); Serial.print(gyroBias[0]); Serial.print("\t"); Serial.print(gyroBias[1]); Serial.print("\t"); Serial.print(gyroBias[2]); Serial.print("\n");

    initMPU9250(gyro_range_cmd, accel_range_cmd, gyro_dlpf, accel_dlpf);

    if (SerialDebug){
      Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    if (SerialDebug){
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    }
    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration, mag_resolution_cmd);
    if (SerialDebug){
      Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    }

  if(SerialDebug) {
    //  Serial.println("Calibration values: ");
    Serial.println("Bias: ");
    serialPrintArray3(magbias);
    
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
//    while(1) ; // Loop forever if communication doesn't happen
  }

  //Set the time for the state estimator
  lastUpdate = micros();
}


// ###############################################################################################
void initMPU9250(int gyro_range_cmd, int accel_range_cmd, int gyro_dlpf, int accel_dlpf)
{
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // ----------------------------------------------------------------------------------------------
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
//  With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_41HZ);//92 41 184


  
  


 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Use a 200 Hz rate; a rate higher than the filter update rate

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
//  c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | gyro_range_cmd << 3; // Set gyro range
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | accel_range_cmd << 3; // Setthe range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
//  c = c & ~0x00 << 3; // enable the accel digital low-pass filter
  c = c | A_DLPF_CFG_41Hz; // Set accelerometer rate to 1 kHz and bandwidth to (X) Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value


  c = readByte(MPU9250_ADDRESS, I2C_MST_CTRL);
  c = c & 0xF0; // clear the I2C_MST_CLK bits (master clock speed)
  c = c & 0x09; // Set master clock speed to 500kHz (probably not necessary)
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, c);

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0b00110010 );//0x22
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}

// ###############################################################################################
// Initialize the magnetometer
void initAK8963(float * destination, uint8_t mag_resolution_cmd)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  
//  Serial.print("Mag mult: ");
//  serialPrintArray3(destination);
  
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set mag_resolution_cmd bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, mag_resolution_cmd << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

// ###############################################################################################
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_tmp[3] = {0, 0, 0}, gyro_tmp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_tmp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_tmp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_tmp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_tmp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_tmp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_tmp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_tmp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_tmp[1];
    accel_bias[2] += (int32_t) accel_tmp[2];
    gyro_bias[0]  += (int32_t) gyro_tmp[0];
    gyro_bias[1]  += (int32_t) gyro_tmp[1];
    gyro_bias[2]  += (int32_t) gyro_tmp[2];

  }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L)
    {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else
    {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//    Serial.print("\n Factory Accel Bias = "); Serial.print(accel_bias_reg[ii],DEC);
//    Serial.print("\n NEW     Accel Bias = "); Serial.print(accel_bias[ii],DEC);
  }//Serial.print("\n ");

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250 (TODO)
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
 // writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
 // writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
 // writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
 // writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
 // writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
 // writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; // accelsensitivity is in LSB/g, so the bias is in g's
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// ###############################################################################################
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

int promedio=200;
  for( int ii = 0; ii < promedio; ii++) {  // get average self-test values of gyro and acclerometer

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= promedio;
  gSTAvg[ii] /= promedio;
  }

 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
   delay(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }

}





// ###############################################################################################
void readAccelData(float *accel_vec) //
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  int16_t tmp[3];
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  tmp[0] = (int16_t)((int16_t)(rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  tmp[1] = (int16_t)((int16_t)(rawData[2] << 8) | rawData[3]) ;
  tmp[2] = (int16_t)((int16_t)(rawData[4] << 8) | rawData[5]) ;

  // Now we'll calculate the accleration value into actual m^2/s
  accel_vec[0] = (((float)tmp[0]) * accel_res - accelBias[0]) * 9.81f;
  accel_vec[1] = (((float)tmp[1]) * accel_res - accelBias[1]) * 9.81f;
  accel_vec[2] = (((float)tmp[2]) * accel_res - accelBias[2]) * 9.81f;
  //Serial.println(accel_vec[1]);
}

// ###############################################################################################
void adjustAccelData(float *accel_vec, float  *quat)
{
  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = quat[0] * quat[0];
  float q1q2 = quat[0] * quat[1];
  float q1q3 = quat[0] * quat[2];
  // float q1q4 = quat[0] * quat[3];
  float q2q2 = quat[1] * quat[1];
  // float q2q3 = quat[1] * quat[2];
  float q2q4 = quat[1] * quat[3];
  float q3q3 = quat[2] * quat[2];
  float q3q4 = quat[2] * quat[3];
  float q4q4 = quat[3] * quat[3];

  // Flipping q0 and q4
//  float q1q1 = q[1] * q[1];
//  float q1q2 = q[1] * q[2];
//  float q1q3 = q[1] * q[3];
//  float q1q4 = q[1] * q[0];
//  float q2q2 = q[2] * q[2];
//  float q2q3 = q[2] * q[3];
//  float q2q4 = q[2] * q[0];
//  float q3q3 = q[3] * q[3];
//  float q3q4 = q[3] * q[0];
//  float q4q4 = q[0] * q[0];

//    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//    pitch *= 180.0f / PI;
//    yaw   *= 180.0f / PI;
//    yaw   -= 12.2; // Declination at Burbank, California is 12.2 degrees
//    roll  *= 180.0f / PI;
//      Serial.print(yaw, 2);  Serial.print("\t ");
//      Serial.print(pitch, 2);  Serial.print("\t ");
//      Serial.print(roll, 2);  Serial.println("\t ");

//  float Q_ab[3][3] = {{q1q1-q2q2-q3q3+q4q4, 2*(q1q2+q3q4), 2*(q1q3-q2q4)},
//                      {2*(q1q2-q3q4), -q1q1+q2q2-q3q3+q4q4, 2*(q2q3+q1q4)},
//                      {2*(q1q3+q2q4), 2*(q2q3-q1q4), -q1q1-q2q2+q3q3+q4q4}};

//  float Q_ab[3] = {q1q1-q2q2-q3q3+q4q4, 2*(q1q2-q3q4), 2*(q1q3+q2q4)};
  float Q_ab[3] = {q1q1-q2q2-q3q3+q4q4, 2*(q1q2+q3q4), 2*(q1q3-q2q4)};
//  float q_star[4] = {q[0], -q[1], -q[2], -q[3]};

//  Serial.println(sqrt((2*(q1q3-q2q4))*(2*(q1q3-q2q4)) + (2*(q2q3+q1q4))*(2*(q2q3+q1q4)) + (-q1q1-q2q2+q3q3+q4q4)*(-q1q1-q2q2+q3q3+q4q4)));

  float new_g[3];
  for (int i=0;i<3;i++){
    new_g[2-i] = Q_ab[i]*-9.81; //matrix multiplication, simplified for a single-valued g-vector of constant value
  }
  a[0] = accel_vec[0] - new_g[0];
  a[1] = accel_vec[1] + new_g[1];
  a[2] = accel_vec[2] + new_g[2];

//  serialPrintArray(accel_vec);
//  serialPrintArray(new_g);
//  serialPrintArrayLn(new_accel_vec);

}
// ###############################################################################################
void readGyroData(float *gyro_vec)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  int16_t tmp[3];

  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  tmp[0] = (int16_t)((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  tmp[1] = (int16_t)((int16_t)rawData[2] << 8) | rawData[3] ;
  tmp[2] = (int16_t)((int16_t)rawData[4] << 8) | rawData[5] ;

  // Calculate the gyro value into actual degrees per second
  gyro_vec[0] = (float)tmp[0]*gyro_res;  // get actual gyro value, this depends on scale being set
  gyro_vec[1] = (float)tmp[1]*gyro_res;
  gyro_vec[2] = (float)tmp[2]*gyro_res;
}















// ###############################################################################################
void magcalMPU9250(float * dest1, float * dest2) 
 {
 uint16_t ii = 0, sample_count = 0;
 int32_t mag_bia[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
 int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
 
 magbias[0]= 0;
 magbias[1]= 0;
 magbias[2]= 0;

 Serial.println("Mag Calibration: Wave device in a figure eight until done!");
   Serial.println("Bias: ");
 serialPrintArray3(dest1);
 delay(4000);

// shoot for ~fifteen seconds of mag data

sample_count = 800;  // at 100 Hz ODR, new mag data is available every 10 ms

for(ii = 0; ii < sample_count; ii++) {
readMagData(m);  // Read the mag data   
for (int jj = 0; jj < 3; jj++) {
  if(m[jj] > mag_max[jj]) mag_max[jj] = m[jj];
  if(m[jj] < mag_min[jj]) mag_min[jj] = m[jj];
}
delay(50);  // at 8 Hz ODR, new mag data is available every 125 ms
Serial.println(ii);

}


// Get hard iron correction
 mag_bia[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 mag_bia[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 mag_bia[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

 dest1[0] = (float) mag_bia[0]*mag_res*magCalibration[0];  // save mag biases in G for main program
 dest1[1] = (float) mag_bia[1]*mag_res*magCalibration[1];   
 dest1[2] = (float) mag_bia[2]*mag_res*magCalibration[2];  
   
// Get soft iron correction estimate
 mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
 mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
 mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

 float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 avg_rad /= 3.0;

 dest2[0] = avg_rad/((float)mag_scale[0]);
 dest2[1] = avg_rad/((float)mag_scale[1]);
 dest2[2] = avg_rad/((float)mag_scale[2]);

  Serial.println("Bias: ");
 serialPrintArray3(dest1);
 Serial.println("Mult");
 serialPrintArray3(dest2);
 

 Serial.println("Mag Calibration done!");
 }





 
// ###############################################################################################
void readMagData(float *mag_vec) //int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  int16_t tmp[3];

  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    tmp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    tmp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    tmp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
  }

//  magbias[0] = 144.96f;//+249.2; // +609.360; //+666.715; //+470.0;  // User environmental x-axis correction in milliGauss, should be automatically calculated
 // magbias[1] = 218.48f;//+83.7; // +447.380; //+452.795; //+120.0;  // User environmental y-axis correction in milliGauss
  //magbias[2] = -337.47f;//-489.63; //-400.095; //-412.265; //+125.0;  // User environmental z-axis correction in milliGauss

//  237.5 +N // from http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
//  51.672 + E
// 404.095 + Up vertical

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  m[0] = (float)tmp[0]*mag_res*magCalibration[0]*magbias2[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
  m[1] = (float)tmp[1]*mag_res*magCalibration[1]*magbias2[1] - magbias[1];
  m[2] = (float)tmp[2]*mag_res*magCalibration[2]*magbias2[2] - magbias[2];
}

// ###############################################################################################
int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}




void serialPrintArray(float *vec){
  Serial.print(vec[0]); Serial.print("\t");
  Serial.print(vec[1]); Serial.print("\t");
  Serial.print(vec[2]); Serial.print("\t");
}


void serialPrintArray4(float *vec){
  Serial.print(vec[0]); Serial.print("\t");
  Serial.print(vec[1]); Serial.print("\t");
  Serial.print(vec[2]); Serial.print("\t");
  Serial.print(vec[3]); Serial.println("\t");
}


void serialPrintArray3(float *vec){
  Serial.print(vec[0]); Serial.print("\t");
  Serial.print(vec[1]); Serial.print("\t");
  Serial.println(vec[2]);
}


void serialPrintArrayLn(float *vec){
  serialPrintArray(vec); Serial.print("\n");
}



// ###############################################################################################
// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

// ###############################################################################################
// Wire.h read and write protocols
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// ###############################################################################################
// Wire.h read and write protocols
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
