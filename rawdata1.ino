#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* This driver reads raw data from the BNO055
  `
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */

#define BNO055_SAMPLERATE_DELAY_MS (2)

Adafruit_BNO055 bno = Adafruit_BNO055();

int calibration = 0;
int start = 0; //check whether device calibrated or not and if yes, start is set to 1
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

typedef struct
{
  float x, y, z;
} AbsAccel;

AbsAccel absAccel;

typedef struct
{
  float x, y, z;
  float q0, q1, q2, q3;
} Position;

Position position;



void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1); //infinite loop
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  //  Serial.println(" Celcius");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  uint8_t system, gyro, accel, mag = 0;
  while (system != 3)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
    delay(100);  //after every reading, a 100ms delay given
  }
  Serial.println(""); Serial.println("Calibrated");
  start = 1;
  delay(5000); //delay after device is calibrated and before void loop starts

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void readLinAcc()
{
  imu::Vector<3> linaccl = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* Display the floating point data
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");*/
}

void quatval()
{
  imu::Quaternion quat = bno.getQuat();
  /*Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");*/
}
void readAbsAcc()
{
  //multiplication of linear acceleration and quaternion
  float tempQuat[4];

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat = bno.getQuat();

  //http://es.mathworks.com/help/aeroblks/quaternionmultiplication.html   q=quat, r=linAcc
  tempQuat[0] = 0 - euler.x() * quat.x() - euler.y() * quat.y() - euler.z() * quat.z();
  //Serial.print("tempquat[0]->");
  //Serial.println(tempQuat[0]);
  tempQuat[1] = 0 + euler.x() * quat.w() - euler.y() * quat.z() + euler.z() * quat.y();
  tempQuat[2] = 0 + euler.x() * quat.z() + euler.y() * quat.w() - euler.z() * quat.x();
  tempQuat[3] = 0 - euler.x() * quat.y() + euler.y() * quat.x() + euler.z() * quat.w();

  //q=tempQuat, r=quatConj
  absAccel.x = quat.w() * tempQuat[1] - quat.x() * tempQuat[0] + quat.y() * tempQuat[3] - quat.z() * tempQuat[2];
  absAccel.y = quat.w() * tempQuat[2] + quat.x() * tempQuat[3] - quat.y() * tempQuat[0] + quat.z() * tempQuat[1];
  absAccel.z = quat.w() * tempQuat[3] + quat.x() * tempQuat[2] - quat.y() * tempQuat[1] - quat.z() * tempQuat[0];
  //Serial.print("absaccel aX aY aZ : ");Serial.print(absAccel.x);Serial.print( absAccel.y);Serial.print( absAccel.z);
}

void led_1000blink()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}

void led_250blink()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(250);
}
void deadReckoning(int mode)
{
  float m = 0;; //magnitude of aX aY aZ
  float aX = 0, aY = 0, aZ = 0;
  static float aXOld = 0, aYOld = 0, aZOld = 0;
  float vX = 0, vY = 0, vZ = 0;
  static float vXOld = 0, vYOld = 0, vZOld = 0;
  float pX = 0, pY = 0, pZ = 0;
  static float pXOld = 0, pYOld = 0, pZOld = 0;
  float accelWindow = 0.1000;
  int sampleCount = 10;
  static int noAccCount = 0;
  int noMovement = 3;
  unsigned long timeNowDeadReckoning = millis();
  static unsigned long timeOldDeadReckoning = 0;
  unsigned long interval = timeNowDeadReckoning - timeOldDeadReckoning;

  imu::Vector<3> linaccl = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  //
  //
  //for (int i = 0; i < sampleCount; i++)
  ///{ //Take average acceleration to reduce error
    //        if (mode==0)
    //        {
    //            readAbsAcc();
    //            aX+=absAccel.x;
    //            aY+=absAccel.y;
    //            aZ+=absAccel.z;
    //         //  Serial.print("aX aY aZ : "); Serial.print(aX);Serial.print(aY); Serial.println(aZ);
    //        }
//    if (mode == 1)
//    {
//      readLinAcc();
//      aX += linaccl.x();
//      aY += linaccl.y();
//      aZ += linaccl.z();
//      //   Serial.print("aX aY aZ : "); Serial.print(aX);Serial.print(aY); Serial.println(aZ);
//    }

    //    }
    //
    //   //Serial.print("aX aY aZ : "); Serial.print(aX);Serial.print(aY); Serial.println(aZ);
//    aX /= sampleCount;
//    aY /= sampleCount;
//    aZ /= sampleCount;

    aX=linaccl.x();
    aY=linaccl.y();
    aZ=linaccl.z();
    //readAbsAcc();
    //aX=absAccel.x;
    //aY=absAccel.y;
    //aZ=absAccel.z;
    //Serial.print("aX aY: "); Serial.print(aX,4);Serial.print(" ");Serial.print(aY,4); //Serial.print(" ");Serial.println( aZ);

    if ( ( aX > -accelWindow ) && ( aX < accelWindow) ) aX = 0; //Give window to reduce noise
    //else aX*=0.981;        //  cm/s2                                      //Convert to m/s2-*0.00981
    if ( ( aY > -accelWindow ) && ( aY < accelWindow) ) aY = 0;
    //else aY*=0.981;
    if ( ( aZ > -accelWindow ) && ( aZ < accelWindow) ) aZ = 0;
    //else aZ*=0.981;

    //comp_filter();
//    Serial.print("-2.,"); //set lower scale
//    Serial.print(aX, 4);
//    Serial.print(",");
//    Serial.print(aY, 4);
//    Serial.println(",2.0");
    Serial.print(" aX:"); Serial.print(aX,4);
    Serial.print(" ");
    Serial.print("aY:");Serial.print(aY,4);Serial.print(" ");//Serial.print("aZ:"); Serial.print(aZ,4);Serial.print("\t");
    //m=sqrt(aX*aX+aY*aY);Serial.print(m,4);Serial.print(" ");

    if (aX == 0.0000 && aY == 0.0000) //&& aZ==0.00)
    { noAccCount++;   //If there is no accel in any axis set speed to 0
      vXOld = 0;
      vYOld = 0;
      vZOld = 0;
      //Serial.println("test1..1stloop");
      // noAccCount=0; //Stops the counter from overflowing
      //        pXOld=0;
      //        pYOld=0;
      //        pZOld=0;
      //

    }
    else
    {
      // Serial.println("test2..2ndloop");
      noAccCount = 0;
    }

    //Serial.print("Acc count : "); Serial.println(noAccCount);

    if (noAccCount > noMovement)
    {
      vXOld = 0;
      vYOld = 0;
      vZOld = 0;
      noAccCount = 0; //Stops the counter from overflowing
      //  Serial.print("oldtime : "); Serial.print(timeOldDeadReckoning);
      //  Serial.print("nowtime : "); Serial.print(timeNowDeadReckoning);
      //  Serial.println("interval : "); Serial.print(interval);
      //        pXOld=0;
      //        pYOld=0;
      //        pZOld=0;
      timeOldDeadReckoning = timeNowDeadReckoning;

      Serial.print("vX : "); Serial.print(vX,4);Serial.print(" ");Serial.print("vY : ");Serial.print(vY,4);Serial.print(" ");
      //Serial.print("interval : "); Serial.print(interval);
      //Serial.print("vX vY vZ : "); Serial.print(vX);Serial.print(" ");Serial.print(vY); Serial.print(" ");Serial.print(vZ);Serial.print("\t");

      // Serial.print("test3..3rdloop");
    }
    //if(!(aX==0.00 && aY==0.00 && aZ==0.00))
    else
      //{noAccCount=0;}
      //if(noAccCount==0)
    {
      //if(m!=1.0000)
      //{
      //aX=abs(aX);
      //Serial.print("abs aX");Serial.print(aX);

      //if(aXOld<aX)
      //{
      // Serial.println("test4..4thloop");
      vX = vXOld + ( aXOld + ( aX - aXOld ) / 2.0 ) * interval; //Area of rectangle:Sample(n-1) * t ,  Area of triangle:(Sample(n) - Sample(n-1)) * 0.5 * t
      vY = vYOld + ( aYOld + ( aY - aYOld ) / 2.0 ) * interval;
      vZ = vZOld + ( aZOld + ( aZ - aZOld ) / 2.0 ) * interval;
      //vX = vXOld + (( aXOld + aX) / 2.0 )*interval;
      vX = vX / 1000;
      vY = vY / 1000;

      //  Serial.print("interval : "); Serial.println(interval);
      // vX = vXOld+aX*interval;
      //  vY = vYOld + aY*interval;
      //  vZ = vZOld + aZ*interval;
//      Serial.print("-2.,"); //set lower scale
//      Serial.print(vX, 4);
//      Serial.print(",");
//      Serial.print(vY, 4);
//      Serial.println(",2.0");
       Serial.print("vX : "); Serial.print(vX,4);Serial.print(" ");Serial.print("vY : ");Serial.print(vY,4);Serial.print(" ");
      pX = pXOld + ( vXOld + ( vX - vXOld ) / 2.0 ) * interval;
      pY = pYOld + ( vYOld + ( vY - vYOld ) / 2.0 ) * interval;
      pZ = pZOld + ( vZOld + ( vZ - vZOld ) / 2.0 ) * interval;
      // pX=pX/1000;
      //pX=ut+1/2at2

      //  pX=vXOld*interval+0.5*aX*interval*interval;

      //Serial.print("interval : "); Serial.print(interval,4);
      //pX = vX*interval;
      //pY = vY*interval;
      //pZ = vZ*interval;
      //  }
      aXOld = aX;
      aYOld = aY;
      aZOld = aZ;

      vXOld = vX;
      vYOld = vY;
      vZOld = vZ;
      //Serial.print("oldtime : "); Serial.print(timeOldDeadReckoning);

      pXOld = pX;
      pYOld = pY;
      pZOld = pZ;

      timeOldDeadReckoning = timeNowDeadReckoning;


      //Serial.print("nowtime : "); Serial.print(timeNowDeadReckoning);
      position.x = pX;
      position.y = pY;
      position.z = pZ;

      //Serial.print("Acc count : "); Serial.print(noAccCount);
      //    if(mode==1) bno.getQuat();
      //    position.q0=quat.w();
      //    position.q1=quat.x();
      //    position.q2=quat.y();
      //    position.q3=quat.z();
      ////Serial.print(" oldtime : "); Serial.print(timeOldDeadReckoning);
      ////Serial.print("  nowtime : "); Serial.println(timeNowDeadReckoning);
      //Serial.print("  interval: "); Serial.println(interval);
    }
  }
  //comp_filter();
  //    if(position.x>=11.00)
  //      led_1000blink() ;
  //   // if((position.x>=9.00 && position.x<=11.00)&& (position.y>=9.00 && position.y<=11.00))
  //     // ledblink(500) ;
  //    else if(position.y>=11.00)
  //      led_250blink() ;
  //

//}



void loop(void)
{
  static int noAccCount = 0;
  unsigned long timeNowDeadReckoning = millis();
  static unsigned long timeOldDeadReckoning = 0;




  /* Display calibration status for each sensor. */

  ///readAbsAcc();

  //Dead Reckoning
  // deadReckoning(0);    //Uses world coordinates, x component of acceleration will be in the user's x axis, independently of orientation
  deadReckoning(1);   //Uses local coordinates, x component of acceleration will be in the sensor's x axis
  // comp_filter();
  // Serial.print("Acc count : "); Serial.print(noAccCount);
  Serial.print(" Location X : "); Serial.print(position.x, 4); Serial.print(" Location Y: "); Serial.print(position.y, 4); // Serial.print(" Location Z: "); Serial.println(position.z);//Serial.print("\t");
//  Serial.print("-2.,"); //set lower scale
//  Serial.print(position.x, 4);
//  Serial.print(",");
//  Serial.print(position.y, 4);
//  Serial.println(",2.0");
  Serial.println(" ");

  //Serial.print("Orientation Q0: "); Serial.print(position.q0); Serial.print(" Orientation  Q1: "); Serial.print(position.q1); Serial.print(" Orientation  Q2: "); Serial.print(position.q2); Serial.print(" Orientation  Q3: "); Serial.println(position.q3);
  //Serial.println(" ");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
