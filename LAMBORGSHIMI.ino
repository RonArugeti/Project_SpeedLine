                   /* Shimi Speed Line Follower Robot V10   
      __               
     / /   ____  _______ __     ____  ____ ______ _____ __  __ __ _____  __
    / /   / __ `/ _  _  / /____/ __ `/ ___/ __  //   _/  /_/ /  / _  _ /  /
   / /___/ /_/ / // // / /__/ / /_/ / /  / /_/ /__\ \/  __  /  / // / /  /
  /_____/\__,_/_//_//_/\_____/\____/_/   \__  /\____/_/  /_/\_/_//_/_/\_/   
                                         __/ /
                                        /___/
*/

#include <Arduino_APDS9960.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position = 0;

boolean bol = LOW;

//Mapeo de pines
const int MotorPinA = 4; // direction motor 1 (Channel A)
const int MotorSpeedPinA = 5; // speed for motor 1 (channel A)

const int MotorPinB = 7; // direction motor 2 (Channel B)
const int MotorSpeedPinB = 6;// speed for motor 2 (channel B)

const int CW  = HIGH;
const int CCW = LOW;
int LED = 3;

// Constantes para PID
float KP = 0.7;
float KD = 4;
float Ki = 0.0;

// Regulación de la velocidad Máxima
int Velmax = 198;

// Data para intrgal 
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;
int error7 = 0;
int error8 = 0;

int r = 0;
int g = 0;
int b = 0;
int proximity = 0;
int gesture = 0;

//declaraos variables para utilizar PID
int proporcional = 0;          // Proporcional
int integral = 0;             //Intrgral
int derivativo = 0;          // Derivativo
int irright = 0;
int irleft = 0;
const int leftPin_ir = 13;
const int rightPin_ir = 12;

int diferencial = 0;   // Diferencia aplicada a los motores
int last_prop = 0;     // Última valor del proporcional (utilizado para calcular la derivada del error)
const int Target = 3500;     // Setpoint (Como utilizamos 8 sensores, la línea debe estar entre 0 y 7000, por lo que el ideal es que esté en 3500)

void Proximity();
void Coloreading();
void Gesture();
void freno(boolean left, boolean right, int value);
void Motor(int left, int right);
void Motorde(int value);
void Motoriz(int value);

void setup()
{
  // print the calibration minimum values measured when emitters were on
 //Serial.begin(9600);
 APDS.begin();
 Serial;
/*if (!APDS.begin())
    Serial.println("Error initializing APDS-9960 sensor!");
    else 
    Serial.println("Initializing APDS-9960 sensor complete!");
  */
  // motor A pin assignment
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorSpeedPinA, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinB, OUTPUT);
  pinMode(MotorSpeedPinB, OUTPUT);

  pinMode(leftPin_ir,INPUT);
  pinMode(rightPin_ir,INPUT);
  pinMode(LED,OUTPUT);
  
  //qtr4.0
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){8, 9, 10, 11, A0, A1, A2, A3}, SensorCount);

// Calibracion 4.0
for (uint16_t i = 0; i < 100; i++)
  { 
    digitalWrite(LED, HIGH); delay(20);
    qtr.calibrate();
    digitalWrite(LED, LOW);  delay(20);
  }
    digitalWrite(LED, LOW); // turn off Arduino's LED to indicate we are through with calibration
 delay(3000);
 while(!bol)
 Gesture();
}

void loop()
{ 
  irright = digitalRead(rightPin_ir);
  irleft = digitalRead(leftPin_ir);
  
  //qtr4.0  
  position = qtr.readLineBlack(sensorValues);
  proporcional = (int)position - Target;
  
/*for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(proporcional);*/

  derivativo = proporcional - last_prop; 
  integral = error1+error2+error3+error4+error5+error6+error7+error8;
  last_prop = proporcional;

  error8=error7;
  error7=error6;
  error6=error5;
  error5=error4;  
  error4=error3;  
  error3=error2;
  error2=error1;
  error1=proporcional;
  
  diferencial = ( proporcional * KP ) + ( derivativo * KD )+ (integral*Ki) ;
 Proximity();
 
 if(proximity < 200){
    digitalWrite(MotorPinA, CCW);
    digitalWrite(MotorPinB, CCW); 
    analogWrite(MotorSpeedPinA,50);
    analogWrite(MotorSpeedPinB,50);
    delay(40);
    digitalWrite(MotorPinA, CW);
    digitalWrite(MotorPinB, CW);
 }
 else{
       digitalWrite(LED,LOW);
  if ( proporcional <= -Target )
  {
    Motorde(0);
    freno(true,false,255);
  }
  else if ( proporcional >= Target )
  {
    Motoriz(0);
    freno(false,true,255);
  }
  
 if (irleft == 0 && irright == 1 ){     //right 90
     digitalWrite(MotorPinA, CCW);
     digitalWrite(MotorPinB, CW);
     analogWrite(MotorSpeedPinA,Velmax);
     analogWrite(MotorSpeedPinB,Velmax);
     delay(100);
     digitalWrite(MotorPinA, CW);
  }

  if (irleft  == 1 && irright == 0){   //left 90
    digitalWrite(MotorPinA, CW);
    digitalWrite(MotorPinB, CCW);
    analogWrite(MotorSpeedPinA,Velmax);
    analogWrite(MotorSpeedPinB,Velmax);
    delay(100);  
    digitalWrite(MotorPinB, CW);
  }
  if (irleft == 1 && irright == 1 ){  
    /*digitalWrite(MotorPinA, CW);
    digitalWrite(MotorPinB, CW);
    analogWrite(MotorSpeedPinA,Velmax-50);
    analogWrite(MotorSpeedPinB,Velmax-50);*/ 
    delay(100); 
  }
  if ( diferencial > Velmax ) diferencial = Velmax; 
  else if ( diferencial < -Velmax ) diferencial = -Velmax;
  ( diferencial < 0 ) ? Motor(Velmax+diferencial, Velmax) : Motor(Velmax, Velmax-diferencial);
 }
}

void Proximity()
{
  // check if a proximity reading is available
  if (APDS.proximityAvailable()) {
   
    // read the proximity
    // - 0   => close
    // - 255 => far
    // - -1  => error
    proximity = APDS.readProximity();
    if(proximity < 200){  
    Coloreading();   
    }
 
    }
    // print value to the Serial Monitor
    //Serial.println(proximity);
  }

  void Coloreading()
{
  // check if a color reading is available
  while (! APDS.colorAvailable()) {
    delay(5);
  }
 analogWrite (MotorSpeedPinA, 0);
 analogWrite (MotorSpeedPinB, 0);
  // read the color
  APDS.readColor(r, g, b);
if(r <=45 && r>=35 && g <= 75 &&g>=65 && b >=75 && b<=85) //azul
{
  digitalWrite(LED,HIGH);
  //Serial.print("b = ");
  //Serial.println(b);
  delay(3000);
  }
if(r <=55 && r>=35 && g <= 85 &&g>=60 && b >=40 && b<=70) //verde
{
  for(int i = 0; i < 4; i++){
digitalWrite(LED,HIGH);
delay(500);
digitalWrite(LED,LOW);
delay(500);
//Serial.print("g = ");
//Serial.println(g);
  }
}
if(r <=80 && r>= 65 && g <=30 && g>=25 && b >= 35 && b<=46) //rojo 
{
  for(int i = 0; i < 50; i++){
digitalWrite(LED,CW);
delay(30);
digitalWrite(LED,CCW);
delay(30);
//Serial.print("r = ");
//Serial.println(r);
  }
}
  /* // print the values
  Serial.print("r = ");
  Serial.println(r);
  Serial.print("g = ");
  Serial.println(g);
  Serial.print("b = ");
  Serial.println(b);
  Serial.println();
  Serial.println(proximity);
  delay(1200);*/
  }
  
void Gesture()
{
  //Serial.println("Detecting gestures ...");

   if (APDS.gestureAvailable()) {
      //Serial.println("Detecting iiiit");
    // a gesture was detected, read and print to Serial Monitor
    gesture = APDS.readGesture();

    switch (gesture) {
      case GESTURE_UP:
        //Serial.println("Detected UP gesture");
        bol = HIGH;
        break;
        
        
      case GESTURE_DOWN:
      // Serial.println("Detected DOWN gesture");
      if(bol){
        APDS.end();
      // Serial.println("Ending loop");
        exit(0);
      }
        break;

      case GESTURE_LEFT:
      // Serial.println("Detected LEFT gesture");
     digitalWrite(MotorPinA, CW);
     digitalWrite(MotorPinB, CCW);
     analogWrite(MotorSpeedPinA,Velmax);
     analogWrite(MotorSpeedPinB,Velmax);
     delay(1000);
     digitalWrite(MotorPinA, CW);
        break;

      case GESTURE_RIGHT:
      // Serial.println("Detected RIGHT gesture");
     digitalWrite(MotorPinA, CCW);
     digitalWrite(MotorPinB, CW);
     analogWrite(MotorSpeedPinA,Velmax);
     analogWrite(MotorSpeedPinB,Velmax);
     delay(1000);
     digitalWrite(MotorPinA, CW);
        break;

      default:
        // ignore
        break;
    }
  }
}

// Función accionamiento motor izquierdo
void Motoriz(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(MotorPinB,CW);
  }
  else
  {
    digitalWrite(MotorPinB,CCW);
    value *= -1;
  }
  analogWrite(MotorSpeedPinB,value);
}

// Función accionamiento motor derecho
void Motorde(int value)
{  
  value *= (20/11);
  
  if ( value >= 0 )
  {
    digitalWrite(MotorPinA,CW);
  }
  else
  {
    digitalWrite(MotorPinA,CCW);
    value *= -1;
  }    
  analogWrite(MotorSpeedPinA,value);
}

//Accionamiento de motores
void Motor(int left, int right)
{
  Motoriz(left);
  Motorde(right);
}

//función de freno
void freno(boolean left, boolean right, int value){
  analogWrite (MotorSpeedPinA, 0);
  analogWrite (MotorSpeedPinB, 0);
  if ( right )
  {
    digitalWrite(MotorPinA,CW);
    analogWrite (MotorSpeedPinA, value);
  }
  if ( left )
  {
    digitalWrite(MotorPinB,CW);
    analogWrite (MotorSpeedPinB, value);
  }
}
