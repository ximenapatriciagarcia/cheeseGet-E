// PISTA A  -----DETECCION DE COLORES           


// Author: Ximena Garcia
//Pist A: deteccion de flags

#include <Wire.h>   //giroscopio y sensor de color (I2C)
#include <NewPing.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>   //sensor de color (flags)

#define BMI160_ADDR 0x69

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void iniciarGiroscopio() {

  Wire.begin();
  writeReg(0x7E, 0xB6);
  delay(200);
  uint8_t id = readReg(0x00);
  if (id != 0xD1) while (1);
  writeReg(0x7E, 0x15);
  delay(100);
  writeReg(0x7E, 0x11);
  delay(100);
  writeReg(0x43, 0x00);
  writeReg(0x41, 0x03);
}

void leerGiroscopio(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 6);
  if (Wire.available() == 6) {
    gx = Wire.read() | (Wire.read() << 8);
    gy = Wire.read() | (Wire.read() << 8);
    gz = Wire.read() | (Wire.read() << 8);
  } else {
    gx = gy = gz = 0; //todo es igual a cero
  }
}

const int ADELANTE_1 = 4; // LLANTA 1: (ADELANTE) 
const int ATRAS_1 = 5;    // LLANTA 1: (ATRAS)    
const int ATRAS_2 = 7;    // LLANTA 2: (ADELANTE)
const int ADELANTE_2 = 8; 

const int PWM1 = 3;
const int PWM2 = 9;

int vBase = 100;
int vGiro = 90;
int vLigero = 40;

const int SERVO_PIN = 9;
Servo servo1;
int angulo = 0;

//---------------------------------------------------------------------Funciones Ultrasonicos---------------------------------------------------------------------------------//
#define MAX_DISTANCE 200
const int trigPinFront = 10;
const int echoPinFront = 11;

const int trigPinLeft  = 12;
const int echoPinLeft  = 13;

const int trigPinRight = 14;
const int echoPinRight = 15;

NewPing sonarFront(trigPinFront, echoPinFront, MAX_DISTANCE);
NewPing sonarLeft(trigPinLeft, echoPinLeft, MAX_DISTANCE);
NewPing sonarRight(trigPinRight, echoPinRight, MAX_DISTANCE);

unsigned long kickStart = 0;
bool kicking = false;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int S0 = 43;
const int S1 = 41;
const int S2 = 44;
const int S3 = 46;
const int sensorOut = 42;

unsigned long previousMillis = 0;
const unsigned long intervalo = 15;
float yaw = 0.0;


//----------------------------Sensor de color GENERAL-------------------------------------------------------------------
String SColor() {
  uint16_t r, g, b, c;
  float red, green, blue;
  tcs.getRawData(&r, &g, &b, &c);
  if (c == 0) c = 1;
  red   = (float) r / c * 255.0;
  green = (float) g / c * 255.0;
  blue  = (float) b / c * 255.0;
  if (red > 120 && green < 100 && blue < 100) return "RED"; Serial.print("RED");
  else if (red < 100 && green > 120 && blue < 100) return "GREEN"; Serial.print("GREEN");
  else if (red < 100 && green < 100 && blue > 120) return "BLUE"; Serial.print("BLUE");
  else if (red > 150 && green > 120 && blue < 80) return "YELLOW"; Serial.print("YELLOW");
  else return "NINGUNO";
}

unsigned long leerCanal(bool s2, bool s3) {
  digitalWrite(S2, s2 ? HIGH : LOW);
  digitalWrite(S3, s3 ? HIGH : LOW);
  unsigned long t = pulseIn(sensorOut, LOW, 50000UL);
  if (t == 0) t = 50000UL;
  return t;
}





//------------------------------------------Funcion color Naranja------------------------------------------------------------------------------------------------------
String Naranja() {
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  unsigned long pR = leerCanal(false, false);
  unsigned long pG = leerCanal(true,  true);
  unsigned long pB = leerCanal(false, true);
  bool rojoFuerte = (pR < 1800);
  bool verdeMedio = (pG > 1500 && pG < 3500);
  bool azulDebil  = (pB > 3000);
  if (rojoFuerte && verdeMedio && azulDebil) return "ORANGE";
  else return "NONE";
}


//----------------------------------------------------------------------------FUNCIONES MOTORES---------------------------------------------------------
void DELANTERAS_adelante(int vB) {
  digitalWrite(ADELANTE_2, HIGH);
  digitalWrite(ADELANTE_1, HIGH);
  analogWrite(PWM1, vB);
  analogWrite(PWM2, vB);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, LOW);
}

void DELANTERAS_atras(int vB) {
  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, vB);
  analogWrite(PWM2, vB);
  digitalWrite(ATRAS_2, HIGH);
  digitalWrite(ATRAS_1, HIGH);
}

void vuelta_Izquierda() {
  digitalWrite(ADELANTE_2, HIGH);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, vGiro);
  analogWrite(PWM2, vGiro);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, HIGH);
}

void vuelta_Derecha() {
  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, HIGH);
  analogWrite(PWM1, vGiro);
  analogWrite(PWM2, vGiro);
  digitalWrite(ATRAS_2, HIGH);
  digitalWrite(ATRAS_1, LOW);
}

void ligero_IZ() {

  digitalWrite(ADELANTE_2, HIGH);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, vLigero);
  analogWrite(PWM2, vLigero);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, HIGH);

}

void ligero_DE() {

  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, HIGH);
  analogWrite(PWM1, vLigero);
  analogWrite(PWM2, vLigero);
  digitalWrite(ATRAS_2, HIGH);
  digitalWrite(ATRAS_1, LOW);

}

void stop() {

  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, LOW);

}
//----------------------------------------------------------------------------FUNCIONES MOTORES----- termina----------------------------------------------------


enum Estado {
  INICIO,

  BUSQUEDA,

  AVANZA_ENCUENTRA,

  FLAG,

  KICK,

  DROP,

  FINAL

};
Estado EActual = INICIO;

//-------------------------------Contador para las flags
const int FLAGS = 5;
  String flags[FLAGS];
int flagCount = 0;

//------------------------------registros de las flags
void registroFLAGS(String color) {
  if (flagCount < FLAGS) {
    flags[flagCount] = color;
    flagCount++;
  }
}

//Deboguear los estados
void CambioEstado(const char* e, const String& cFlag, const String& cBall, float d) {
  Serial.print("ESTADO: "); Serial.print(e);

  Serial.print("FLAG: "); Serial.print(cFlag);

  Serial.print("BALL: "); Serial.print(cBall);

  Serial.print("DF: "); Serial.println(d);
}


//----------------------------------Funcion para leer los ultrasonicos
void leerUltrasonicos(float &dFront, float &dLeft, float &dRight) {
  
  dFront = sonarFront.ping_cm(); 
        if (dFront == 0) dFront = 999; // distancia maxima
  dLeft  = sonarLeft.ping_cm();  
        if (dLeft  == 0) dLeft  = 999;
  dRight = sonarRight.ping_cm(); 
        if (dRight == 0) dRight = 999;
}

void mantenerDireccionPorGiro(int16_t gz, const String& colorFlag) {

  if (colorFlag == "BLUE" || colorFlag == "YELLOW" || colorFlag == "RED" || colorFlag == "GREEN" || colorFlag == "NINGUNO") return;

  float dps = ((float)gz) / 16.4;
  yaw += dps * (intervalo / 1000.0);
  //ajuste de las llantas dependiendo del giroscopio
  if (yaw > 5) ligero_IZ();
  else if (yaw < -5) ligero_DE();

  else DELANTERAS_adelante(vBase);
}

void evitarObstaculos(float dFront, float dLeft, float dRight, const String& colorFlag) {
  if (colorFlag == "BLUE" || colorFlag == "YELLOW" || colorFlag == "RED" || colorFlag == "GREEN" || colorFlag == "NINGUNO") return;

  if (dFront < 20) {
    stop();
    delay(5);
    if (dLeft > dRight) { //cuando la distancia izquierda es mayor a la derecha
      vuelta_Izquierda();
      delay(50);

    } else {
      vuelta_Derecha();
      delay(50);
    }
    yaw = 0; // se reacomoda el robot
    DELANTERAS_adelante();

  } else if (dLeft < 10) { //si la distancia es demasiado corta de la izquierda
    ligero_DE();
    delay(50);

  } else if (dRight < 10) { //si la distancia es demasiado corta de la derecha
    ligero_IZ();
    delay(150);
  }
}




//------MAQUINA DE ESTADOS-----------------------


void newEstado(String color, float dFront, String naranja) {

  switch (EActual) {

    case INICIO:

      if (color == "GREEN") {
        EActual = BUSQUEDA;
        CambioEstado("BUSQUEDA", color, naranja, dFront); //solamente para verlo desplegado en la pantalla
      }
      break;

    case BUSQUEDA:

      if (naranja != "ORANGE") {
        DELANTERAS_adelante(vGiro);
      } else {
        stop();
        for (angulo = 0; angulo <= 180; angulo++) servo1.write(angulo);
        EActual = AVANZA_ENCUENTRA;
        CambioEstado("AVANZA_ENCUENTRA", color, naranja, dFront);
      }
      break;

    case AVANZA_ENCUENTRA:

      if (color == "NINGUNO" && dFront > 10) {
        DELANTERAS_adelante(vGiro);
      } else if (color == "BLUE" || color == "YELLOW") {
        stop();
        EActual = FLAG;
        CambioEstado("FLAG", color, naranja, dFront);
      }
      break;

    case FLAG:

      if (color == "BLUE") {

        registroFLAGS("BLUE");
        EActual = DROP;

        CambioEstado("DROP", color, naranja, dFront);

      } else if (color == "YELLOW") {

        registroFLAGS("YELLOW");
        EActual = KICK;
        CambioEstado("KICK", color, naranja, dFront);
        
      }
      break;

    case KICK:

      if (color == "YELLOW" && !kicking) {

        kickStart = millis();
        kicking = true;
        DELANTERAS_adelante(vGiro);

      }
      if (kicking && millis() - kickStart >= 500) {

        stop();
        kicking = false;
        EActual = BUSQUEDA;
        CambioEstado("BUSQUEDA", color, naranja, dFront);
      }
      break;

    case DROP:

      for (angulo = 180; angulo >= 0; angulo--) servo1.write(angulo);
      delay(300);
      DELANTERAS_atras(vBase);
      delay(700);
      DELANTERAS_adelante(vBase);
      delay(800);
      EActual = FINAL;
      CambioEstado("FINAL", color, naranja, dFront);
      break;

    case FINAL:
      stop();
      break;
  }
}

void setup() {
  Serial.begin(115200);

  iniciarGiroscopio();
  servo1.attach(SERVO_PIN);

  pinMode(ADELANTE_1, OUTPUT);
  pinMode(ATRAS_1,OUTPUT);
  pinMode(ADELANTE_2,OUTPUT);
  pinMode(ATRAS_2,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);

  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);

  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  tcs.begin();
}

void loop() {
  unsigned long now = millis();

  if (now - previousMillis >= intervalo) {

    previousMillis = now;

    String color = SColor(); //funcion para el sensor de color
    String naranja = Naranja(); // funcion solamente para la pelota (deteccion de la pelota)

    float dFront, dLeft, dRight;
    leerUltrasonicos(dFront, dLeft, dRight);

    int16_t gx, gy, gz;
    leerGiroscopio(gx, gy, gz); //valores para el giroscopio

    evitarObstaculos(dFront, dLeft, dRight, color);

    mantenerDireccionPorGiro(gz, color);

    newEstado(color, dFront, naranja);

  }
}
