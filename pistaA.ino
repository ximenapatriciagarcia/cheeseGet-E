// PISTA A  -----DETECCION DE COLORES           

// Author: Ximena Garcia
//Pist A: deteccion de flags
#include <NewPing.h> //ultrasonicos
#include <Servo.h>
#include <DFRobot_BMI160.h>     //Giroscopio
#include <Wire.h>                       //giroscopio y sensor de color (I2C)
#include <Adafruit_TCS34725.h>              //sensor de color (flags)

//------------------------------- DECLARACION DE PINES--------------------
// (Puente H )  // 08-10-2025 ya estan arreglados >>> son los pines finales con PWM

const int ADELANTE_1 = 4;   // LLANTA 1    ADELANTE
const int ATRAS_1    = 5;   /

/    LLANTA 1  ATRAS
const int ADELANTE_2 = 7;   // LLANTA 2   ADELANTE
const int ATRAS_2    = 8;   //     LLANTA 2 ATRAS

const int PWM1 = 3;         // PWM motor 1
const int PWM2 = 6;         //      PWM motor 2 

//DECLARACION DE VARIABLES GLOBALES DE VELOCIDAD
int vBase = 100;
int vGiro = 80;

// Servo GARRA

const int SERVO_PIN = 9; 
Servo servo1;

int angulo = 0;

// Ultrasonico
const int trigPinLeft = 12;
const int echoPinLeft = 13;
const int trigPinRight = 14;
const int echoPinRight = 15;

//-------MILLIS--------------------------------
// para KICK 
unsigned long kickStart = 0;
bool kicking = false;

//variables para millis para garra para cerrar
bool cerrandoGarra = false;
unsigned long tiempoInicioCierre = 0;
unsigned long duracionCierre = 800;

//variables para millis para garra abrir
bool abriendoGarra = false;
unsigned long tiempoInicioAbrir = 0;
unsigned long duracionAbrir = 800;

//=======GIROSCOPIO BMI160 =====
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69; //check de la direccion


//grados inical
float z_offset = 0;
float yawActual = 0;
float yawInicial = 0;
float z_gyro = 0;

void calibrarGyro() {
  sBmi160SensorData_t gyroData;
  z_offset = 0;

  for (int i = 0; i < 100; i++) {
    bmi160.getGyroData(&gyroData);
    z_offset += gyroData.z;
    delay(10);
  }
  z_offset /= 100.0;
}

//para inicializar giroscopio
bool iniciarGiroscopio() {
  Wire.begin();

  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("ERROR:reinicio BMI160");
    return false;
    }

  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {

    Serial.println("ERROR:I2C BMI160");
    return false;

    }

  calibrarGyro();
  Serial.println("Giroscopio listo");
  return true;
}

//--------- SENSOR ULTRASONICO ------------
float Udistancia() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);


  long tiempo = pulseIn(echoPin, HIGH, 25000UL); // timeout 25ms (~4.3m)
  if (tiempo == 0) return 0.0;  // si no detecta
  float d = tiempo * 0.0343 / 2.0; // cm
  return d;
}

//======Sensor de COLOR TCS34725 (para FLAGS) quitar los colores no necesarios
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,  
  TCS34725_GAIN_4X                
);

// -------- FUNCION Sensor de color (me regresa un dato string utilizado para activar los condicionales)
String SColor() {
  uint16_t r, g, b, c; //color crudo
  float red, green, blue;

  // Se leen los valores RGBC
  tcs.getRawData(&r, &g, &b, &c);
  if (c == 0) c = 1;

  // Normalizar los colores
  red   = (float) r / c * 255.0;
  green = (float) g / c * 255.0;
  blue  = (float) b / c * 255.0;

  //debogueo

  Serial.print("red: "); Serial.print(red);
  Serial.print(" green: "); Serial.print(green);
  Serial.print(" blue: "); Serial.print(blue);
  Serial.print(" c: "); Serial.println(c);

  if (red > 120 && green < 100 && blue < 100)             
        return "RED";

  else if (red < 100 && green > 120 && blue < 100)        
            return "GREEN";

  else if (red < 100 && green < 100 && blue > 120)        
            return "BLUE";

  else if (red > 150 && green > 120 && blue < 80)         
            return "YELLOW";

  else         return "NINGUNO";
}



// Pines para el TCS230
const int S0 = 43;
const int S1 = 41;
const int S2 = 44;
const int S3 = 46;
const int sensorOut = 42;

//  TCS230

    unsigned long leerCanal(bool s2, bool s3) {
    digitalWrite(S2, s2 ? HIGH : LOW);

    digitalWrite(S3, s3 ? HIGH : LOW);

    unsigned long t = pulseIn(sensorOut, LOW, 50000UL); // 50ms
    if (t == 0) t = 50000UL; //para evitar lecturas nulas
    return t;
}





//para el de color naranja >>pelota
String Naranja() {

    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
//config colores
  // ROJO  S2=LOW  S3=LOW
  unsigned long pR = leerPeriodoCanal(false, false);
  // VERDE S2=HIGH S3=HIGH

  unsigned long pG = leerPeriodoCanal(true,  true);
  // AZUL  S2=LOW  S3=HIGH
  unsigned long pB = leerPeriodoCanal(false, true);

//esta es la config que necesito para el naranja
  bool rojoFuerte   = (pR < 1800);   

  bool verdeMedio   = (pG > 1500 && pG < 3500);

  bool azulDebil    = (pB > 3000);   

  if (rojoFuerte && verdeMedio && azulDebil) {

    return "ORANGE";

  } else {
    return "NONE"; //tener en cuenta que si no lee nada no se active el agarre
  }
}


//======================FUNCIONES para movimiento======

// MOTORES
void DELANTERAS_adelante(int vB) {  //ojo con el parameter de vB para l funcion

  digitalWrite(ADELANTE_2, HIGH);
  digitalWrite(ADELANTE_1, HIGH);
  analogWrite(PWM1, vB);
  analogWrite(PWM2, vB);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, LOW);

}

void DELANTERAS_atras(int vB = 60) {

  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, vB);
  analogWrite(PWM2, vB);
  digitalWrite(ATRAS_2, HIGH);
  digitalWrite(ATRAS_1, HIGH);

}

// ------------ giros 
void vuelta_Izquierda() { // IZQUIERDA ----------------

    digitalWrite(ADELANTE_2, HIGH);
    digitalWrite(ADELANTE_1, LOW);
    analogWrite(PWM1, vGiro);
    analogWrite(PWM2, vGiro);
    digitalWrite(ATRAS_2, LOW);
    digitalWrite(ATRAS_1, HIGH);
}

void vuelta_Derecha() {  // DERECHA  ------

  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, HIGH);
  analogWrite(PWM1, vGiro);
  analogWrite(PWM2, vGiro);
  digitalWrite(ATRAS_2, HIGH);
  digitalWrite(ATRAS_1, LOW);
}


// funciones para giros ligeros pero analizar
void ligero_IZ(){
    digitalWrite(ADELANTE_2, LOW);
    digitalWrite(ADELANTE_1, HIGH);
    analogWrite(PWM1, vGiro);
    analogWrite(PWM2, vGiro);
    digitalWrite(ATRAS_2, HIGH);
    digitalWrite(ATRAS_1, LOW);
}


void ligero_DE() {

    digitalWrite(ADELANTE_2, LOW);
    digitalWrite(ADELANTE_1, HIGH);
    analogWrite(PWM1, vGiro);
    analogWrite(PWM2, vGiro);
    digitalWrite(ATRAS_2, HIGH);
    digitalWrite(ATRAS_1, LOW);
}

// Stop
void stop() {

  digitalWrite(ADELANTE_2, LOW);
  digitalWrite(ADELANTE_1, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(ATRAS_2, LOW);
  digitalWrite(ATRAS_1, LOW);
}

//--------------------- FUNCION GARRA  --------
// Cambie a control por grados (suave)
void GARRA_CERRAR() {                                               // cierra de 0 a 180

  for (angulo = 0; angulo <= 180; angulo++) {
    servo1.write(angulo);
  }

}

void GARRA_ABRIR() { // abre de 180 a 0
  for (angulo = 180; angulo >= 0; angulo--) {
    servo1.write(angulo);

  }


}

//======================== MAQUINA DE ESTADOS ========================
// Pista A: Flags celestes y amarillas
//logica accion dependiendo del color
enum  Estado {
  INICIO,           // VERDE : solamente se activa una vez: busca la pelota para poder pasar a "BUSQUEDA" de flag
  BUSQUEDA,         // pelota: localiza pelota
  AVANZA_ENCUENTRA, // si aun no ha detectado ningun color
  FLAG,             // en busca de identificar la flag >> decision (KICK,DROP)
  KICK,             // AMARILLO (empujón corto)
  DROP,             // CELESTE
  FINAL             // ROJO: Se activa solamente una vez es el checkpoint


};

Estado EActual = INICIO; //principal 
Estado EPrevio = INICIO;  //para detectar cambios

// ----------------- registro de FLAGS (manipulacion de listas)
const int FLAGS = 5;    // numero total >> n-1
String flags[FLAGS];    // guarda "BLUE"  "YELLOW"
int flagCount = 0;     // contador

//imprimir lista completa de flags
void imprimirFlags() {

  Serial.print("flags detectadas: ");
  if (flagCount == 0) {
    Serial.println("NINGUNA");
    return;
  }
  for (int i = 0; i < flagCount; i++) {
    Serial.print(flags[i]);
    if (i < flagCount - 1);
  }

  Serial.println();
}

// Funcion para el registro de flags
void registroFLAGS(String color) {
  if (flagCount < FLAGS) {

    flags[flagCount] = color;
    flagCount++;
    imprimirFlags(); // imprimir cada vez que se actualiza

  }
}


//esta funcion me ayuda a detectar e cambio  de estado a debogguear
void CambioEstado(const char* Estado_e, const String& colorBandera, const String& colorPelota, float distancia) {

    Serial.print("ESTADO: ");
    Serial.print(Estado_e);

    Serial.print("color deflag: ");
    Serial.print(colorBandera);

    Serial.print("pelota detectada:");
    Serial.print(colorPelota);

    Serial.print("d:");
    Serial.print(distancia);

    imprimirFlags();
}

//-----logica para la maquina de estado
void newEstado(String color, float d, String naranja) {

//switch para la maquina de estado
  switch (EActual) {

    case INICIO:

      if (color == "GREEN") {
        EActual = BUSQUEDA;
        logCambioEstado("BUSQUEDA", color, naranja, d);
      }
      break;

    case BUSQUEDA:

      if (naranja != "ORANGE") {
        // segundo sensor de color>>> detecta solamente el color de la pelota
        DELANTERAS_adelante(vGiro); // vGiro  avanzar lento hasta detectar la pelota, ver si puedo aumentatr

      } else {
        stop();
        GARRA_CERRAR();
        EActual = AVANZA_ENCUENTRA; 
        CambioEstado("AVANZA_ENCUENTRA", color, naranja, d);
      }

      break;

    case AVANZA_ENCUENTRA:

      if (color=="NINGUNO" && d>10) {    // distancia para no chocar (ver si funciona)
        
        DELANTERAS_adelante(vGiro);

      }
      else if (color =="BLUE"||color =="YELLOW") {
        stop();
        EActual = FLAG;
        CambioEstado("FLAG", color, naranja, d);
      }
      break;

    case FLAG:
      if (color=="BLUE") {

        registroFLAGS("BLUE"); 
        EActual = DROP;
        CambioEstado("DROP", color, naranja, d);

      }
      else if (color=="YELLOW") {
        registroFLAGS("YELLOW"); 
        EActual = KICK;
        CambioEstado("KICK", color, naranja, d);

      }
      break;

    case KICK:
      if (color=="YELLOW" && !kicking) { 

        kickStart = millis();
        kicking = true;
        DELANTERAS_adelante(vGiro);

      }
      if (kicking && millis() - kickStart >= 500) { 
        stop();
        kicking = false;
        EActual = BUSQUEDA;
        CambioEstado("BUSQUEDA", color, naranja, d);

      }
      break;

    case DROP:
        GARRA_ABRIR(); //ver si funciona con delays
        delay(500);

        DELANTERAS_atras(vBase);
        delay(900);

        DELANTERAS_adelante(vBase);
        delay(1000); 

        EActual = FINAL; //se CAMBIA A ESTADO FINAL

        CambioEstado("FINAL", color, naranja, d);

      break;

    case FINAL:
      stop();
      break;
  }
}

//=======INICIALIZACION DEL PROGRAMA =============
void setup() {
  // Servo
  servo1.attach(SERVO_PIN);

  Serial.begin(9600);

  // Ultrasonico
    pinMode(trigPinLeft, OUTPUT); //cambiarlo porquue ahora estoy usado la biblioteca de Ping para poder leer al mismo tiempo Ultrasonicos
    pinMode(echoPinLeft, INPUT);
    pinMode(trigPinRight, OUTPUT);
    pinMode(echoPinRight, INPUT);

  // Motores
  pinMode(ADELANTE_1, OUTPUT);
  pinMode(ATRAS_1, OUTPUT);
  pinMode(ADELANTE_2, OUTPUT);
  pinMode(ATRAS_2, OUTPUT);

  // Iniciamos giroscopio aqui (una sola vez)
  iniciarGiroscopio();

  // TCS34725 
  if (tcs.begin()) {
    Serial.println("CHECK   :)  TCS34725 detectado");

  } else {
    Serial.println("ERROR: TCS34725 no detectado :(");
    while (1);
  }

  // TCS230
  pinMode(S0, OUTPUT); 
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); 
  pinMode(S3, OUTPUT);

  pinMode(sensorOut, INPUT);

//frecuencia para el de color
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);


  imprimirFlags();
  logCambioEstado("INICIO", "NINGUNO", "NINGUNA");
}


//----------------loop----------

void loop() {
//delaracion de los parametros
  String color = SColor();             
  float d = Udistancia(); //distancia en (m)             
  String naranja = leerTCS230_Naranja(); 

  newEstado(color, d, naranja);
}

