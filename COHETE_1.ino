#include <Wire.h>
#include <SPI.h>
#include <SD.h>    
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
//#include "DHT.h"

// Lista de dispositivos del Cohete 1 7/ABR/22
/*
   Módulo GPS G28U7FTTL
   Sensor BMP280
   Tarjeta SD
   EEPROM 24FC512
   Adxl345
   DTH11
   Apertura paracaidas POR SERVOMOTORES
   Alarma
*/

#if _SS_MAX_RX_BUFF != 128
#error "ERROR: No esta definido el buffer del SofwareSerial.h en 128 bytes"
#endif

//-------------------------------------------------
//              Parámetros Cohete
//-------------------------------------------------
#define ACELERACION_INICIO             2.0          // g
#define GRAVEDAD                       9.81         // m/s^2
#define TIEMPO_LANZAMIENTO             240000       // ms (Tiempo de espera desde que hay señal GPS)
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms
#define T_MIN_PARACAIDAS               4000         // ms
#define T_MAX_PARACAIDAS               13000        // ms
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms


//-------------------------------------------------
//             Declaración de pines
//-------------------------------------------------
#define PIN_MICRO_SD_CS   10
#define PIN_LED_ERROR     6
#define PIN_ALARMA        5 
#define TX_GPS            4
#define RX_GPS            3
#define SERVO_1           2  
#define SERVO_2           8  
//#define DHT11             7 


//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------

//Control apertura
float alt_max = 0.0;
uint16_t t_inicio = 0;
#define FLIGHT_TIME millis() - t_inicio

// Servomotores
Servo servoMotor1;
Servo servoMotor2;

// Sensor aceleración ADXL345
const int ADXL345 = 0x53; // Direccion I2C
float X_out, Y_out, Z_out;
#define OFFSET_X 0.00
#define OFFSET_Y 0.00
#define OFFSET_Z 0.16

// Sensor de presión BMP280
Adafruit_BMP280* bmp = new Adafruit_BMP280;

/*// Sensor de humedad DHT11
#define DHTTYPE DHT11
DHT *dht(DHT11, DHTTYPE);*/

// SD
File *archivo; 

// Módulo GPS G28U7FTTL
#define GPSBaud 9600
TinyGPSPlus *gps;
SoftwareSerial *ss(TX_GPS, RX_GPS);
#define GPS_SAT gps->satellites.value()
#define GPS_LAT gps->location.lat()
#define GPS_LON gps->location.lng()
#define GPS_ALT gps->altitude.meters()
#define GPS_SEC gps->time.second()
#define GPS_MIN gps->time.minute()
#define GPS_HOU gps->time.hour() + 1

// EEPROM I2C
#define EEPROM_I2C_ADDRESS 0x50
uint16_t eeprom_mem = 0;


//-------------------------------------------------
//       DECLARACION DE VARIABLES Y FUNCIONES
//-------------------------------------------------

//Control de apertura de paracaidas
bool condicion_aceleracion = false;
bool condicion_presion = false;
bool condicion_alarma = false;
unsigned int tiempo_de_vuelo; //CREO QUE ESTA NO LA UTILIZAS
float presion_min = 99999999999;

// Variables
float temperatura_BMP;
float altura_BMP;
float presion_BMP;
//float humedad_DHT11;
//float temperatura_DHT11;
float presion_referencia;
float latitud_gps;
float altura_gps;
float longitud_gps;
unsigned long tiempo;
//unsigned int tiempo_DHT11;

// Prototipos de funciones
void error_inicio();
void alarma_on();
void alarma_off();
void paracaidas_open();
boolean init_EEPROMI2C();
void gps_read();


void setup() 
{

// 1.- VERIFICACION DE FUNCIONAMIENTO DE LOS SENSORES

  Serial.begin(115200);

  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_ALARMA, OUTPUT);
  alarma_off();

// Verificación funcionamiento acelerómetro ADXL345
  ADXL345_16g_init();
  
// Verificación funcionamiento sensor de presión BMP280
  if (!bmp->begin()) 
  {
    Serial.print("Fallo en el sensor BMP280");
    error_inicio(); 
  }

/*// Inicio del sensor DHT11
  dht->begin();*/

// Verificación funcionamiento lector microSD
  if (!SD.begin(PIN_MICRO_SD_CS)) 
  {   
    Serial.print("Fallo en el lector microSD");
    error_inicio();
  }
  archivo = &(SD.open("datos.txt", FILE_WRITE));

// Verificacion del funcionamiento de la EEPROMI2C

  if (!init_EEPROMI2C() == NULL) 
  {
    Serial.println("Error EEPROM I2C");
    error_inicio();
  }

  Wire.begin();
  delay(50);
  

// 2.- ESPERANDO SEÑAL DEL GPS VÁLIDA

  ss->begin(GPSBaud);

  while (!gps->location.isValid()) 
  {
    while (ss->available()) 
    {
      gps->encode(ss->read());
    }
  } 


// 3.- PREVIO AL LANZAMIENTO

  for (uint8_t i = 0; i<50 ; i++)
   {
    presion_referencia += bmp->readPressure(); 
   }
    presion_referencia = presion_referencia/5000;
    // Serial.println(presion_referencia);


// 4.- ESPERAR TIEMPO DE LANZAMIENTO

  uint16_t var = millis();
  t_inicio = millis(); 
  while (true) 
  {    
    if ((millis() - var) >= TIEMPO_LANZAMIENTO) 
    {
      break;
    }
  }

  Serial.println("Inicialización correcta y establecida la presión de referencia, listos para el lanzamiento");
  alarma_on();
  delay(5000);
  alarma_off();
  digitalWrite(PIN_LED_ERROR, HIGH);
}


void loop() 
{

/********************************************************
                   LECTURA DE DATOS
*********************************************************/

// Tiempo
  tiempo = millis();

// Datos del BMP280
  temperatura_BMP = bmp->readTemperature();
  presion_BMP     = bmp->readPressure();
  altura_BMP      = bmp->readAltitude(1013.25); /* HAY QUE AJUSTAR LA PRESIÓN DE REFERENCIA */

  Serial.print(temperatura_BMP);
  Serial.print(",");
  Serial.print(presion_BMP);
  Serial.print(",");
  Serial.print(altura_BMP);
  Serial.print(",");

// Datos del ADXL345
  ADXL345_16g_read_acc();

  Serial.print(X_out);
  Serial.print(",");
  Serial.print(Y_out);
  Serial.print(",");
  Serial.print(Z_out);

/*// Datos del DHT11 // PROBAR ANTES LO DE MEDIR SÓLO CADA 2 O MÁS SEGUNDOS
  if(millis()-tiempo_DHT11 > 2000)
   {
   humedad_DHT11 = dht->readHumidity();
   temperatura_DHT11 = dht->readTemperature();
   tiempo_DHT11 = millis();
   }
   Serial.print(humedad_DHT11);
   Serial.print(",");
   Serial.println(temperatura_DHT11);*/

// Datos del GPS G28U7FTTL
  longitud_gps = GPS_LON;
  latitud_gps  = GPS_LAT;
  altura_gps   = GPS_ALT;

  Serial.print(longitud_gps);
  Serial.print(",");
  Serial.print(latitud_gps);
  Serial.print(",");
  Serial.println(altura_gps);
  

/********************************************************
                Escritura en microSD
*********************************************************/
  
  archivo->write("EE");
  archivo->write((byte*)&tiempo,4); //Revisar si el entero son 4 o 2 bytes
  archivo->write((byte*)&temperatura_BMP,4);  
  archivo->write((byte*)&presion_BMP,4); 
  archivo->write((byte*)&altura_BMP,4); 
  archivo->write((byte*)&X_out,4); 
  archivo->write((byte*)&Y_out,4); 
  archivo->write((byte*)&Z_out,4);
  archivo->write((byte*)(&altura_gps), 4);
  archivo->write((byte*)(&latitud_gps), 4);
  archivo->write((byte*)(&longitud_gps), 4);
//archivo->write((byte*)&humedad_DHT11,4);
//archivo->write((byte*)&temperatura_DHT11,4);

  archivo->flush();


/********************************************************
                Escritura en EEPROMI2C
*********************************************************/

  EEPROM_I2C_Almacena_datos();

  
/********************************************************
               APERTURA PARACAIDAS
*********************************************************/

  if (altura_BMP > alt_max && (FLIGHT_TIME < T_MIN_PARACAIDAS)) 
  {
    alt_max = altura_BMP;
  }

  if (!condicion_aceleracion && abs(Z_out) > ACELERACION_INICIO)  
  {
    condicion_aceleracion = true;
    tiempo_de_vuelo = millis();       
    digitalWrite(PIN_LED_ERROR, LOW);
    archivo->write("DD");
  }

   if ((alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS)
  {
    condicion_presion = true;
    paracaidas_open();
    digitalWrite(PIN_LED_ERROR, HIGH);
    archivo->write("AA");
    alt_max = altura_BMP;
  }

  if ( (alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA)  
  {
    alarma_on();
  }
  

  if (millis() - tiempo < 10)
  {
    delay(10-millis()+tiempo); //delay de lo que falta para llegar a 10
  }

}
