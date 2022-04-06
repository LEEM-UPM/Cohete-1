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
#define TIEMPO_LANZAMIENTO             20000         // ms (Tiempo de espera desde que hay señal GPS)
#define DIF_ALTURA_APERTURA            20.0         // m
#define DIF_ALTURA_ALARMA              200.0        // m
#define T_MIN_ALARMA                   30000        // ms
#define T_MIN_PARACAIDAS               10000         // ms
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
#define TX_GPS            3
#define RX_GPS            4
#define SERVO_1           2  
#define SERVO_2           8  
//#define DHT11             7 


//-------------------------------------------------
//              MODULOS Y SENSORES
//-------------------------------------------------

//Control apertura
float alt_max = 0.0;
uint32_t t_inicio = 0;
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
DHT  *dht= new DHT(DHT11, DHTTYPE);*/

// SD
File *archivo; 

// Módulo GPS G28U7FTTL
#define GPSBaud 9600
TinyGPSPlus *gps = new TinyGPSPlus();
SoftwareSerial *ss = new SoftwareSerial(TX_GPS, RX_GPS);
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
bool condicion_alarma = false;
bool condicion_apertura = false; 
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

  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_ALARMA, OUTPUT);
  alarma_off();

  Wire.begin();
  delay(50);

// Verificación funcionamiento acelerómetro ADXL345
  ADXL345_16g_init();
  
// Verificación funcionamiento sensor de presión BMP280
  if (!bmp->begin(0x76)) 
  {
    error_inicio(); 
  }

/*// Inicio del sensor DHT11
  dht->begin();*/

// Verificación funcionamiento lector microSD
  if (!SD.begin(PIN_MICRO_SD_CS)) 
  {   
    error_inicio();
  }
  archivo = &(SD.open("datos.txt", FILE_WRITE));

// Verificacion del funcionamiento de la EEPROMI2C

  if (!init_EEPROMI2C() == NULL) 
  {
    error_inicio();
  }
  alarma_on();
  delay(1000);
  alarma_off();
  

// 2.- ESPERANDO SEÑAL DEL GPS VÁLIDA

  ss->begin(GPSBaud);
  ss->print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss->print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss->print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss->print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss->print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss->print("$PUBX,40,RMC,0,0,0,0*47\r\n");

  while (!gps->location.isValid()) 
  {
    while (ss->available()) 
    {
      gps->encode(ss->read());
    }
  } 
  alarma_on();
  delay(4000);
  alarma_off();


// 3.- PREVIO AL LANZAMIENTO

  for (uint8_t i = 0; i<50 ; i++)
   {
    presion_referencia += bmp->readPressure(); 
   }
    presion_referencia = presion_referencia/5000;
    // Serial.println(presion_referencia);


// 4.- ESPERAR TIEMPO DE LANZAMIENTO

  uint32_t var = millis();
  t_inicio = millis(); 
  while (!((millis() - var) > TIEMPO_LANZAMIENTO)) 
  {    
    ADXL345_16g_read_acc();
    if (!condicion_aceleracion && (abs(Z_out) > ACELERACION_INICIO))
    {
      condicion_aceleracion = true; 
      archivo->write("DD");
      break;
    }
    delay(10);
  }
  alarma_on();
  delay(2000);
  alarma_off();
  delay(2000);
  alarma_on();
  delay(2000);
  alarma_off();
  delay(1000);
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

// Datos del ADXL345
  ADXL345_16g_read_acc();

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
  gps_read();
  longitud_gps = GPS_LON;
  latitud_gps  = GPS_LAT;
  altura_gps   = GPS_ALT;


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
    t_inicio= millis();       
    digitalWrite(PIN_LED_ERROR, LOW);
    archivo->write("DD");
  }

   if (!condicion_apertura && condicion_aceleracion && (alt_max > (altura_BMP + DIF_ALTURA_APERTURA) && FLIGHT_TIME < T_MIN_PARACAIDAS)  ||  FLIGHT_TIME > T_MAX_PARACAIDAS)
  {
    condicion_apertura = true;
    paracaidas_open();
    digitalWrite(PIN_LED_ERROR, HIGH);
    archivo->write("AA");
    alt_max = altura_BMP;
  }

  if (!condicion_alarma && ((alt_max > (altura_BMP + DIF_ALTURA_ALARMA)) || FLIGHT_TIME > T_MIN_ALARMA))  
  {
    condicion_alarma = true;
    alarma_on();
  }
  

  if (millis() - tiempo < 10)
  {
    delay(10-millis()+tiempo); //delay de lo que falta para llegar a 10
  }
}
