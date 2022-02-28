#include <Wire.h>
#include <SPI.h>
#include <SD.h>    
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SSpin 10
#define eeprom 0x50 // Dirección base EEPROM

File archivo; 
Servo servoMotor;

Adafruit_BMP280 bmp;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();


// Control de la apertura del paracaidas

bool con_acc = false;
bool con_pres = false;
int flight_time;
float pres_min = 99999999999;

void error_inicio() {
  while (true) {
    digitalWrite(7, HIGH);
    delay(200);
    digitalWrite(7, LOW);
    delay(200);
  }
}

void setup() {
  
 Serial.begin(115200);
 servoMotor.attach(2);
 servoMotor.write(0);

   unsigned int address = 0; //Primera dirección de la EEPROM


 
 accel.setRange(ADXL345_RANGE_16_G);

 pinMode(7, OUTPUT);
  delay(2000);

  if (!SD.begin(SSpin)) {     
    Serial.println("Fallo al iniciar el lector SD");
    error_inicio();
  }

  if (!bmp.begin()) {
    Serial.println(F("Fallo en el sensor BMP280"));
    error_inicio();
  }
  
if(!accel.begin())
   {
      Serial.println("Fallo en el sensor ADXL345");
      error_inicio();
   }
  
archivo = SD.open("datos.txt", FILE_WRITE);

}


void loop(){
  
   digitalWrite(7, HIGH);
   
   
  // Datos del BMP280

  Serial.print(bmp.readTemperature());  // En grados
  Serial.print(",");
  Serial.print(bmp.readPressure()); // En Pascales
  Serial.print(",");
  Serial.print(bmp.readAltitude(1013.25)); /* HAY QUE AJUSTAR LA PRESIÓN DE REFERENCIA */
  Serial.print(",");
  

 // Datos del ADXL345

   sensors_event_t event; 
   accel.getEvent(&event);
   Serial.print(event.acceleration.x/9.81); 
   Serial.print(",");
   Serial.print(event.acceleration.y/9.81); 
   Serial.print(",");
   Serial.print(event.acceleration.z/9.81); 
   Serial.println(",");
   
 
  // Escritura SD

  archivo.print(bmp.readTemperature());  // En grados
  archivo.print(",");
  archivo.print(bmp.readPressure()); // En Pascales
  archivo.print(",");
  archivo.print(bmp.readAltitude(1013.25)); /* HAY QUE AJUSTAR LA PRESIÓN DE REFERENCIA */
  archivo.print(",");
  archivo.print(event.acceleration.x/9.81); // En g
  archivo.print(",");
  archivo.print(event.acceleration.y/9.81); 
  archivo.print(",");
  archivo.print(event.acceleration.z/9.81); 
  archivo.println(",");
  archivo.flush();


  // Paracaidas
  if (  Presion < pres_min)
  {
    pres_min = Presion;
  }

  if (!con_acc && abs(X_out) > 5)
  {
    con_acc = true;
    flight_time = millis();
    digitalWrite(7, LOW);
  }
  if (!con_pres && con_acc && (pres_min < 0.9 * Presion || (millis - flight_time) > 10000))
  {
    con_pres = true;
    servoMotor.write(110);
  }

  delay(10);

}
  
}
