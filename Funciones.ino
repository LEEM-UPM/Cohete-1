/********************************************************
                ERROR EN ALGÚN SISTEMA
*********************************************************/

void error_inicio() 
{
  while (true) 
  {
    alarma_on();
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(200);
    alarma_off();
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(200);
  }
}


/********************************************************
                        ALARMA
*********************************************************/

void alarma_off() 
{
  digitalWrite(PIN_ALARMA, HIGH);
}

void alarma_on() 
{
  digitalWrite(PIN_ALARMA, LOW);
}


/********************************************************
                      PARACAÍDAS
*********************************************************/

void paracaidas_open() 
{
  servoMotor1.attach(SERVO_1);
  servoMotor2.attach(SERVO_2);
}


/********************************************************
                ACELEROMETRO ADXL345
*********************************************************/
void ADXL345_16g_read_acc() 
{
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true);
  X_out = ( Wire.read() | Wire.read() << 8);
  X_out = X_out / 32 - OFFSET_X;
  Y_out = ( Wire.read() | Wire.read() << 8);
  Y_out = Y_out / 32 - OFFSET_Y;
  Z_out = ( Wire.read() | Wire.read() << 8);
  Z_out = Z_out / 32 - OFFSET_Z;
}

boolean ADXL345_16g_init() 
{
  
  Wire.beginTransmission(ADXL345);
  if (Wire.endTransmission() == 2) {
    return 0;
  }
  
  // Inicio de la comunicación
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();
  delay(10);

  // Rango máximo +-16g
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31);
  Wire.write(B00000011);
  Wire.endTransmission();
  
  return 1;
}


/********************************************************
                    GPS G28U7FTTL
*********************************************************/

void gps_read() {
  char aaa = ss->peek();
  while (ss->available()) 
  {
    aaa = ss->read();
    //Serial.write(aaa);
    gps->encode(aaa);
  }
}

/*
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
*/


/********************************************************
                      EEPROM EXTERNA
*********************************************************/

boolean init_EEPROMI2C() 
{
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) 
  {
    return 1;
  }
  return 0;
}

void EEPROM_I2C_Almacena_datos() 
{
  byte paquete[30];  // No se pueden guardar paquetes superiores a 30 bytes
  float_to_4byte(&Z_out, &(paquete[0]));
  float_to_4byte(&X_out, &(paquete[4]));
  float_to_4byte(&Y_out, &(paquete[8]));
  float_to_4byte(&presion_BMP, &(paquete[12]));
  float_to_4byte(&altura_BMP, &(paquete[16]));
  float_to_4byte(&latitud_gps, &(paquete[20]));
  float_to_4byte(&longitud_gps, &(paquete[24]));
  float_to_4byte(&altura_gps, &(paquete[24]));
  uint16_to_2byte(tiempo, &(paquete[28]));

  writeEEPROM_Page(eeprom_mem, paquete, 30);
  eeprom_mem += 30;
}

void float_to_4byte(float* var, byte* aux) 
{
  byte* p = (byte*)var;
  for (char i = 3; i >= 0; i--) 
  {
    *(aux + i) = *p;
    p++;
  }
}

void uint16_to_2byte(uint16_t dato_in, byte* dir_dato_out) 
{
  *(dir_dato_out) = (byte)dato_in;
  *(dir_dato_out + 1) = (byte)(dato_in >> 8);
}

void writeEEPROM_Page(uint16_t address, byte *val, uint16_t tam) 
{
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((uint8_t)(address >> 8));   // MSB
  Wire.write((uint8_t)(address & 0xFF)); // LSB
  for (uint16_t i = 0; i < tam; i++) 
  {
    Wire.write(*val);
    val++;
  }
  Wire.endTransmission();
}
