//senzor altitudine/presiuna Pa MPL 3115A2
//senzor metan MQ-4
//senzor temperatura/umiditate HIH-6130
//Licence: Marin Andrei 2015

#include <Wire.h>
//#include <MPL3115A2.h>
#include <SparkFunMPL3115A2.h>
#include <SoftwareSerial.h>

int bluetoothTx = 2;
int bluetoothRx = 3;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

byte fetch_humidity_temperature(unsigned int *p_Humidity, unsigned int *p_Temperature);
void print_float(float f, int num_digits);

#define TRUE 1
#define FALSE 0

MPL3115A2 myPressure;

char pastring[10];
char tmpstring[10];

float pressure;
float temperature;

int ipress;
int itemp;

void setup()
{
  //bluetooth
  Serial.begin(9600);
  
  bluetooth.begin(115200);
  bluetooth.print("$$$");
  delay(500);
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);

  Wire.begin();   
  
  myPressure.begin();

  //sensor HIH-6130
  
  pinMode (4, OUTPUT);
  digitalWrite (4, HIGH);
  delay(500);

   myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  Serial.begin(9600);
  delay(500);
}

int senzor_Value;

void loop()
{
  //senzor MQ-4
  
  senzor_Value = analogRead(A0);
  bluetooth.print ("Valoare metan (MQ-4): ");
  bluetooth.print (analogRead(A0), DEC);
  bluetooth.print (" ppm");
  bluetooth.println();

  bluetooth.println();
  delay (500);

  //senzor MPL3115A2

  float pressure = myPressure.readPressure();
  int ipress = pressure;
    
  bluetooth.print("Presiune(kPa): ");
  bluetooth.print(pressure/1000);

  temperature = myPressure.readTempF();
  itemp = temperature;

  //display.write("Temperature(F): ");
  //display.write(tmpstring);

  float tempC = (myPressure.readTempF() - 32) * 5/9;

  const float sea_press = 1013.25;
  float altitude = myPressure.readAltitude();
  
  float altituded = ((pow((sea_press /pressure), 1/5.257) - 1.0) * (tempC + 273.15)) / 0.0065;

  bluetooth.print("    Altitude(m): ");
  bluetooth.print(altitude/1000);

  float altitude_ft = myPressure.readAltitudeFt();
  bluetooth.print("    Altitudine(Feet): ");
  bluetooth.print(altitude_ft/1000,2);
  bluetooth.println();

  bluetooth.print("Temperature(C): ");
  bluetooth.print(tempC, 2);

  float celsius = myPressure.readTemp();

  float tempf = myPressure.readTempF();
  bluetooth.print("    Temperature(F): ");
  bluetooth.print(tempf);
  bluetooth.println();

  bluetooth.println();

  delay(500);

 //senzor HIH_6130
   
  pressure = myPressure.readPressure();
  ipress = pressure;
  
  byte _status;
   unsigned int H_dat, T_dat;
   float RH, T_C;
   
      _status = fetch_humidity_temperature(&H_dat, &T_dat);
      
      switch(_status)
      {
          case 0:  Serial.println("Normal.");
                   break;
          case 1:  Serial.println("Stale Data.");
                   break;
          case 2:  Serial.println("In command mode.");
                   break;
          default: Serial.println("Diagnostic."); 
                   break; 
      }       
    
      RH = (float) H_dat * 6.10e-3;
      T_C = (float) T_dat * 1.007e-2 - 40.0;

      bluetooth.print ("Humidity: ");
      bluetooth.print (RH, 2);
      bluetooth.print("% ");
      bluetooth.print("  ");
      bluetooth.print("Temperature: ");
      bluetooth.print (T_C, 2);
      bluetooth.print(" C");
     
      bluetooth.println();
      bluetooth.println(">>><<<<<<<<<<<<<<");
      bluetooth.println();
      
      delay(500);
}

byte fetch_humidity_temperature(unsigned int *p_H_dat, unsigned int *p_T_dat)
{
      byte address, Hum_H, Hum_L, Temp_H, Temp_L, _status;
      unsigned int H_dat, T_dat;
      address = 0x27;;
      Wire.beginTransmission(address); 
      Wire.endTransmission();
      delay(100);
      
      Wire.requestFrom((int)address, (int) 4);
      Hum_H = Wire.read();
      Hum_L = Wire.read();
      Temp_H = Wire.read();
      Temp_L = Wire.read();
      Wire.endTransmission();
      
      _status = (Hum_H >> 6) & 0x03;
      Hum_H = Hum_H & 0x3f;
      H_dat = (((unsigned int)Hum_H) << 8) | Hum_L;
      T_dat = (((unsigned int)Temp_H) << 8) | Temp_L;
      T_dat = T_dat / 4;
      *p_H_dat = H_dat;
      *p_T_dat = T_dat;
      return(_status);
}
   
void print_float(float f, int num_digits)
{
    int f_int;
    int pows_of_ten[4] = {1, 10, 100, 1000};
    int multiplier, whole, fract, d, n;

    multiplier = pows_of_ten[num_digits];
    if (f < 0.0)
    {
        f = -f;
        bluetooth.print("-");
    }
    whole = (int) f;
    fract = (int) (multiplier * (f - (float)whole));

    bluetooth.print(whole);
    bluetooth.print(".");

    for (n=num_digits-1; n>=0; n--) // print each digit with no leading zero suppression
    {
         d = fract / pows_of_ten[n];
         bluetooth.print(d);
         fract = fract % pows_of_ten[n];
    }
}      


