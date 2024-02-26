#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
//#include <RTClib.h>


File myFile;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(18)) {
  Serial.println("initialization failed!");
  while (1);

    if(SD.exists("data.csv")){
    SD.remove("data.csv");
  }
  
  }

  
  
  Serial.println(" SD IS CHILL :3");
 
  unsigned status;
    
    //bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
    
    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!"));
      while (1) 
      delay(10);
    }
    

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
     
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  Serial.println("initialization done. :3");
}



void loop() {
  sensors_event_t temp_event, pressure_event;

  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);


  myFile = SD.open("data.csv", FILE_WRITE);
  int fishSense = analogRead(A1);

  if (myFile) {
    //myFile.print("Temp: ");
    myFile.print(temp_event.temperature);
    myFile.print("\t");
    //myFile.print("Pres: ");
 
    myFile.println(pressure_event.pressure);

    myFile.println(fishSense);
  
    Serial.print(temp_event.temperature);
    Serial.print(",");
    Serial.print(pressure_event.pressure);
    Serial.print(",");
    Serial.println(fishSense);
  
   myFile.close();

   delay(2000);
  } 
  else {
  // if the file didn't open, print an error:
  Serial.println("error opening CANSAT_DATA.txt");
  }

}
