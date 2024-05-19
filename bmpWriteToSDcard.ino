#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <dht.h>


const int fs = A1;
const int four = A3;
const int seven = A2;
const int nine = A0;

dht DHT;

#define DHT11_PIN 6
#define LED_PIN 5

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

File myFile;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


void setup() {
  Wire.begin();  //sets up the I2C
  rtc.begin(); 
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 
  // Open serial communications and wait for port to open:
  //Serial.begin(9600);
  /*
  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
 */
  
  //Serial.print("Initializing SD card...");
  if (!SD.begin(7)) {
  //println("initialization failed!");
    while (1);
    delay(10);
  }
  /*
  if(SD.exists("DATA.csv")){
    SD.remove("DATA.csv");
  }
 */
  unsigned status;
    
    //bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
    
    if (!status) {
      //Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
      while (1);
      delay(10);
    }
    

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
     
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  //Serial.println("initialization done. :3");

  if (! rtc.begin()) {
    //Serial.println("Couldn't find RTC");
    //Serial.flush();
    while (1);
    delay(10);
  }


  pinMode(LED_PIN, OUTPUT);
  
}



void loop() {
  
  sensors_event_t temp_event, pressure_event;
  
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  DateTime now = rtc.now();
  
  static String fileName = String(now.day()) + String(now.hour()) + String(now.minute()) + String(now.second())+ ".csv";
  
  myFile = SD.open(fileName, FILE_WRITE);
  int fishSense = analogRead(fs);
  int fishNine = analogRead(nine);
  int fishFour = analogRead(four);
  int fishSeven = analogRead(seven);

  int chk = DHT.read11(DHT11_PIN);

  delay (1000);
  digitalWrite(LED_PIN, HIGH);
  
  if (myFile) {
    //myFile.print("Temp: ");
    myFile.print(rtc.getTemperature());
    myFile.print(",");
    myFile.print(temp_event.temperature);
    myFile.print(",");
    myFile.print(pressure_event.pressure);
    myFile.print(",");
    myFile.print(fishSense);
    myFile.print(",");
    
    myFile.print(fishNine);
    myFile.print(",");
    myFile.print(fishFour);
    myFile.print(",");
    myFile.print(fishSeven);
    myFile.print(",");
    myFile.print(DHT.temperature);
    myFile.print(",");
    myFile.print(DHT.humidity);
    myFile.print(",");
    
    myFile.print(now.day(), DEC);
    myFile.print(" ");
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.println();    
    
    // Wait one second before repeating
    delay (1000);
    
    
    myFile.close();
    digitalWrite(LED_PIN, LOW);
    
   
  } 

  /*
  else {
  // if the file didn't open, print an error:
  //Serial.println("error opening CANSAT_DATA.txt");
  }
  */
    
    /*
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C");

    Serial.println();
    */
    
    
}
