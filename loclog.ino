#include <LCD5110_Basic.h>
#include <AltSoftSerial.h>
#include <SdFat.h>
#include <avr/pgmspace.h>

// SD card
const int chipSelect = 5;
// GPS Serial
const int gpsRX = 8, gpsTX = 9; //fixed assignment by AltSoftSerial
// LCD SPI
const int lcdCLK = 3, lcdDI = 4, lcdDC = 7, lcdCS = 6, lcdRST = 2;

// *** functions *********************************************************

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);  
}

void print_free_ram() {
  Serial.print(F("Free SRAM = "));
  Serial.println(freeRam());
}

SIGNAL(TIMER0_COMPA_vect) {
  //  if (gpSerial.available()) {
  //    int c = gpSerial.read();
  //    if (c) {
  //      UDR0 = c;  
  //    }
  //  }

}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  //I've to split all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  //Serial.println(dist_calc);
  return dist_calc;
}

inline ofstream open_new_logfile(SdFat &sd, char *logfile_name) {
  uint8_t lognr = 0;
  //  char logfile_name[13];
  // open new logfile
  while (lognr == 0 || sd.exists(logfile_name))  {
    lognr++;
    sprintf(logfile_name, "log%05i.txt", lognr);
  }

  Serial.print(F("Open file "));
  Serial.print(logfile_name);
  Serial.println(F(" for logging."));

  return ofstream(logfile_name, ios::out | ios::app);
}

inline void setup_gpserial(AltSoftSerial &gpSerial) {
  gpSerial.begin(9600);
  delay(100);
  gpSerial.println(F("$PMTK251,38400*27"));
  delay(100);
  gpSerial.end();
  delay(1000);
  gpSerial.begin(38400);
  // configure update rate 
  //  gpSerial.println(F("$PMTK220,200*2C")); // 5hz
  gpSerial.println(F("$PMTK220,100*2F")); // 10hz
  // configure NMEA sentences frequency
  // - No GLL, 10Hz RMC, No VTG, 2Hz GGA, GSA, GSV
  delay(100);
  gpSerial.println(F("$PMTK314,0,1,0,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C"));
  // enable DGPS (EGNOS)
  delay(100);
  gpSerial.println(F("$PMTK313,1*2E"));
  delay(100);
  gpSerial.println(F("$PMTK319,0*25"));
  delay(100);
  gpSerial.println(F("$PMTK513,1*28"));
  delay(100);
  // no antenna status
  gpSerial.println(F("$PGCMD,33,0*6D"));
}

// *** glocal objects ***************************************************

LCD5110 lcd(lcdCLK,lcdDI,lcdDC,lcdRST,lcdCS);
//Adafruit_PCD8544 display = Adafruit_PCD8544(lcdCLK, lcdDI, lcdDO, lcdCS, lcdRST);
AltSoftSerial gpSerial;
SdFat sd;
ofstream sdout;
char logfile_name[13];

extern uint8_t SmallFont[];

uint32_t timer = millis();
uint32_t distance = 0;
float lastLat=0, lastLon=0;

char c;
char checksum_char;
uint8_t checksum_step = 0;
String checksum_string;
String checksum_reference;
String nmea_sentence;
uint16_t sentences_all = 0;
uint16_t sentences_valid = 0;
uint16_t sentences_invalid = 0;
uint32_t bytes_read = 0, bytes_written = 0;

// *** setup *************************************************************

void setup() {

  Serial.begin(115200);
  Serial.println(F("LocLog v. 0.2 starting..."));

  lcd.InitLCD();
  lcd.setFont(SmallFont);
  lcd.print("Starting...", LEFT, 0);

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (sd.begin(chipSelect, SPI_HALF_SPEED))
    sdout = open_new_logfile(sd,logfile_name);
  else
    sd.initErrorHalt();
  pinMode(10,OUTPUT);

  setup_gpserial(gpSerial);

  print_free_ram();

  Serial.println(F("LocLog startup completed."));

  // finished
  delay(1000);

}

// *** loop *************************************************************

void loop() // run over and over
{

  if (digitalRead(5) == LOW) {
    Serial.println("********");
  }
  if (gpSerial.available()) {
    c = gpSerial.read();
    bytes_read++;
    nmea_sentence += c;
    Serial.print(c);
    switch (c) {
    case '$': 
      checksum_step = 1;
      break;
    case '*': 
      checksum_step = 3;
      checksum_string = "";
      break;
    case '\r':
      break;
    case '\n': 
      checksum_step = 4;
      checksum_reference = String(checksum_char, HEX);
      if (checksum_reference.length() == 1)
        checksum_reference = "0" + checksum_reference;
      if (checksum_string.equalsIgnoreCase(checksum_reference) == true) {
        sentences_valid++;
      } 
      else {
        sentences_invalid++;
      }
      sentences_all++;

      Serial.print(nmea_sentence);
      sdout << nmea_sentence.c_str();
      sdout.flush();

      if (sentences_all % 50 == 0) {
        SdFile file;
        file.open(logfile_name, O_RDWR);
        bytes_written = file.fileSize();
        file.close();

        Serial.print("*** Stat: ");
        Serial.print(sentences_valid);
        Serial.print("/");
        Serial.print(sentences_invalid);
        Serial.print("/");
        Serial.print(bytes_read);
        Serial.print("/");
        Serial.print(bytes_written);
        Serial.println("");
        
        char buffer[16];
        lcd.clrScr();
        sprintf(buffer, "OK:   %6i", sentences_valid);
        lcd.print(buffer, LEFT, 0);
        sprintf(buffer, "BAD:  %6i", sentences_invalid);
        lcd.print(buffer, LEFT, 8);       
        sprintf(buffer, "READ: %6ik", bytes_read/1024);
        lcd.print(buffer, LEFT, 16);       
        sprintf(buffer, "WRITE:%6ik", bytes_written/1024);
        lcd.print(buffer, LEFT, 24);   
        sprintf(buffer, "RUN:  %6is", millis()/1000);    
        lcd.print(buffer, LEFT, 32);   


      }
      nmea_sentence = "";
      break;
    default:
      switch(checksum_step) {
      case 1:
        checksum_char = c;
        checksum_step = 2;
        break;
      case 2:
        checksum_char ^= c;
        break;
      case 3:
        checksum_string += c;
        break;
      }
      break;
    }
  }

  if (millis() - timer > 60000) {
    timer = millis();
    Serial.println("*** 1min divider ***");
  }
}

// if a sentence is received, we can check the checksum, parse it...
//  if (GPS.newNMEAreceived()) {
// a tricky thing here is if we print the NMEA sentence, or data
// we end up not listening and catching other sentences! 
// so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

//    buffer += GPS.lastNMEA();
//    buffer += "\n";
//    nmea_count++;
//   if (!GPS.parse(GPS.lastNMEA())) 
//     nmea_fail++;

//   ofstream sdout("log2.txt", ios::out | ios::app);
//   sdout << GPS.lastNMEA();
//   sdout.close();

// Serial.println(GPS.lastNMEA());
//    logFile = SD.open("test4.txt", FILE_WRITE);
//    Serial.print(GPS.lastNMEA());
//    logFile.print(GPS.lastNMEA());
//    logFile.close();

//if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
// return;  // we can fail to parse a sentence in which case we should just wait for another


//    print_free_ram();
//    if (lastLat!=0 && lastLon !=0) {
//      distance += calc_dist(GPS.latitude/100.0, GPS.longitude/100.0, lastLat/100.0, lastLon/100.0);
//    }
//    lastLat = GPS.latitude;
//    lastLon = GPS.longitude;

//    logFile = SD.open("test4.txt", FILE_WRITE);
//    Serial.print(buffer);
//    logFile.print(buffer);
//    buffer = "";
//    logFile.close();

//    char buffer[64];
//    if (GPS.fix) {
//      sprintf_P(buffer, PSTR("%02i.%02i.%02i %02i:%02i\nSAT: %02i\nNMEA: %i/%i\nUptime: %is\n"), 
//      GPS.day, GPS.month, GPS.year, GPS.hour, GPS.minute, 
//      (int)GPS.satellites, nmea_fail, nmea_count, millis()/1000);
//    }
//    else
//      sprintf_P(buffer, PSTR("%02i.%02i.%02i %02i:%02i\nNO FIX\nNMEA: %i/%i\nUptime: %is\n"), 
//      GPS.day, GPS.month, GPS.year, GPS.hour, GPS.minute, 
//      nmea_fail, nmea_count, millis()/1000);
//    print_lcd(buffer);
//Serial.print(buffer);

//    Serial.print("\n");
//    Serial.print(GPS.hour, DEC); 
//    Serial.print(':');
//    Serial.print(GPS.minute, DEC); 
//    Serial.print(':');
//    Serial.print(GPS.seconds, DEC); 
//    Serial.print('.');
//    Serial.print(GPS.milliseconds);
//    Serial.print(" ");
//    Serial.print(GPS.day, DEC); 
//    Serial.print('.');
//    Serial.print(GPS.month, DEC); 
//    Serial.print(".20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); 
//    Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); 
//    Serial.println((int)GPS.fixquality); 
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); 
//      Serial.print(GPS.lat);
//      Serial.print(", "); 
//      Serial.print(GPS.longitude, 4); 
//      Serial.println(GPS.lon);
//      
//      Serial.print("Distance: ");
//      Serial.print(distance, 4);
//      Serial.println("m");
//
//      Serial.print("Speed (knots): "); 
//      Serial.println(GPS.speed);
//      Serial.print("Angle: "); 
//      Serial.println(GPS.angle);
//      Serial.print("Altitude: "); 
//      Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); 
//      Serial.println((int)GPS.satellites);
//    }


































