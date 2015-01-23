#include <AltSoftSerial.h>
#include <SdFat.h>
#include <avr/pgmspace.h>

// *** glocal objects ***************************************************
// TODO: Parameter auf SD-Karte speichern

const int GPS_RATE = 1; // 1, 5, 10
const int DEBUG_RAM = 0; // 1, 0

// *** glocal objects ***************************************************

// *** functions *********************************************************

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);  
}

void print_free_ram() {
  if (DEBUG_RAM == 1) {
    Serial.print(F("LocLog: "));
    Serial.print(F("Free SRAM = "));
    Serial.println(freeRam());
  }
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

inline void sd_get_new_logfile(SdFat &sd, char *logfile_name) {
  SdFile f;
  uint8_t lognr = 0;
  while (lognr == 0 || sd.exists(logfile_name))  {
    lognr++;
    sprintf(logfile_name, "log%05i.nmea", lognr);
  }

  Serial.print(F("LocLog: "));
  Serial.print(F("Open file "));
  Serial.print(logfile_name);
  Serial.println(F(" for logging."));
}

inline void gps_setup(AltSoftSerial &gpSerial) {
  // connect to gps
  gpSerial.begin(9600);
  delay(100);
  // set up baud rate 38400
  gpSerial.println(F("$PMTK251,38400*27"));
  delay(100);
  gpSerial.end();
  delay(1000);
  // re-connect
  gpSerial.begin(38400);
  // configure update rate 
  if (GPS_RATE == 1)
    gpSerial.println(F("$PMTK220,1000*1F")); // 1hz
  if (GPS_RATE == 5) 
    gpSerial.println(F("$PMTK220,200*2C")); // 5hz
  else if (GPS_RATE == 10)
    gpSerial.println(F("$PMTK220,100*2F")); // 10hz
  // configure NMEA sentences frequency
  // - No GLL, 10Hz RMC, No VTG, 2Hz GGA, GSA, GSV
  delay(100);
  gpSerial.println(F("$PMTK314,0,1,0,5,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C"));
  delay(100);
  // enable SBAS
  gpSerial.println(F("$PMTK313,1*2E"));
  delay(100);
  // allow SBAS testing signals
  gpSerial.println(F("$PMTK319,0*25"));
  delay(100);
  // search for SBAS sats
  gpSerial.println(F("$PMTK513,1*28"));
  delay(100);
  // no antenna status
  gpSerial.println(F("$PGCMD,33,0*6D"));
}

inline void command_directory_listing(SdFat *sd) {
  sd->ls(LS_SIZE);
  Serial.print(F("LocLog: Listing completed."));
}

inline void command_retrieve_log(SdFat *sd, const char* file) {
  Serial.print(F("LocLog: Retrieving file "));
  Serial.println(file);
  if (sd->exists(file)) {
    SdFile logfile;    
    if (logfile.open(file, O_READ)) {
      char buf;
      while ((buf = logfile.read()) >= 0)
        Serial.write(buf);
      Serial.print(F("LocLog: End of log."));  
      logfile.close();
    } else {
      Serial.print(F("LocLog: Could not open file."));   
    }
  } else {
    Serial.print(F("LocLog: File not found."));  
  }  
}

// *** glocal objects (used only in setup and loop) *********************

AltSoftSerial gpSerial;
SdFat sd;
SdFile file;
ofstream sdout;
char logfile_name[13];
bool hasSDCard = false;
bool isLogging = false; //TODO only log when fix available
// TODO: name log file to fix date
// TODO: log time to first fix

uint32_t timer = millis();
uint32_t distance = 0;
float lastLat=0, lastLon=0;

char c;
char checksum_char;
uint8_t checksum_step = 0;
String checksum_string;
String checksum_reference;
String nmea_sentence;
String command;
uint16_t sentences_all = 0;
uint16_t sentences_valid = 0;
uint16_t sentences_invalid = 0;
uint32_t bytes_read = 0, bytes_written = 0;
uint8_t menu_page = 0;

// *** setup *************************************************************

void setup() {

  Serial.begin(57600*2);
  Serial.print(F("LocLog: "));
  Serial.println(F("LocLog v. 1.1 starting..."));

  // enable GPS chip
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(1000);

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (sd.begin(10, SPI_FULL_SPEED)) {
    hasSDCard = true;
  } else {
    //sd.initErrorHalt();
    Serial.print(F("LocLog: "));
    Serial.println(F("No SD CARD. Logging disabled!"));
    hasSDCard = false;
  }
  pinMode(10, OUTPUT);

  gps_setup(gpSerial);

  print_free_ram();

  Serial.print(F("LocLog: "));
  Serial.println(F("Startup completed."));

  // finished
  delay(1000);
}

// *** loop *************************************************************

void loop() // run over and over
{
  // read and process data from gps
  if (gpSerial.available()) {
    c = gpSerial.read();
    bytes_read++;
    if (c != '\r')
      nmea_sentence += c;
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

      // write this sentence to bluetooth and sd card
      Serial.print(nmea_sentence);
      if (!nmea_sentence.startsWith("$GPGSV") && hasSDCard) {
        if (!isLogging) {
          int date_stop = nmea_sentence.lastIndexOf(',', nmea_sentence.lastIndexOf(',', nmea_sentence.lastIndexOf(',')-1)-1);
          if (nmea_sentence[date_stop-2] == '1') {
            String date = nmea_sentence.substring(date_stop-2, date_stop);
            date = date + nmea_sentence.substring(date_stop-4, date_stop-2);
            date = date + nmea_sentence.substring(date_stop-6, date_stop-4);
            date = date + nmea_sentence.substring(7, 9);
            date = date + ".nme";
            date.toCharArray(logfile_name, 13);
                          Serial.print("File:");
                          Serial.print(logfile_name);
            if (!file.open(logfile_name, O_RDWR | O_CREAT | O_AT_END)) {
              Serial.print(F("LocLog: "));
              Serial.println(F("Could not write to SD CARD. Logging disabled."));
              hasSDCard = false;
            } else {
              Serial.print(F("LocLog: "));
              Serial.print(F("Open file "));
              Serial.print(logfile_name);
              Serial.println(F(" for logging."));
              isLogging = true;
            }
          }          
        } else {
          file.print(nmea_sentence);
        }
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
  
  // periodic stats
  if (millis() - timer > 10000) {
    Serial.print(F("LocLog: "));
    Serial.print(F("valid "));
    Serial.print(sentences_valid);
    Serial.print(F(" / invalid "));
    Serial.print(sentences_invalid);
    Serial.print(F(" / bytes r "));
    Serial.print(bytes_read);
    Serial.print(F(" / bytes w"));
    Serial.println(bytes_written);
    if (hasSDCard) {
      // has to be called periodically, otherwise data is not accessible on SD card
      file.sync();  
    }
    timer = millis();
  }
}

