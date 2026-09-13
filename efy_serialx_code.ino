/*
  ESP32 REAL GPS + DHT11 + Temperature Signal Generator
  ------------------------------------------------------

  GPS:
    - Real NEO-6M GPS module
    - UART2
    - ESP32 RX2 = GPIO16
    - ESP32 TX2 = GPIO17
    - GPS data is forwarded RAW
    - GPS output starts ONLY after "SEND_GPS" is received
    - No GPS parsing
    - No filtering
    - No modification of NMEA sentences

  DHT11:
    - DATA = GPIO25
    - TEMP3 = actual DHT11 temperature

  Temperature / Signal Outputs:
    TEMP0: Sine wave generator
    TEMP1: Square wave generator
    TEMP2: Triangle wave generator
    TEMP3: Real DHT11 temperature

  Signal frequency:
    5 Hz
    One update every 200 ms

  Serial Monitor baud:
    9600

  GPS baud:
    9600
*/


#include <Arduino.h>
#include <DHT.h>
#include <math.h>


/* =========================================================
   PIN DEFINITIONS
   ========================================================= */

// NEO-6M connected to ESP32 UART2
#define GPS_RX_PIN 16       // ESP32 RX2  <- GPS TX
#define GPS_TX_PIN 17       // ESP32 TX2  -> GPS RX

// DHT11
#define DHT_PIN 25
#define DHT_TYPE DHT11


/* =========================================================
   SERIAL DEFINITIONS
   ========================================================= */

// USB serial monitor
#define PC_BAUD 9600

// NEO-6M default baud rate
#define GPS_BAUD 9600

HardwareSerial GPS_Serial(2);


/* =========================================================
   DHT11
   ========================================================= */

DHT dht(DHT_PIN, DHT_TYPE);

// Cached DHT temperature
// DHT11 cannot realistically be read at 5 Hz,
// so we read it periodically and retain the latest
// valid value for TEMP3.
float dhtTemperature = 0.0;

const unsigned long DHT_READ_INTERVAL_MS = 2000;
unsigned long lastDHTReadTime = 0;


/* =========================================================
   SERIAL COMMAND
   ========================================================= */

// Fixed-size buffer instead of dynamic String
// for a more embedded-friendly implementation.
char inputBuffer[32];
uint8_t inputIndex = 0;


/* =========================================================
   GPS COMMAND-TRIGGERED RAW OUTPUT
   ========================================================= */

// When SEND_GPS is received, the ESP32 forwards
// whatever raw bytes arrive from the NEO-6M
// for this amount of time.
//
// 1500 ms gives enough time to receive a useful
// NMEA burst from a normal 1 Hz NEO-6M output.
const unsigned long GPS_CAPTURE_TIME_MS = 1500;

bool gpsCaptureActive = false;
unsigned long gpsCaptureStartTime = 0;


/* =========================================================
   5 Hz SIGNAL GENERATOR
   ========================================================= */

const unsigned long SIGNAL_INTERVAL_MS = 200;

unsigned long lastSignalTime = 0;

float phase = 0.0;

// 5 Hz:
// 1 complete cycle = 200 ms
//
// 200 ms / 10 samples per cycle = 20 degree step
//
// BUT your original dummy code uses:
// 36 degrees per sample.
//
// This is preserved exactly from the original.
const float PHASE_INCREMENT = 36.0;


/* =========================================================
   READ DHT11
   ========================================================= */

void readDHT11()
{
  float temp = dht.readTemperature();

  // DHT libraries return NaN when the read fails.
  if (!isnan(temp))
  {
    dhtTemperature = temp;
  }

  lastDHTReadTime = millis();
}


/* =========================================================
   START GPS RAW CAPTURE
   ========================================================= */

void startGPSCapture()
{
  /*
    Clear any partially received/stale bytes from UART2.

    This prevents SEND_GPS from immediately dumping an
    incomplete NMEA sentence that happened to already
    be sitting in the UART receive buffer.
  */
  while (GPS_Serial.available())
  {
    GPS_Serial.read();
  }

  gpsCaptureStartTime = millis();
  gpsCaptureActive = true;
}


/* =========================================================
   PROCESS RAW GPS DATA
   ========================================================= */

void processGPS()
{
  if (!gpsCaptureActive)
  {
    return;
  }

  /*
    Forward every byte exactly as received.

    There is:
      - no NMEA parsing
      - no filtering
      - no checksum processing
      - no sentence modification
      - no GPS library

    The NEO-6M data is simply passed through.
  */

  while (GPS_Serial.available())
  {
    char c = GPS_Serial.read();

    Serial.write(c);
  }


  /*
    Stop forwarding after the capture interval.
  */

  if (millis() - gpsCaptureStartTime >= GPS_CAPTURE_TIME_MS)
  {
    gpsCaptureActive = false;

    Serial.println();
    Serial.println("----- GPS CAPTURE END -----");
  }
}


/* =========================================================
   GENERATE TEMPERATURE SIGNALS
   ========================================================= */

void generateSignals()
{
  /*
    phase:
      0   -> 0 degrees
      180 -> half cycle
      360 -> complete cycle

    Original waveform generation retained.
  */

  float radians =
    phase * PI / 180.0;


  /* -------------------------------------------------------
     TEMP0 - SINE WAVE
     Range: 0 to 100
     ------------------------------------------------------- */

  float sine =
    50.0 + 50.0 * sin(radians);


  /* -------------------------------------------------------
     TEMP1 - SQUARE WAVE
     Range: 0 or 100
     ------------------------------------------------------- */

  float square;

  if (phase < 180.0)
  {
    square = 100.0;
  }
  else
  {
    square = 0.0;
  }


  /* -------------------------------------------------------
     TEMP2 - TRIANGLE WAVE
     Range: 0 to 100
     ------------------------------------------------------- */

  float triangle;

  if (phase < 180.0)
  {
    triangle =
      (100.0 / 180.0) * phase;
  }
  else
  {
    triangle =
      100.0 -
      ((100.0 / 180.0) *
       (phase - 180.0));
  }


  /* -------------------------------------------------------
     TEMP3 - REAL DHT11 TEMPERATURE
     ------------------------------------------------------- */

  /*
    TEMP3 is now the actual measured temperature
    from the DHT11 connected to GPIO25.

    No artificial waveform is applied.
  */


  /* =======================================================
     SERIAL OUTPUT
     ======================================================= */

  Serial.printf(
    "TEMP0: %.2f\n",
    sine
  );

  Serial.printf(
    "TEMP1: %.2f\n",
    square
  );

  Serial.printf(
    "TEMP2: %.2f\n",
    triangle
  );

  Serial.printf(
    "TEMP3: %.2f\n",
    dhtTemperature
  );


  /* =======================================================
     ADVANCE PHASE
     ======================================================= */

  phase += PHASE_INCREMENT;

  if (phase >= 360.0)
  {
    phase -= 360.0;
  }
}


/* =========================================================
   PROCESS SERIAL COMMANDS
   ========================================================= */

void processSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();


    /*
      Ignore carriage return.
      Newline terminates the command.
    */

    if (c == '\r')
    {
      continue;
    }


    if (c == '\n')
    {
      inputBuffer[inputIndex] = '\0';

      if (inputIndex > 0)
      {
        /*
          Remove leading spaces.
        */

        char *command = inputBuffer;

        while (*command == ' ')
        {
          command++;
        }


        /*
          SEND_GPS command
        */

        if (strcasecmp(command, "SEND_GPS") == 0)
        {
          Serial.println();
          Serial.println("----- GPS CAPTURE START -----");

          startGPSCapture();
        }
        else
        {
          Serial.print("Unknown command: ");
          Serial.println(command);
        }
      }

      /*
        Reset command buffer.
      */

      inputIndex = 0;
      inputBuffer[0] = '\0';
    }
    else
    {
      /*
        Store character only if buffer has space.
      */

      if (inputIndex < sizeof(inputBuffer) - 1)
      {
        inputBuffer[inputIndex++] = c;
      }
      else
      {
        /*
          Prevent buffer overflow.
        */

        inputIndex = 0;
        inputBuffer[0] = '\0';

        Serial.println("Command too long");
      }
    }
  }
}


/* =========================================================
   SETUP
   ========================================================= */

void setup()
{
  /*
    USB Serial Monitor
  */

  Serial.begin(PC_BAUD);


  /*
    Start UART2 for NEO-6M

    ESP32:
      RX = GPIO16
      TX = GPIO17
  */

  GPS_Serial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );


  /*
    Start DHT11
  */

  dht.begin();


  /*
    Allow DHT11 to stabilize.
  */

  delay(2000);


  /*
    Initial DHT11 temperature read.
  */

  readDHT11();


  /*
    Startup information
  */

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 REAL GPS + DHT11 SYSTEM");
  Serial.println("========================================");

  Serial.println("GPS:");
  Serial.println("  Module : NEO-6M");
  Serial.println("  UART   : UART2");
  Serial.println("  RX2    : GPIO16");
  Serial.println("  TX2    : GPIO17");
  Serial.println("  Baud   : 9600");
  Serial.println("  Mode   : RAW NMEA");

  Serial.println();

  Serial.println("DHT11:");
  Serial.println("  DATA   : GPIO25");

  Serial.println();

  Serial.println("Temperature Signals:");
  Serial.println("  TEMP0  : Sine");
  Serial.println("  TEMP1  : Square");
  Serial.println("  TEMP2  : Triangle");
  Serial.println("  TEMP3  : DHT11 temperature");

  Serial.println();

  Serial.println("GPS command:");
  Serial.println("  SEND_GPS");

  Serial.println();

  Serial.println("Signal frequency: 5 Hz");
  Serial.println("========================================");
  Serial.println();
}


/* =========================================================
   LOOP
   ========================================================= */

void loop()
{
  unsigned long currentTime = millis();


  /* =======================================================
     PROCESS USB SERIAL COMMANDS
     ======================================================= */

  processSerial();


  /* =======================================================
     PROCESS GPS RAW DATA
     ======================================================= */

  processGPS();


  /* =======================================================
     DHT11 UPDATE
     ======================================================= */

  /*
    DHT11 is read every 2 seconds.

    TEMP3 continues using the latest valid reading
    between actual DHT11 measurements.
  */

  if (currentTime - lastDHTReadTime >= DHT_READ_INTERVAL_MS)
  {
    readDHT11();
  }


  /* =======================================================
     GENERATE TEMP0-TEMP3 AT 5 Hz
     ======================================================= */

  if (currentTime - lastSignalTime >= SIGNAL_INTERVAL_MS)
  {
    lastSignalTime = currentTime;

    generateSignals();
  }
}