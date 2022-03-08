
/***************************************************
 * 
 *                 IoT Spirometer
 * 
 *                       by
 *                  Marc van Zyl
 * 
 * *************************************************/

// Global config defines
#define IOT_ACTIVE 0


#include <Arduino.h>
#include <ctype.h>
 
#include "SDP3x.h"
#include "U8g2lib.h"   // LCD Screen includes
#include "U8x8lib.h"  // LCD Screen font includes



// LCD Screen
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);


// THese are the states
enum states {
  STATE_IDLE,
  STATE_W_INHALE_B,
  STATE_R_INHALE_B,
  STATE_W_EXHALE,
  STATE_R_EXHALE,
  STATE_W_INHALE,
  STATE_R_INHALE,
  STATE_PRINT,
  STATE_IOT_UPLOAD,
  STATE_ERROR
};

float simpleVol;
float simpleVelocity;
float lastSimpleVelocity;
float maxSimpleVol;

#define SIMPLE_VOL_THRESHOLD 0.05  // This is the trial accuracy threshold 

int state;

// constants for the pressures thresholds between states
// if the pressure goes above or below these trigger the state transition
#define EXHALE_PRESSURE_THRESHOLD 40
#define INHALE_PRESSURE_THRESHOLD 40
#define INHALE_PRESSURE_THRESHOLD_B 50

// Constants to decide on timeouts
#define MAX_EXHALE_TIME 5000  // 5 seconds
#define MAX_INHALE_TIME 5000  // 5 seconds

// Other constants
#define DIR_EXHALE 1.0
#define DIR_INHALE -1.0
#define D1 0.025  // Diameter of the venturi tubes
#define D2 0.015  // Diameter of the venturi throat
#define A1 3.14159*D1*D1/4.  // area of the venturi tubes
#define A2 3.14159*D2*D2/4.  // area of the venturi throat
#define AIR_DENSITY 1.225  // density at sea level at 20 deg C 1.225kg/m3
#define M3_TO_LITRES 1000.0  // cubic meters to litres conversion
#define A2D_UNITS_TO_PRESSURE 1/20.0 // from the pressure sensor datasheet

// arrays to store 5 seconds of the pressure for inhale and EXHALE_PRESSURE_THRESHOLD
int16_t exhalePressure[500];
int16_t inhalePressure[500];

using namespace SDP3X;

SDP3x sensor(Address1, MassFlow); //create the pressure sensor

#define PERIOD 10 // this is the period in ms of the pressure sensor readings
#define DT PERIOD/1000.0 // This is the time of each loop in seconds

unsigned long next_tick;
unsigned long start_time;

// This draws the cool flying triangles
void u8g2_triangle(uint8_t a){
  uint16_t offset = a;
  u8g2.drawStr( 0, 0, "drawTriangle");
  u8g2.drawTriangle(14,7, 45,30, 10,40);
  u8g2.drawTriangle(14+offset,7-offset, 45+offset,30-offset, 57+offset,10-offset); 
  u8g2.drawTriangle(57+offset*2,10, 45+offset*2,30, 86+offset*2,53); 
  u8g2.drawTriangle(10+offset,40+offset, 45+offset,30+offset, 86+offset,53+offset);

}

// This is from the u8g2 Graphics Test program - prepares the screen
void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

// function to clear the screen and prepare it for printing
void LCD_Clear(void){
  u8g2.clearBuffer();
  u8g2_prepare();
}

// function to print a string to the screen
void LCD_Print(int x, int y, const char *str1){
  u8g2.drawStr(x, y, str1);
  u8g2.sendBuffer();
}

// Function to print a floating point number
void LCD_Print_Float(int x, int y, float f){
  String str = String(f);
  char chars[str.length()];
  str.toCharArray(chars, str.length());
  u8g2.drawStr(x, y, chars);
  u8g2.sendBuffer();
}

// function to play a tone for .2 seconds

#define TONE_PIN 13 

void play_tone(int duration){
  digitalWrite(TONE_PIN, HIGH);
  delay(duration);
  digitalWrite(TONE_PIN, LOW);
}

/*****************************************************************/
/*************************      SETUP      ***********************/
/*****************************************************************/

void setup() {
  // this setup code runs once only
  
  // start the screen
  u8g2.begin();
  u8g2.setFlipMode(true);

  // Start the serial connection for debugging.
  Serial.begin(115200);
  Serial.flush();

  // Start the communication with the pressure sensor
  Wire.begin();
  while(!sensor.begin()){
    Serial.print("Board not initialized!");
    delay(1000);
  }

  // Start the sensor in "continuous mode" with averaging enabled
  // this means the sensor gets a new reading every 1ms but I only
  // read it every 10ms.  It averages the readings in between.
  sensor.startContinuous(true);

  // This prints the start screen 
  LCD_Clear();
  LCD_Print(0,0,"An IoT Spirometer");
  delay(1000);
  LCD_Clear();
  LCD_Print(0,0,"by Marc van Zyl");
  delay(1000);
  for (int i=0; i<8; i++){
    u8g2_prepare();
    u8g2_triangle(i);
    delay(100);
  }

  // Setup the buzzer
  pinMode(TONE_PIN, OUTPUT);
  digitalWrite(TONE_PIN, LOW);
  play_tone(1000);
  
  // Setup the button
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);  // this activates the internal pull-up resistor

  // Print the instructions screen
  LCD_Clear();
  LCD_Print(0,0, "Ready to read");
  LCD_Print(0,10, "Bring the");
  LCD_Print(0,20, "Spirometer to");
  LCD_Print(0,30, "your lips and");
  LCD_Print(0, 40, "Press the button");
  LCD_Print(0, 50, "and inhale");
  delay(1000);

  // This sets the timer
  next_tick = millis() + PERIOD;
  start_time = next_tick;
}


// Global variables
bool status;
float f_pressure;
float cum_vol;
float flow_rate;
float last_flow_rate;
float direction;
int16_t pressure;

unsigned long readFlowStartTime;
unsigned int exhaleCount;
unsigned int inhaleCount;

// Function to read the pressure sensor and to do the calculations

void readSensor(void){
  status = sensor.readMeasurement(&pressure, NULL, NULL);
  f_pressure = -1.0*pressure*A2D_UNITS_TO_PRESSURE;
  pressure *= -1;
  float velocity = direction*sqrt(-2.0*f_pressure/(AIR_DENSITY*(pow((A2/A1),2.0) - 1.0)));
  flow_rate = velocity*A2*M3_TO_LITRES;
  cum_vol += DT*((flow_rate + last_flow_rate)/2.0);
  last_flow_rate = flow_rate;
}

/*****************************************************************/
/*************************      LOOP       ***********************/
/*****************************************************************/

// The main loop that runs continuously
void loop() {

  // wait for the next tick to start the cycle
  while (millis() < next_tick){
    delay(1);
  }

  next_tick += PERIOD;  // set the next tick time

  readSensor();  // read the pressure sensor

  switch(state){
    case STATE_IDLE:
      // *** state execution commands

      // *** state transition - check for button press
      if (digitalRead(2) == LOW){  // Check for button press
        // next state initialization
        LCD_Clear();
        LCD_Print(0,0,"Waiting for inhale");
        Serial.println("Starting data acqusition");
        Serial.println("Waiting for initial inhale");
        readFlowStartTime = millis();
        // initialize the variables for volumne Check
        simpleVol = 0.0;
        simpleVelocity = 0.0;
        lastSimpleVelocity = 0.0;
        maxSimpleVol = 0.0;
        play_tone(300);  // play a short tone 
        state = STATE_W_INHALE_B;
      }
      break;
    case STATE_W_INHALE_B:
      // *** state execution commands

      // *** state transition - wait for the inhale to begin
      if (pressure > INHALE_PRESSURE_THRESHOLD_B){ // advance the state if true
        readFlowStartTime = millis();
        LCD_Clear();
        LCD_Print(0,0, "Initial inhaling");
        Serial.println("Initial inhaling");
        state = STATE_R_INHALE_B; // advance the state to read inhale B
      }
      // *** state transition - check for timeout after MAX_INHALE_TIME
      if (millis() > readFlowStartTime + MAX_INHALE_TIME){
        LCD_Clear();
        LCD_Print(0,0, "Timed out!");
        Serial.println("Timed out on state STATE_W_INHALE_B");
        delay(1000);
        state = STATE_ERROR;  // go to the error state
      }
      break;

    case STATE_R_INHALE_B:
      // *** state execution commands
      
      // *** state transition - check for end of inhale
      if  (pressure < INHALE_PRESSURE_THRESHOLD-5){ // check if the inhale has ended
        LCD_Clear();
        LCD_Print(0,0,"Waiting for exhale");
        Serial.println("Found end of initial inhale....waiting for exhale");
        readFlowStartTime = millis();
        play_tone(300);
        state = STATE_W_EXHALE;
      }
      if (millis() > readFlowStartTime + 5000){
        LCD_Clear();
        LCD_Print(0,0,"Timed out");
        Serial.println("Timed out on state STATE_R_INHALE_B");
        delay(1000);
        state = STATE_ERROR;
      }
      break;
    case STATE_W_EXHALE:
      // *** state execution commands

      // *** state transition -  check for begin of exhale
      if (pressure > EXHALE_PRESSURE_THRESHOLD){ // check if the exhale has started
        LCD_Clear();
        LCD_Print(0,0,"Found Exhale");
        Serial.println("Found exhale");
        exhaleCount = 0;
        readFlowStartTime = millis();
        lastSimpleVelocity = 0;
        state = STATE_R_EXHALE;
      }
      // *** state transition - timeout
      if (millis() > readFlowStartTime + 5000){
        LCD_Clear();
        LCD_Print(0,0,"Timed out");
        Serial.println("Timed out on state STATE_W_EXHALE");
        state = STATE_ERROR;
      }
      break;
    case STATE_R_EXHALE:
      // *** state execution commands
      exhalePressure[exhaleCount] = pressure; // save the pressure in the array
      
      if (exhaleCount < 500-1)  // this protects against writing more than the allocated memory
        exhaleCount++;
      simpleVelocity = sqrt( (float)abs(pressure));
      simpleVol += (lastSimpleVelocity + simpleVelocity)/2;
      lastSimpleVelocity = simpleVelocity;

      // *** state transition - check for end of exhale
      if (pressure < EXHALE_PRESSURE_THRESHOLD-5){  // this means the exhale is over
        LCD_Clear();
        LCD_Print(0,0,"Waiting for inhale");
        Serial.println("Exhale done....Waiting for inhale");
        maxSimpleVol = simpleVol;
        readFlowStartTime = millis();
        play_tone(300);
        state = STATE_W_INHALE;
      }
      // *** state transition - timeout
      if (millis() > readFlowStartTime + 5000){
        LCD_Clear();
        LCD_Print(0,0,"Timed out");
        Serial.println("Timed out on state STATE_R_EXHALE");
        delay(1000);
        state = STATE_ERROR;
      }
      break;

    case STATE_W_INHALE:
      // *** state execution commands

      // *** state transition - check for start of inhale
      if (pressure > INHALE_PRESSURE_THRESHOLD){  // the inhale has started
        inhaleCount = 0;
        readFlowStartTime = millis();
        lastSimpleVelocity = 0;
        LCD_Clear();
        LCD_Print(0,0,"Found inhale");
        Serial.println("Found inhale");
        state = STATE_R_INHALE;
      }
      // *** state transition - timeout
      if (millis() > readFlowStartTime + 5000){
        LCD_Clear();
        LCD_Print(0,0,"Timed out");
        Serial.println("Timed out on state STATE_W_INHALE");
        delay(1000);
        state = STATE_ERROR;
      }
      break;
    case STATE_R_INHALE:
      // *** state execution commands
      inhalePressure[inhaleCount] = pressure; // save the pressure in the array

      if (inhaleCount < 500-1)
        inhaleCount++;
      simpleVelocity = sqrt( (float)abs(pressure));
      simpleVol -= (lastSimpleVelocity + simpleVelocity)/2;
      lastSimpleVelocity = simpleVelocity;

      // *** state transition - check for end of inhale
      if (pressure < INHALE_PRESSURE_THRESHOLD-5){ // check for end of inhale
        play_tone(300);  // play three tones to signal end of cycle
        delay(300);
        play_tone(300);
        delay(300);
        play_tone(300);
        delay(300);
        state = STATE_PRINT;
      }

      // *** state transition - timeout
      if (millis() > readFlowStartTime + 5000){
        LCD_Clear();
        LCD_Print(0,0,"Timed out");
        Serial.println("Timed out on state STATE_R_INHALE");
        delay(1000);
        state = STATE_ERROR;
      }
      break;

    case STATE_PRINT:  
      // *** state execution commands
      // first print the results to serial 
      Serial.println("Starting Print");
      for (unsigned int i=0; i<exhaleCount; i++)  // print the exhale data
        Serial.println(exhalePressure[i], DEC);
      for (unsigned int i=0; i<inhaleCount; i++)  // print the inhale data
        Serial.println(inhalePressure[i], DEC);

      // Check if the exhale - inhale cycle was successful  
      if ( abs(simpleVol/maxSimpleVol) > SIMPLE_VOL_THRESHOLD ){
        LCD_Clear();
        LCD_Print(0,0,"Failed blow");
        LCD_Print_Float(0,10, simpleVol/maxSimpleVol);
        delay(3000);
      }
      // print results to the LCD
      LCD_Clear();
      LCD_Print(0,0,"Max Vol: ");
      LCD_Print_Float(45,0, maxSimpleVol);
      // print results to the serial port
      Serial.print("maxSimpleVol: ");
      Serial.println(maxSimpleVol);
      Serial.print("simpleVol: ");
      Serial.println(simpleVol);
      Serial.print("Percent error:");
      Serial.println(simpleVol/maxSimpleVol);

      // *** state transition - to STATE_IOT_UPLOAD
      state = STATE_IOT_UPLOAD;
      break;

    case STATE_ERROR:
      // *** state execution commands
      LCD_Clear();
      LCD_Print(0,0,"Return to Idle");
      Serial.println("In Error state .... Returning to idle");

      // *** state transition - to STATE_IOT_UPLOAD
      state = STATE_IOT_UPLOAD;
      break;
    
    case STATE_IOT_UPLOAD:
      // *** state execution commands
      LCD_Clear();
      LCD_Print(0,0,"Uploading to cloud");
      Serial.println("In STATE_IOT_UPLOAD state .... ");

      if (IOT_ACTIVE){
        // insert the IoT upload here

      } else {
        LCD_Print(0,10,"IoT Upload-False");
        Serial.println("Upload to cloud bypassed IOT_ACTIVE is false ");
        Serial.println("Returning to STATE_IDLE");

        delay(2000);
        
      }
      // *** state transition - to STATE_IDLE
      state = STATE_IDLE;
      break;

    default:
      // *** state execution commands
      LCD_Clear();
      LCD_Print(0,0,"Error default state");
      Serial.println("Error in default state - return to state STATE_IDLE");
      
      // *** state transition - to STATE_IDLE
      state = STATE_IDLE;
      break; 
  }
}
