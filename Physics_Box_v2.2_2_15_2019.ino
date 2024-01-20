 
/*****************************************************************/
/*                       Physics Box Program v2.1
 *                        
 *    Past Contributors:
 *    - Kyle Diekhoff (May 2016)
 *    - Mahesh Gorantla (May 2017)
 *    - Rodolfo Leiva (Summer 2017)
 *    
 *               This program last updated on 9/26/2017
 *               Modifications on v2.1 by: Rodolfo Leiva
 *               Contact Email: rodolfoed94@gmail.com             
 *****************************************************************/

 
/* --------------------------------------------------------------
  |.........................INCLUDE LIBRARIES................... |
   --------------------------------------------------------------  */
#include <EEPROM.h>
#include <SD.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_ADXL345_U.h>
#include "RTClib.h"

/* Peripheral Devices */
// This is used to fetch the RealTime Clock
RTC_PCF8523 rtc;

// Assigining a Unique ID to the Accelerometer Device and initializing it.
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


/* --------------------------------------------------------------
  |.............DEFINE CONSTANTS AND GLOBAL VARIABLES........... |
   --------------------------------------------------------------  */

#define LOG_INT 200 // Period of time (milliseconds) for each sample to be logged in the SD card (Energy is exeption)
#define ELOG_INT 2000 // Period of time (milliseconds) for each Energy sample to be logged in the SD card
#define E_INT 10000 // Period of time (milliseconds) for energy to update in the eeprom
#define LCD_INT 1000 //Period of time for LCD display to update
#define RPM_INT 1000 // Period of time (milliseconds) over which RPM (Revolutions per Minute) is calculated 
#define MAGNETS 4 // number of magnets attached to axel for hall effect sensor to sense

#define CurrIn A0 // Current Sense input pin
#define VoltIn A1 // Voltage Sense input pin
#define AnalogA A2 // Optional Analog input A pin
#define AnalogB A3 // Optional Analog input B pin

#define ClearMemory 2 // Button to clear EEPROM memory input pin
#define Hall_Effect 3 // Hall Effect Sensor input pin
#define Digital 4 // Optional Digital input pin
#define ChipSelect 10 // SD comunications output pin

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// The Logging File
File logFile;
char filename[] = "LOGGER55.csv";

int a = 0; // address where eeprom memory index is stored
unsigned long eeprom_c = 0; //eeprom writes counter
int data_i = 0; //address where Energy memory is stored

float Energy = 0; // Collective energy added up since restart
float newEnergy = 0; //new energy added to the total energy

float revolutions = 0; // This is used to keep track of the no. of revolutions made
float rpm = 0; // This is used to store the RPM (Revolutions Per Minute) of the Kart
float rpm_factor = 0; // This is used to multiply the impulses computed for every 'x' ms as specified by RPM_INT

unsigned long millisValPrev = 0; // variable that captures the miliseconds that have passed since the program started


void setup() {

/* --------------------------------------------------------------
  |.............BEGIN SERIAL OUTPUT AT 9600 BAUD................ |
   --------------------------------------------------------------  */
  // This is default RS 232 Baud Rate (Used for LCD Display).
  // Changing this Value from 9600 to some other value will make LCD Work Abnormally.
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

/* --------------------------------------------------------------
  |......................SET UP LCD DISPLAY..................... |
   --------------------------------------------------------------  */
  setBacklightBrightness(2); //set brightness level from 1 to 8: 1 being backlight off, 8 being brightest.
  loadCharacters();  //loads characters for x_x symbol
  clearLCD(); // Clear the Contents of LCD.
  displayOn(); // LCD Display On
  cursorHome(); // Set the LCD

/* --------------------------------------------------------------
  |...................Initializing program message.............. |
   --------------------------------------------------------------  */
  clearLCD();
  cursorSet(0,0);
  Serial.print("Initializing");
  cursorSet(1,0);
  Serial.print("Program");
  delay(1000);

/* --------------------------------------------------------------
  |.............DECLARE AND SET UP I/O PIN MODES................ |
   --------------------------------------------------------------  */
  // Setting the pin Configurations
  pinMode(VoltIn, INPUT);   // Analog Pin 0
  pinMode(CurrIn, INPUT); // Analog Pin 1
  pinMode(Hall_Effect, INPUT); // Configuring the Hall Effect Sensor Pin to the INPUT Mode
  pinMode(ClearMemory, INPUT); // Configuring the Clear the EEPROM Button
  pinMode(Digital, INPUT_PULLUP); // Configuring Digital Pin 4 as an input
  
  // These pins will read 5V when there is no input from external interface or the no inputs are connected
  pinMode(AnalogA, INPUT); // Activating the Pull-Up Resistors for Analog Pin 2
  pinMode(AnalogB, INPUT); // Activating the Pull-Up Resistors for Analog Pin 3

  // Make sure that default chip select pin is set to output
  // even if you don't use it:
  pinMode(ChipSelect, OUTPUT);

  // Setting initial Pin Values
  digitalWrite(Hall_Effect, HIGH); // Switching on the Internal Pull-Up Resistors
  digitalWrite(ClearMemory, HIGH); // Switching on the Internal Pull-Up Resistors

/* --------------------------------------------------------------
  |..............SET UP EEPROM RESET INTERRUPT.................. |
   --------------------------------------------------------------  */  
  // Setting the EEPROM Clear Button
  attachInterrupt(digitalPinToInterrupt(ClearMemory), reset_eeprom, FALLING);

/* --------------------------------------------------------------
  |...........SET UP HALL EFFECT SENSOR INTERRUPT............... |
   --------------------------------------------------------------  */  
  // Setting the Clear for Power Outage LED
  attachInterrupt(digitalPinToInterrupt(Hall_Effect), count_revolutions, FALLING);

/* --------------------------------------------------------------
  |................CHECK IF RTC HAS STARTED..................... |
   --------------------------------------------------------------  */  
  // Beginning the RTC. If this step is not performed, then RTC will read out very abnormal Values.
  if (! rtc.begin()) {
    clearLCD();
    cursorSet(0,0);
    Serial.print("Couldn't find RTC");
    //while (1);
  }

/* --------------------------------------------------------------
  |..............CHECK IF RTC HAS INITIALIZED................... |
   --------------------------------------------------------------  */
  // The following line sets the RTC to date & time this sketch was compiled.
  if (!rtc.initialized()) {
    clearLCD();
    cursorSet(0,0); 
    Serial.print("Adjusting RTC");

    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    
    //while (1);
  }

/* --------------------------------------------------------------
  |...........CHECK IF ACCELEROMETER IS INITIALIZED............. |
   --------------------------------------------------------------  */   
  if(!accel.begin()) {
    clearLCD();
    cursorSet(0,0);    
    Serial.print("Accel Failure");
    delay(2000);
    //while(1); //
  }

/* --------------------------------------------------------------
  |.................SET ACCELEROMETER'S RANGE................... |
   --------------------------------------------------------------  */
  // Setting the range of g's the accelerometer can measure
  accel.setRange(ADXL345_RANGE_2_G);

/* --------------------------------------------------------------
  |.....................DEFINE RPM FACTOR....................... |
   --------------------------------------------------------------  */
  // This is used to compute RPM of the vehicle from the count of impulses
  rpm_factor = ((1000. * 60.) / RPM_INT);


/* --------------------------------------------------------------
  |..........INITIALIZE EEPROM IF NOT YET INITIALIZED........... |
   --------------------------------------------------------------  */
  unsigned long eeprom_c_next = 0; //eeprom_c comparison variable
  unsigned long eeprom_c2 = 0;
  float Energy2 = 0;
  
  EEPROM.get(a,eeprom_c); //get eeprom_c (eeprom write counter) starting at a = 0;
  EEPROM.get(a+256, eeprom_c2); //get newly initialized eeprom_c2
  EEPROM.get(a+512, Energy); //get newly initialized Energy
  EEPROM.get(a+768, Energy2); //get newly initialized Energy2

  //When an arduino is new and hasn't written to its eeprom yet, the preset unsigned long value for each address in the eeprom is 4294967295.
  //This following if statement checks if the first memory address reads a preset value meaning the eeprom has not yet been initialized. 
  //This if statement was added because of some issues experienced when starting up the program caused by preset values in the eeprom. 
  if((eeprom_c == 4294967295) || (eeprom_c2 == 4294967295) || (Energy != Energy) || (Energy2 != Energy2)){
      EEPROM.put(0, uint32_t(0)); //writes a 0 value for the first eeprom_c (eeprom write counter) value on the first memory address
      EEPROM.put(256, uint32_t(0)); //writes a 0 value for the first eeprom_c2 (eeprom_c backup) value
      EEPROM.put(512, 0.00); //writes a 0.00 value for the first Energy value
      EEPROM.put(768, 0.00); //writes a 0.00 value for the first Energy2 value (backup energy)
     
      EEPROM.get(a, eeprom_c); //get newly initialized eeprom_c starting at a = 0;
      EEPROM.get(a+256, eeprom_c2); //get newly initialized eeprom_c2
      EEPROM.get(a+512, Energy); //get newly initialized Energy
      EEPROM.get(a+768, Energy2); //get newly initialized Energy2
   } 

/* --------------------------------------------------------------
  |...get eeprom write counter (eeprom_c) and its address 'a' .. |
   --------------------------------------------------------------  */
  EEPROM.get((a+4) % 256, eeprom_c_next); //get eeprom_c comparison from next eeprom_c value at a = 4; (each address holds 4 bytes)   
  while(eeprom_c == eeprom_c_next - 1){  //while eeprom_c is 1 less than the next eeprom_c value is 
    a = (a + 4) % 256; //update 'a' to next address (loop around to 0 if at a = 252) 
    EEPROM.get(a,eeprom_c); //get new value for eeprom_c and eeprom_c_next and compare again till the next value is not 1 larger than the current meaning we have gotten to the last saved counter value or to the corrupted value.
    EEPROM.get((a+4) % 256, eeprom_c_next);
  }

/* --------------------------------------------------------------
  |....................CHECK IF OVER WARRANTY................... |
   --------------------------------------------------------------  */
  //64 available 4-byte memory cells available for data and 100,000 guaranteed writes for each cell means 6,400,000 guaranteed writes before writing becomes a risk for corruption
  if(eeprom_c >= 6400000){ //if eeprom_c is ever greater than 6400000 then the arduino has expired its guaranteed number of trustworthy writes
      clearLCD();
      cursorSet(0,0);
      Serial.print("(");
      Serial.write(0x00);
      Serial.write(0x01);
      Serial.write(0x03);
      Serial.print(")");   
      Serial.print("  EEPROM");
      cursorSet(1,0);
      Serial.print("OVER WARRANTY");
      delay(2000);
    }  //The message in this if statement reads as "x_x EEPROM OVER WARRANTY"
  
  data_i = (eeprom_c % 64) * 4 + 512; //determine data index from eeprom counter

/* --------------------------------------------------------------
  |.............CHECK EEPROM FOR DATA CORRUPTION................ |
   --------------------------------------------------------------  */
   boolean CounterCorruption = CheckSetupCounterCorruption(); //check if there has been corruption in the eeprom writes counter and correct it
   boolean DataCorruption = CheckDataCorruption(); //check if there has been corruption in the data stored in eeprom and correct it
   if(CounterCorruption || DataCorruption){ //if there has been any corruption in eeprom, send warning
    clearLCD();
    cursorSet(0,0);
    Serial.print("WARNING:    DATA");
    cursorSet(1,0);
    Serial.print("CORRUPTION FOUND");
    delay(2000);
  }

/* --------------------------------------------------------------
  |.................READ ENERGY FROM EEPROM..................... |
   --------------------------------------------------------------  */    
  EEPROM.get(data_i, Energy); //get energy from eeprom (though this has already been done when checking corruption.

/* --------------------------------------------------------------
  |........CHECK IF SD CARD IS PRESENT AND INITIALIZED.......... |
   --------------------------------------------------------------  */
  // Checking if the SD Card is present and can be initialized:
  if (!SD.begin(ChipSelect)) {
    clearLCD();
    cursorSet(0,0);
    Serial.print("Card failed, or");
    cursorSet(1,0);
    Serial.print("not present");
    delay(2000);
    return;
  }

  //Messaging that the SD card works and has initialized
  clearLCD();
  cursorSet(0,0);
  Serial.print("Card Initialized");
  delay(2000);
  clearLCD();

  //messaging name of file the data is being logged to in sd card
  cursorSet(0,0);
  Serial.print("Logging to: ");
  cursorSet(1,0);
  Serial.print(filename);
  delay(2000);
  clearLCD();

/* --------------------------------------------------------------
  |..START UP '.CSV' FILE WITH TIME STAMP, AND COLUMN HEADERS... |
   --------------------------------------------------------------  */
  logFile = SD.open(filename, FILE_WRITE);

  // This below piece of code inserts the Time Stamp for new Logging every time when Power is RESET.
  DateTime now = rtc.now();
  logFile.println("");
  logFile.print(now.month(), DEC);
  logFile.print("/");
  logFile.print(now.day(), DEC);
  logFile.print("/");
  logFile.print(now.year(), DEC);
  logFile.print(" : ");
  logFile.print("(");
  logFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
  logFile.print(")");
  logFile.print(now.hour(), DEC);
  logFile.print(':');
  logFile.print(now.minute(), DEC);
  logFile.print(':');
  logFile.print(now.second(), DEC);
  logFile.println();

  logFile.println("millis,Volt,Current,Power(kW),Energy(kJ),Analog Input 2,Analog Input 3,RPM,X-Accel(m/s^2),Y-Accel(m/s^2),Z-Accel(m/s^2),DIGIN_4");
  logFile.close();

}

void loop() {
/* --------------------------------------------------------------
  |.......................CAPTURE TIME.......................... |
   --------------------------------------------------------------  */  
  unsigned long millisVal = millis();

/* --------------------------------------------------------------
  |.....................CALCULATE RPM........................... |
   --------------------------------------------------------------  */
  // Computing the RPM for every second
  if(millisVal/RPM_INT != millisValPrev/RPM_INT)
  {
    rpm = (revolutions * rpm_factor);
    revolutions = 0;
  }

/* --------------------------------------------------------------
  |......COMPLETE FOLLOWING SERIES OF TASKS EVERY 200ms......... |
   --------------------------------------------------------------  */
  // Logging the Moving Window average of the Voltage and Current for every 200 ms.
  if (millisVal/LOG_INT != millisValPrev/LOG_INT) { //the 200ms period between samples is marked by a change in the value of millisVal/200

/* --------------------------------------------------------------
  |...CALL FUNCTION TO MEASURE VOLTAGE AND CURRENT EVERY 200ms.. |
   --------------------------------------------------------------  */    
    float volt = readVoltage();
    float current = -1 * readCurrent();

/* --------------------------------------------------------------
  |........CALCULATE POWER AND NEW ENERGY EVERY 200ms........... |
   --------------------------------------------------------------  */    
    float Power = volt * current;
    newEnergy += Power * LOG_INT / 1000; // This always the newly computed Energy

/* --------------------------------------------------------------
  |..........RETRIEVE X,Y,Z ACCELERATION EVERY 200ms............ |
   --------------------------------------------------------------  */
    // Getting new Accelerometer Event
    sensors_event_t accelEvent;
    accel.getEvent(&accelEvent); // Library Call

/* --------------------------------------------------------------
  |......CALL FUNCTION TO MEASURE ANALOG INPUTS EVERY 200ms..... |
   --------------------------------------------------------------  */    
    float analog_input_2 = readAnalogInput2();
    float analog_input_3 = readAnalogInput3();

    boolean dig_in_4 = digitalRead(Digital);

/* --------------------------------------------------------------
  |.....CALL FUNCTION TO LOG DATA TO '.csv' FILE EVERY 200ms.... |
   --------------------------------------------------------------  */
    // Saving the Data to a LogFile.
    saveLogData(volt, current, Power, millisVal, Energy + newEnergy / 1000, analog_input_2, analog_input_3, rpm, dig_in_4, accelEvent);

/* --------------------------------------------------------------
  |......CALL FUNCTION TO DISPLAY INFO ON LCD EVERY SECOND...... |
   --------------------------------------------------------------  */ 
    if (millisVal/LCD_INT != millisValPrev/LCD_INT){
    LCD_Display(analog_input_2, analog_input_3, dig_in_4, accelEvent);
    }
  
  }

/* --------------------------------------------------------------
  |..GET PREVIOUS ENERGY STORED IN EEPROM AND UPDATE EVERY 10s.. |
   --------------------------------------------------------------  */
  if (millisVal/E_INT != millisValPrev/E_INT) {    

    // Updating the Value of Energy to the EEPROM Memory so that the most recent Energy Value is available even when the power is switched off.
    Energy += newEnergy / 1000; // Here we are computing cumulative energy value in kJ
    newEnergy = 0;

    eepromUpdate(); //update eeprom by writing Energy, offCurrent and eeprom_c in designated memory addresses.

  }

/* --------------------------------------------------------------
  |.................CHECK FOR DATA CORRUPTION................... |
   --------------------------------------------------------------  */
  boolean CounterCorruption = CheckCounterCorruption(); //check if there has been corruption in the eeprom writes counter and correct it
  boolean DataCorruption = CheckDataCorruption(); //check if there has been corruption in the data stored in eeprom and correct it

  if(CounterCorruption || DataCorruption){ //if there has been any corruption in eeprom, send warning
    clearLCD();
    cursorSet(0,0);
    Serial.print("WARNING:    DATA");
    cursorSet(1,0);
    Serial.print("CORRUPTION FOUND");
    CounterCorruption = 0;
    DataCorruption = 0;
  }
  
   millisValPrev = millisVal; //store time capture so it can be compared with next time capture
}

/* --------------------------------------------------------------
  |...............INTERRUPT FUNCTIONS START HERE................ |
   --------------------------------------------------------------  */

//interrupt resets energy to 0 when reset button is pressed
void reset_eeprom() { 
  static unsigned long last_interrupt_time = 0; //last time button was pressed
  unsigned long interrupt_time = millis(); //time button was pressed and interrupt began
  if(interrupt_time - last_interrupt_time > 200){  //if statement used to debounce button
    Energy = 0; // Clearing the Cumulative Energy Stored in the EEPROM.
    eepromUpdate();
  
    boolean CounterCorruption = CheckCounterCorruption(); //check if there has been corruption in the eeprom writes counter and correct it
    boolean DataCorruption = CheckDataCorruption(); //check if there has been corruption in the data stored in eeprom and correct it
  
    if(CounterCorruption || DataCorruption){ //if there has been any corruption in eeprom, send warning
      clearLCD();
      cursorSet(0,0);
      Serial.print(" ");
      Serial.print("WARNING:    DATA");
      cursorSet(1,0);
      Serial.print("CORRUPTION FOUND");
      CounterCorruption = 0;
      DataCorruption = 0;    
    }
  }
  last_interrupt_time = interrupt_time;   //update to debounce button
}

// This keeps count of the no. of revolutions for every 'x' ms as specified by LOG_INT 
void count_revolutions() {
  revolutions += 1.0 / MAGNETS;  
}


/* --------------------------------------------------------------
  |................REGULAR FUNCTIONS START HERE................. |
   --------------------------------------------------------------  */

// Always gives a value between -320 and 320 Amperes with 0.5V => -320A and 4.5V => 320A
float readCurrent() {
  long Current = analogRead(CurrIn); // Always returns a value between 0 and 1023
  float CurrentVal = Current * 5.0 / 1023.0 ; // Converting Digital Values (0 to 1023) to Analog Voltages
  CurrentVal = (CurrentVal - 2.5) * 100 / 0.625; // convert analog voltage value to current in Amps
                                  
  return CurrentVal;
}

// Always returns a value between 0.00 and 100.00 Volts
float readVoltage() {
  return (analogRead(VoltIn) * 100.0 / 1023.0) ;
}

// Always returns a value between 0.00 and 5.00 Volts
float readAnalogInput2() {
  return (analogRead(A2) * 5.0 / 1023.0);
}

// Always returns a value between 0.00 and 5.00 Volts
float readAnalogInput3() {
  return (analogRead(A3) * 5.0 / 1023.0);
}

// stores all relevant data in sd card
void saveLogData(float Volt, float Current, float Power, unsigned long millisVal, float Energy, float AI2, float AI3, float rpm, boolean D, sensors_event_t accelEvent) {
  // This opens a new file for writing if it doesn't exist
  // Else it will append the data to the end of the file
  logFile = SD.open(filename, FILE_WRITE);

  // Logging the PhotoVolt data to the SD Card
  logFile.print(millisVal);
  logFile.print(",");

  logFile.print(Volt, 1);
  logFile.print(",");

  logFile.print(Current, 1);
  logFile.print(",");

  logFile.print(Power / 1000, 1); // Logging the Power in kW
  logFile.print(",");

  // Logging the Energy every 2 Seconds in the LogFile
  if (millisVal / ELOG_INT != millisValPrev / ELOG_INT) {
    logFile.print(Energy, 0); // 2s Interval and Logging the Energy in kJ
    logFile.print(",");
  }
  else{    
    logFile.print(" ,");
  }

  // Logging the Analog Input 2
  logFile.print(AI2);
  logFile.print(",");
  
  // Logging the Analog Input 3
  logFile.print(AI3);
  logFile.print(",");
  
  // Logging the RPM
  logFile.print(rpm);
  logFile.print(",");
  
  // X-Axis g-Force (Acceleration)
  logFile.print(accelEvent.acceleration.x);
  logFile.print(",");

  // Y-Axis g-Force (Acceleration)
  logFile.print(accelEvent.acceleration.y);
  logFile.print(",");

  // Z-Axis g-Force (Acceleration)
  logFile.print(accelEvent.acceleration.z);
  logFile.print(",");
  
  // Digital Input 4 
  logFile.print(D);
  logFile.print(",");
    
  logFile.println("");
  logFile.close();
}

//Defines what will be dilplayed on the LCD screen
float LCD_Display(float AI2, float AI3, boolean D, sensors_event_t accelEvent) {

  clearLCD();
  cursorSet(0,0);
  Serial.print("RM");
  Serial.print(rpm, 0); 
  cursorSet(0,7);
  Serial.print("A");
  Serial.print(AI2, 1);
  Serial.print("B");
  Serial.print(AI3, 1);  
  if (D == 1) {
  Serial.print("D");
  }
  cursorSet(1,0);
  Serial.print("E");
  if(Energy >= 0){
    Serial.print(Energy, 0);  
  }else{
    Serial.print(Energy * -1, 0);
  }
  cursorSet(1,7);
  if(accelEvent.acceleration.x <0){
    Serial.print(accelEvent.acceleration.x, 0);   
  }else{
    Serial.print("+");
    Serial.print(accelEvent.acceleration.x, 0);
  }
  cursorSet(1,10);
  if(accelEvent.acceleration.y <0){
    Serial.print(accelEvent.acceleration.y, 0);   
  }else{
    Serial.print("+");
    Serial.print(accelEvent.acceleration.y, 0);
  }
  cursorSet(1,13);
  if(accelEvent.acceleration.z <0){
    Serial.print(accelEvent.acceleration.z, 0);   
  }else{
    Serial.print("+");
    Serial.print(accelEvent.acceleration.z, 0);
  }
  
}


//need to make data_i which keeps track of data index in eeprom
void eepromUpdate(){ 
  noInterrupts();
  
  data_i = (data_i + 4) % 256 + 512;   //update data index (data_i) to write next data sample
  int backup_i = data_i + 256; //index for backup data

  
  EEPROM.put(data_i, Energy); //write new energy in next EEPROM address
  EEPROM.put(backup_i, Energy); 

  eeprom_c++;  //update eeprom 4byte writes counter (eeprom_c). Energy and PastOffCurr = 1 4byte write.

  a = (a + 4) % 256;  //update eeprom_c address         
  int b = a + 256;    //back up eeprom_c address location
  EEPROM.put(a, eeprom_c);   
  EEPROM.put(b, eeprom_c);

  interrupts();  
}

//checks for corrupted data in the eeprom
boolean CheckDataCorruption(){ 
  noInterrupts(); 
  boolean corruption = 0; // tells whether corruption is true(1) or false(0)
  float Energy2 = 0; //Energy read from backup Data memory in eeprom
  int backup_i = data_i + 256; //index for backup data

  EEPROM.get(data_i, Energy);
  EEPROM.get(backup_i, Energy2); 

  //if Energy does not match the value in its backup memory then correct Energy and signal that there was data corrupted
  if( (Energy != Energy2) ){
    EEPROM.get((data_i - 4) % 256 + 512, Energy);
    EEPROM.put(data_i, Energy); 
    corruption = 1;
  } 
  
  interrupts();  
  return(corruption);
}

// checks for a corrupted write count in the eeprom while in the program main loop
boolean CheckCounterCorruption(){
  noInterrupts();
  
  boolean corruption = 0; // tells whether corruption is true(1) or false(0)
  unsigned long eeprom_c2 = 0; // eeprom_c read from backup eeprom_c memory in eeprom
  int b = a + 256; //back up eeprom_c address location
  
  EEPROM.get(a, eeprom_c);
  EEPROM.get(b, eeprom_c2);

  //if eeprom_c does not match the value in its backup memory then correct eeprom_c and signal that there was data corrupted
  if( eeprom_c != eeprom_c2 ){
    if(a != 0){
      EEPROM.get( a - 4, eeprom_c); //
    }else{
      EEPROM.get( 252, eeprom_c);
    }
    eeprom_c++;
    EEPROM.put(a, eeprom_c);
    EEPROM.put(b, eeprom_c);
    corruption = 1;
  }

  interrupts();
  return(corruption);
}

//checks for a corrupted write count in the eeprom while starting up the program
//this function uses a different correction method from CheckCounterCorruption() designed especially for startup
boolean CheckSetupCounterCorruption(){
  noInterrupts();
  boolean corruption = 0;
  unsigned long eeprom_c1 = 0;
  unsigned long eeprom_c2 = 0;
  int b = a + 256;

  EEPROM.get(a + 4, eeprom_c1);
  EEPROM.get(b + 4, eeprom_c2);

  //if eeprom_c does not match the value in its backup memory then correct eeprom_c and signal that there was data corrupted
  if( eeprom_c1 != eeprom_c2 ){
    eeprom_c++;
    a = (a + 4) % 256;
    b = a + 256;
    EEPROM.put(a, eeprom_c);
    EEPROM.put(b, eeprom_c);
    data_i = (eeprom_c % 64) * 4 + 512;
    corruption = 1;
  }
  
  interrupts();
  return(corruption);
}


//  LCD  FUNCTIONS-- keep the ones you need.
// More Details at http://www.newhavendisplay.com/specs/NHD-0216K3Z-FL-GBW-V3.pdf on Page [7]
void clearLCD() {
  Serial.write(254); // Prefix: 0xFE => 254
  Serial.write(81); // 0x51 => 81
}

void displayOn() {
  Serial.write(254); // Prefix 0xFE
  Serial.write(65);  // 0x41
}

void displayOff() {
  Serial.write(254); // Prefix 0xFE
  Serial.write(66); // 0x42
}


// start a new line
void newLine() {
  cursorSet(1, 0); // 1st Row and Column 0.
}

// move the cursor to the home position
void cursorHome() {
  Serial.write(254);
  Serial.write(70); // Set the Cursor to (0,0) position on the LCD
}

//set LCD backlight brightness level with number between 1 and 8: 8 being brightest, 1 being backlight off.
//If going to change brightness, for some reason requires to set brightness to 8 first and then reset it to whatever brightness you desire.
void setBacklightBrightness(int brightness) {
   Serial.write(0xFE); //command prefix
   Serial.write(0x53); //command to set backlight
   Serial.write(8);    //preset brightness to 8
 
   Serial.write(0xFE);
   Serial.write(0x53);
   Serial.write(brightness); //set brightness to desired level

}

// move the cursor to a specific place
// Here is xpos is Column No. and ypos is Row No.
void cursorSet(int ypos, int xpos) {
  Serial.write(254);
  Serial.write(69);

  // Bounding the X-Position
  if (xpos >= 15) {
    xpos = 15;
  }
  else if (xpos <= 0) {
    xpos = 0;
  }

  // Bounding the Y-Position
  if (ypos >= 1) {
    ypos = 1;
  }
  else if (ypos <= 0) {
    ypos = 0;
  }

  int finalPosition = ypos * 64 + xpos;

  Serial.write(finalPosition);
  //Serial.write(ypos); //Row position
}

// move cursor left by one Space
void cursorLeft() {
  Serial.write(254);
  Serial.write(73);  // 0x49
}

// move cursor right by One Space
void cursorRight() {
  Serial.write(254);
  Serial.write(74); // 0x4A
}

//loads characters used for frownie face on "EEPROM over warranty" error message
void loadCharacters(){ 
    Serial.write(0xFE);
    Serial.write(0x54);
    Serial.write(0x00);
    Serial.write(0x00);    
    Serial.write(0x05);
    Serial.write(0x02);
    Serial.write(0x05);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);


    Serial.write(0xFE);
    Serial.write(0x54);
    Serial.write(0x01);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x0E);
    Serial.write(0x11);
    Serial.write(0x00);

    Serial.write(0xFE);
    Serial.write(0x54);
    Serial.write(0x03);
    Serial.write(0x00);    
    Serial.write(0x14);
    Serial.write(0x08);
    Serial.write(0x14);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);  
}
