//SCULLCOM HOBBY ELECTRONICS
//ELECTRONIC DC LOAD PROJECT
//Software Version 35 (4x5 Matrix Keypad Version - modified key layout)
//14th April 2018

#include <SPI.h>                              //include SPI library (Serial Peripheral Interface)
#include <Wire.h>                             //include I2C library
#include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip
#include <math.h>                             //
#include <Adafruit_MCP4725.h>                 //Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <MCP342X.h>                          //uChip library avaiable from https://github.com/uChip/MCP342X
#include <MCP79410_Timer.h>                   //Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip
#include <EEPROM.h>                           //include EEPROM library used for storing setup data

#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad

const byte ROWS = 5;                          //five rows
const byte COLS = 4;                          //four columns

//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'<','0','>','F'},
  {'7','8','9','E'},
  {'4','5','6','D'},
  {'1','2','3','C'},
  {'A','B','#','*'}
};

byte rowPins[ROWS] = {9, 10, 11, 12, 14}; //connect to the row pin outs of the keypad
byte colPins[COLS] = {5, 6, 7, 8}; //connect to the column pin outs of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
char customKey;

char decimalPoint;                            //used to test for more than one press of * key (decimal point)

Adafruit_MCP4725 dac;                         //constructor

MCP342X adc;

const byte MCP79410_ADDRESS = 0x6f;           //0x6f is the default address for the MCP79410 Real Time Clock IC
MCP79410_Timer timer = MCP79410_Timer(MCP79410_ADDRESS);

//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);    //0x27 is the default address of the LCD with I2C bus module

const byte pinA = 2;                          //digital pin (also interrupt pin) for the A pin of the Rotary Encoder (changed to digital pin 2)
const byte pinB = 4;                          //digital pin for the B pin of the Rotary Encoder

const byte CursorPos = 17;                    //analog pin A3 used as a digital pin to set cursor position (rotary encoder push button)
const byte LoadOnOff = 15;                    //analog pin A1 used as a digital pin to set Load ON/OFF

const byte TriggerPulse = 16;                 //analog pin A2 used as a digital pin for trigger pulse input in transient mode

const byte fan = 3;                           //digital pin 3 for fan control output (changed to Digital pin 3)
const byte temperature = A6;                  //analog pin used for temperature output from LM35 (was A0 previously but changed)
int tempmin = 30;
int tempmax = 35;
int temp;                                     //

float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float Seconds = 0;                            //time variable used in Battery Capacity Mode (BC)
float SecondsLog = 0;                         //variable used for data logging of the time in seconds
float BatteryCutoffVolts;                     //used to set battery discharge cut-off voltage
int   BatteryNumOfCells = 1;				  //number of cell in battery discharge preset mode
float MaxBatteryCurrent = 1.0;                //maximum battery current allowed for Battery Capacity Testing

int stopSeconds;                              //store for seconds when timer stopped

int CP = 8;                                   //cursor start position

boolean toggle = false;                       //used for toggle of Load On/Off button

unsigned long controlVoltage = 0;             //used for DAC to control MOSFET

long current = 0;                             //variable used by ADC for measuring the current
long voltage = 0;                             //variable used by ADC for measuring the voltage

float reading = 0;                            //variable for Rotary Encoder value divided by 1000

float setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 0.980;    //calibration adjustment - set as required if needed (was 1.000)

float displayCurrentCal = 0.000;              //calibration correction for LCD current display (was 0.040)
int Load = 0;                                 //Load On/Off flag

float setControlCurrent = 0;                  //variable used to set the temporary store for control current required

int VoltsDecimalPlaces = 3;                   //number of decimal places used for Voltage display on LCD

float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load

float ResistorCutOff = 999;                   //maximum Resistor we want to deal with in software
float BatteryCurrent;                         //
float LoadCurrent;                            //

int CurrentCutOff = EEPROM.read(0x00);
int PowerCutOff = EEPROM.read(0x20);
int tempCutOff = EEPROM.read(0x40);

int setReading = 0;                           //

int ControlVolts = 0;                         //used to set output current
float OutputVoltage = 0;                      //

String Mode ="  ";                            //used to identify which mode

int modeSelected = 0;                         //Mode status flag

int lastCount = 50;                           //
volatile float encoderPosition = 0;           //
volatile unsigned long factor= 0;             //number of steps to jump
volatile unsigned long encoderMax = 999000;   //sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

float LiPoCutOffVoltage = 3.0;                //set cutoff voltage for LiPo battery
float LiFeCutOffVoltage = 2.8;                //set cutoff voltage for LiFe battery
float NiCdCutOffVoltage = 1.0;                //set cutoff voltage for NiCd battery
float ZiZnCutOffVoltage = 1.0;                //set cutoff voltage for ZiZn battery
float PbAcCutOffVoltage = 1.75;               //set cutoff voltage for PbAc battery
String BatteryType ="    ";

byte exitMode = 0;                           //used to exit battery selection menu and return to CC Mode

char numbers[10];                            //keypad number entry - Plenty to store a representation of a float
byte index = 0;
int z = 1;                                   //was previously 0
float x = 0;
int y = 0;
int r = 0;

float LowCurrent = 0;               //the low current setting for transcient mode
float HighCurrent = 0;              //the high current setting for transcient mode
unsigned long transientPeriod;      //used to store pulse time period in transcient pulse mode
unsigned long current_time;         //used to store the current time in microseconds
unsigned long last_time;            //used to store the time of the last transient switch in micro seconds
boolean transient_mode_status;      //used to maintain the state of the trascient mode (false = low current, true = high current)

float transientList [10][2];        //array to store Transient List data
int total_instructions;             //used in Transient List Mode
int current_instruction;            //used in Transient List Mode

//--------------------------------Interrupt Routine for Rotary Encoder------------------------
void isr()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {  //
    if (digitalRead(pinB) == LOW)
    {
      encoderPosition = encoderPosition - factor;
    }else{
      encoderPosition = encoderPosition + factor;
    }
    encoderPosition = min(encoderMax, max(0, encoderPosition));  // sets maximum range of rotary encoder
    lastInterruptTime = interruptTime;
  }
    }
//---------------------------------Initial Set up---------------------------------------
void setup() {
  Serial.begin(9600);                                      //used for testing only
 
  Wire.begin();                                            //join i2c bus (address optional for master)
  Wire.setClock(400000L);                                  //sets bit rate to 400KHz

  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);

  pinMode (CursorPos, INPUT_PULLUP);
  pinMode (LoadOnOff, INPUT_PULLUP);
 
  pinMode (TriggerPulse, INPUT_PULLUP);

  pinMode (fan, OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 1;                      //change PWM to above hearing (Kenneth Larvsen recommendation)
  pinMode (temperature, INPUT);
 
  analogReference(INTERNAL);                               //use Arduino internal reference for tempurature monitoring

  attachInterrupt(digitalPinToInterrupt(pinA), isr, LOW);

  dac.begin(0x61);                                         //the DAC I2C address with MCP4725 pin A0 set high
  dac.setVoltage(0,false);                                 //reset DAC to zero for no output current set at Switch On

  lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
  lcd.setBacklightPin(3,POSITIVE);                         // BL, BL_POL
  lcd.setBacklight(HIGH);                                  //set LCD backlight on
  lcd.clear();                                             //clear LCD display
  lcd.setCursor(6,0);									   //set LCD cursor to column 0, row 4
  lcd.print("SCULLCOM");										//print SCULLCOM to display with 5 leading spaces (you can change to your own)
  lcd.setCursor(1,1);										//set LCD cursor to column 0, row 1 (start of second line)
  lcd.print("Hobby Electronics");										//print Hobby Electronics to display (you can change to your own)
  lcd.setCursor(1,2);
  lcd.print("DC Electronic Load");										//DC Electronic Load
  lcd.setCursor(0,3);
  lcd.print("Ver. 37P(Mod.)");										//Version
  delay(3000);												//3000 mSec delay for intro display
  lcd.clear();												 //clear dislay
  setupLimits();
  delay(3000);
  lcd.clear();

  last_time = 0;                                           //set the last_time to 0 at the start (Transicent Mode)
  transient_mode_status = false;                           //set the initial transient mode status (false = low, true = high);
  setCurrent = LowCurrent;                                 //first set the current to the low current value (Transicent Mode)
 
  lcd.setCursor(8,0);
  lcd.print("OFF");                                        //indicate that LOAD is off at start up
  Current();                                               //sets initial mode to be CC (Constant Current) at Power Up
  
  customKey = customKeypad.getKey();

  //Check temp on Startup, if higher than lower limit, turn on fan
  //This is needed due isteresi control
	temp = analogRead(temperature);
	temp = analogRead(temperature);
	temp = analogRead(temperature);
	temp = analogRead(temperature);
	temp = temp * 0.107421875;
	if (temp > tempmin) {
		digitalWrite(fan, HIGH);
	}

  
}

//------------------------------------------Main Program Loop---------------------------------
void loop() {

 if(CurrentCutOff > 5){                                 //Test and go to user set limits if required
    userSetUp();
     }else{
  
  readKeypadInput();                                     //read Keypad entry
  
  if (digitalRead(LoadOnOff) == LOW) {
    LoadSwitch();                                          //Load on/off
  }
  
  transient();                                           //test for Transient Mode
  
  lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
  lcd.print(Mode);                                       //display mode selected on LCD (CC, CP, CR or BC)

  if(Mode != "TC" && Mode != "TP" && Mode != "TT" && Mode != "TL"){      //if NOT transient mode then Normal Operation
    reading = encoderPosition/1000;                        //read input from rotary encoder 
    maxConstantCurrentSetting();                           //set maxiumum Current allowed in Constant Current Mode (CC)
    powerLevelCutOff();                                    //Check if Power Limit has been exceeded
  
    temperatureCutOff();                                   //check if Maximum Temperature is exceeded
  
    batteryCurrentLimitValue();                            //Battery Discharge Constant Current Limit Value in BC Mode
    displayEncoderReading();                               //display rotary encoder input reading on LCD
    lastCount = encoderPosition;                           //store rotary encoder current position
    CursorPosition();                                      //check and change the cursor position if cursor button pressed
  }else{
    transientLoadToggle();                                  //Start Transient Mode
  }
    
  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET

  batteryCapacity();                                     //test if Battery Capacity (BC) mode is selected - if so action
  fanControl();                                          //call heatsink fan control
  }
}
//------------------------------------------------Read Keypad Input-----------------------------------------------------
void readKeypadInput (void) {
  customKey = customKeypad.getKey();
  
//  if (customKey != NO_KEY){                             //only used for testing keypad (uncomment if required for testing keypad)
// Serial.print("customKey = ");                          //only used for testing keypad (uncomment if required for testing keypad)
// Serial.println(customKey);                             //only used for testing keypad (uncomment if required for testing keypad)
//  }                                                     //only used for testing keypad (uncomment if required for testing keypad)

if(customKey == 'E'){                                     //check for Load on/off
  LoadSwitch();
}

  if(customKey == 'D'){                                   //check if Set-Up Mode Selected 
  delay(200);
  toggle = false;                                         //switch Load OFF
  userSetUp();
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point text character reset
}
 
  if(customKey == 'C'){                                   //check if Transient Mode Selected
  toggle = false;                                         //switch Load OFF
  transientType();
 
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point text character reset
  }
               
  if(customKey == 'A'){                                   //check if Constant Current button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");
  Current();                                              //if selected go to Constant Current Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }
         
  if(customKey == 'B'){                                   //check if Constant Power button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF"); 
  Power();                                                //if selected go to Constant Power Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }
          
  if(customKey == '#'){                                   //check if Constant Resistance button pressed  
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");  
  Resistance();                                           //if selected go to Constant Resistance Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }

  if(customKey == '*'){                                   //check if Battery Capacity button pressed
  dac.setVoltage(0,false);                                //Ensures Load is OFF - sets DAC output voltage to 0
  toggle = false;                                         //switch Load OFF
  batteryType();                                          //select battery type
  index = 0;
  z = 1;                                                  //sets column position for LCD displayed character
  decimalPoint = (' ');                                   //clear decimal point test character reset

    if (exitMode == 1){                                   //if NO battery type selected revert to CC Mode
    lcd.setCursor(8,0);
    lcd.print("OFF");
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
    }
    else
    {
    lcd.setCursor(16,2);
    lcd.print(BatteryType);                               //print battery type on LCD 
    lcd.setCursor(8,0);
    lcd.print("OFF");
    timer.reset();                                        //reset timer
    BatteryLifePrevious = 0;
    CP = 9;                                               //set cursor position
    BatteryCapacity();                                    //go to Battery Capacity Routine
    }
  }

if (Mode != "BC"){

  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,3);                                //                   
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '>'){                                   //check if decimal button key pressed
      if (decimalPoint != ('>')){                         //test if decimal point entered twice - if so skip 
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,3);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('>');                             //used to indicate decimal point has been input
        }
      }

  if(customKey == '<'){                                 //clear input entry
     index = 0;
     z = 1;                                             //sets column position for LCD displayed character
     lcd.setCursor(z,3);
     lcd.print("     ");
     numbers[index] = '\0';                             //
     decimalPoint = (' ');                              //clear decimal point test character reset
    }

  if(customKey == 'F') {                                //use entered number
    x = atof(numbers);     
         reading = x;
         encoderPosition = reading*1000;
         index = 0;
         numbers[index] = '\0';
         z = 1;                                         //sets column position for LCD displayed character
         lcd.setCursor(0,3);
         lcd.print("        ");
         decimalPoint = (' ');                          //clear decimal point test character reset
          }
    }
  
}

//----------------------Limit Maximum Current Setting-----------------------------------------
void maxConstantCurrentSetting (void) {
  if (Mode == "CC" && reading > CurrentCutOff){           //Limit maximum Current Setting
  reading = CurrentCutOff;
  encoderPosition = (CurrentCutOff * 1000);               //keep encoder position value at maximum Current Limit
  lcd.setCursor(0,3);
  lcd.print("                    ");                      //20 spaces to clear last line of LCD 
  }

  if (Mode == "CP" && reading > PowerCutOff) {             //Limit maximum Current Setting
        reading = PowerCutOff;
        encoderPosition = (PowerCutOff * 1000);            //keep encoder position value at maximum Current Limit
        lcd.setCursor(0,3);
        lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }

   if (Mode == "CR" && reading > ResistorCutOff ) {             //Limit maximum Current Setting
        reading = ResistorCutOff;
        encoderPosition = (ResistorCutOff * 1000);            //keep encoder position value at maximum Current Limit
        lcd.setCursor(0,3);
        lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }

}

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Exceeded Power");
  lcd.setCursor(8,0);
  lcd.print("OFF");
  toggle = false;                                         //switch Load Off
  }
}

//----------------------Battery Constant Current Limit Value------------------------------------------
void batteryCurrentLimitValue (void) {
  if (Mode == "BC" && reading > MaxBatteryCurrent){
  reading = MaxBatteryCurrent;
  encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at 1000mA
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {

    lcd.setCursor(8,2);                                      //start position of setting entry

    if ( ( Mode == "CP" || Mode == "CR" ) && reading < 100 ) {
        lcd.print("0");
    }
    
    if (reading < 10) {                                      //add a leading zero to display if reading less than 10
        lcd.print("0"); 
    }

    if ( Mode == "CP" || Mode == "CR" ) {
        lcd.print (reading, 2);                              //show input reading from Rotary Encoder on LCD
    } else {
        lcd.print (reading, 3);
    }
    lcd.setCursor (CP, 2);                                   //sets cursor position
    lcd.cursor();                                            //show cursor on LCD
}

//--------------------------Cursor Position-------------------------------------------------------
//Change cursor the position routine
void CursorPosition(void) {

    // Defaults for two digits before decimal and 3 after decimal point
    int unitPosition = 9;

    //Power and Resistance modes can be 3 digit before decimal but only 2 decimals
    if ( Mode == "CP" || Mode == "CR" ) {
        unitPosition = 10;        
    }

    if (digitalRead(CursorPos) == LOW) {
    
        delay(200);                                          //simple key bounce delay  
        CP = CP + 1;
        if (CP == unitPosition + 1 ) {
            CP = CP + 1;
        }
    }
    
    if (CP > 13)  { CP = unitPosition; }                     //No point in turning tens and hundreds
    if (CP == unitPosition +4 ) { factor = 1; }
    if (CP == unitPosition +3 ) { factor = 10; }
    if (CP == unitPosition +2 ) { factor = 100; }
    if (CP == unitPosition )    { factor = 1000; }
}

//---------------------------------------------Read Voltage and Current--------------------------------------------------------------
void readVoltageCurrent (void) {
	static int16_t  result;

	adc.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_16BIT | MCP342X_GAIN_1X);
	adc.startConversion();
	adc.getResult(&result);
	voltage = (long)result;

	adc.configure(MCP342X_MODE_CONTINUOUS |	MCP342X_CHANNEL_2 |	MCP342X_SIZE_16BIT | MCP342X_GAIN_4X);
	adc.startConversion();
	adc.getResult(&result);
	current = (long)result;
	
}

//-----------------------------------Calculate Actual Voltage and Current and display on LCD-----------------------------------------
void ActualReading(void) {

  ActualCurrent = (((current*2.048)/32767) * 2.5);        //calculate load current
  currentDisplayCal();                                    //LCD display current calibration correction
  ActualVoltage = (((voltage*2.048)/32767) * 50.4);       //calculate load voltage upto 100v (was 50)
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualPower <=0){
    ActualPower = 0;
  }

 if (ActualVoltage <=0.0){                              //added to prevent negative readings on LCD due to error
  ActualVoltage = 0.0;
 }
 if (ActualCurrent <= 0.0){                             //added to prevent negative readings on LCD due to error
  ActualCurrent = 0.0;
 }
 
 lcd.setCursor(0,1);
    
    if ( ActualCurrent < 10.0 ) {
        lcd.print(ActualCurrent,3);
    } else {
        lcd.print(ActualCurrent,2);
    }
    
    lcd.print("A");
    lcd.print(" ");
    
    if (ActualVoltage < 10.0) {
        lcd.print(ActualVoltage, 3);
    } else {
        lcd.print(ActualVoltage, 2);
    }    

    lcd.print("V");
    lcd.print(" ");
     
    if (ActualPower < 100 ) {
        lcd.print(ActualPower,2);
    } else {
        lcd.print(ActualPower,1);
    }
    lcd.print("W");
    lcd.print(" ");
}

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (Mode == "CC"){
  setCurrent = reading*1000;                                //set current is equal to input value in Amps
  setReading = setCurrent;                                  //show the set current reading being used
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "CP"){
  setPower = reading*1000;                                  //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;                       //
  }

  if (Mode == "CR"){
  setResistance = reading;                                  //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

if (Mode == "TC" || Mode == "TP" || Mode == "TT" || Mode == "TL"){                            //Transient Modes
  setControlCurrent = (setCurrent * 1000) * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

}

//-------------------------------------Battery Capacity Discharge Routine----------------------------------------------------
void batteryCapacity (void) {
  if (Mode == "BC"){
    
    setCurrent = reading*1000;                             //set current is equal to input value in Amps
    setReading = setCurrent;                               //show the set current reading being used
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent;

    lcd.setCursor(0,3);
    lcd.print (timer.getTime());                           //start clock and print clock time

    Seconds = timer.getTotalSeconds();                     //get totals seconds
  
    LoadCurrent = ActualCurrent;                           //if timer still running use present Actual Current reading
    if (timer.status() == 2){                              //if timer is halted then use last Actual Current reading before timer stopped
      LoadCurrent = BatteryCurrent;
      }
 
    BatteryLife = (LoadCurrent*1000)*(Seconds/3600);       //calculate battery capacity in mAh
    lcd.setCursor(9,3);
    BatteryLife = round(BatteryLife);

    if(BatteryLife >= BatteryLifePrevious){                //only update LCD (mAh) if BatteryLife has increased
  
      if (BatteryLife < 10) {                              //add a 3 leading zero to display if reading less than 10
      lcd.print("000");
      }

      if (BatteryLife >= 10 && BatteryLife <100){          //add a 2 leading zero to display
      lcd.print("00");  
      }

      if (BatteryLife >= 100 && BatteryLife <1000){        //add a 1 leading zero to display
      lcd.print("0"); 
      }
  
    lcd.print(BatteryLife,0);
    lcd.setCursor(13,3);
    lcd.print("mAh");
    BatteryLifePrevious = BatteryLife;                      //update displayed battery capacity on LCD
    } 
  }


  if (Mode == "BC" && ActualVoltage <= BatteryCutoffVolts){ //stops clock if battery reached cutoff level and switch load off

  BatteryCurrent = ActualCurrent;
  dac.setVoltage(0,false);                                  //reset DAC to zero for no output current set at switch on                                             
  toggle = false;                                           //Load is toggled OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");                                         //indicate that LOAD is off at start up
  timer.stop();
  }

      if (Mode == "BC" && Load == 1){                       //Routine used for data logging in Battery Capacity Mode
          if (Seconds != SecondsLog){                       //only send serial data if time has changed
            SecondsLog = Seconds;
            Serial.print (SecondsLog);                    //sends serial data of time in seconds
            Serial.print (",");                             //sends a comma as delimiter for logged data
            Serial.println (ActualVoltage);                   //sends serial data of Voltage reading         
              }
          }

}

//--------------------------------------------------Fan Control----------------------------------------------------------
void fanControl(void) {
	temp = analogRead(temperature);
	temp = temp * 0.107421875;                                // convert to Celsius

	/*if (temp >= 40) {                                        //if temperature 40 degree C or above turn fan on.
		digitalWrite(fan, HIGH);
	}
	else {
		digitalWrite(fan, LOW);                               //otherwise turn fan turned off
	}*/

	if (temp >= tempmax) {
		digitalWrite(fan, HIGH);
	}
	else if (temp <= tempmin) {
		digitalWrite(fan, LOW);
	}


  lcd.setCursor(16,0);
  lcd.print(temp);                                        //display temperature of heatsink on LCD
  lcd.print((char)0xDF);
  lcd.print("C");

}

  
//-----------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {
  
  delay(200);                                              //simple key bounce delay 
 
    if(toggle)
    {
      lcd.setCursor(8,0);
      lcd.print("OFF");
      current_instruction = 0;                            //reset current instruction for Transient List Mode to zero
      last_time = 0;                                      //reset last time to zero
      transientPeriod = 0;                                //reset transient period time to zero
      setCurrent = 0;                                     //reset setCurrent to zero
      toggle = !toggle;
      Load = 0;        
    }
    else
    {
      lcd.setCursor(8,0);
      lcd.print("ON ");
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //clear bottom line of LCD
      toggle = !toggle;
      Load = 1;
    }
}

//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  Mode = ("CC");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  Mode = ("CP");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set W = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("W");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                               //sets cursor starting position to units.
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  Mode = ("CR");
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");  
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set R = ");
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print((char)0xF4);
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                               //sets cursor starting position to units.
}

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  Mode = ("BC");
  lcd.setCursor(0,0);
  lcd.print("BATTERY");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
}

//----------------------Battery Type Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                                         //reset EXIT mode
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Select Battery Type");  
  lcd.setCursor(0,1);
  lcd.print("1=LiPo/Li-Ion 2=LiFe");
  lcd.setCursor(0,2);
  lcd.print("3=NiCd/NiMH   4=ZiZn");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("5=Set Voltage 6=Exit");                    //20 spaces so as to allow for Load ON/OFF to still show

  customKey = customKeypad.waitForKey();                //stop everything till the user press a key.

  if (customKey == '1'){
	  BatteryCutoffVolts = LiPoCutOffVoltage;
	  BatteryType = ("LiPo");
    }

  if (customKey == '2'){
	  BatteryCutoffVolts = LiFeCutOffVoltage;
	  BatteryType = ("LiFe");
    }

  if (customKey == '3'){
	  BatteryCutoffVolts = NiCdCutOffVoltage;
	  BatteryType = ("NiCd");  
    }

  if (customKey == '4'){
	  BatteryCutoffVolts = ZiZnCutOffVoltage;
	  BatteryType = ("ZiZn"); 
    }

  if (customKey == '5'){ 
	  BatteryType = ("SetV");
    }

  if (customKey == '6'){                                  //Exit selection screen
	  exitMode = 1;
    }

  if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#' || customKey == 'E' || customKey == 'F' || customKey == '<' || customKey == '>' ){
	  batteryType();                                                      //ignore other keys
    }

  if (BatteryType == "LiPo" && exitMode != 1) {
	  setBatteryCells();
	  BatteryCutoffVolts = (BatteryNumOfCells*LiPoCutOffVoltage);
	  //BatteryType = BatteryType+" "+String(BatteryNumOfCells)+"cells";
  }
  if (BatteryType == "LiFe" && exitMode != 1) {
	  setBatteryCells();
	  BatteryCutoffVolts = (BatteryNumOfCells*LiFeCutOffVoltage);
  }
  if (BatteryType == "NiCd" && exitMode != 1) {
	  setBatteryCells();
	  BatteryCutoffVolts = (BatteryNumOfCells*NiCdCutOffVoltage);
  }
  if (BatteryType == "ZiZn" && exitMode != 1) {
	  setBatteryCells();
	  BatteryCutoffVolts = (BatteryNumOfCells*ZiZnCutOffVoltage);
  }

  if(BatteryType == "SetV" && exitMode != 1){
	 setBatteryCutOff();
    }



  batteryTypeSelected();                                    //briefly display battery type selected and discharge cut off voltage

  lcd.clear();

}

//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
  dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() == 1){
    timer.stop();
    }
  
  }else{
  //Serial.println("Control Voltage");                    //used for testing only
  //Serial.println(controlVoltage);                       //used for testing only
  dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() != 1){
    timer.start();
    }
  }
}

//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode !=1){                                      //if battery selection was EXIT then skip this routine
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Battery Selected");
  lcd.setCursor(8,1);
  lcd.print(BatteryType);                                 //display battery type selected
  lcd.setCursor(2,2);
  lcd.print("Discharge Cutoff");
  lcd.setCursor(6,3);
  lcd.print(BatteryCutoffVolts);                          //display battery discharge cut off voltage
  lcd.print(" volts");                                    
  delay(3000);
  }
}

//--------------------------Set Battery Cut-Off Voltage--------------------------------------------
void setBatteryCutOff (void) {

  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("Enter Battery");
  lcd.setCursor(3,1);
  lcd.print("Cut-Off Voltage");
  y = 8;
  z = 8;
  r = 2;

  inputValue();
  BatteryCutoffVolts = x;

  lcd.clear();
}

//--------------------------Set Battery number of Cells--------------------------------------------
void setBatteryCells(void) {

	lcd.clear();
	lcd.setCursor(3, 0);
	lcd.print("Enter Number of");
	lcd.setCursor(3, 1);
	lcd.print("Cells in Series");
	y = 8;
	z = 8;
	r = 2;

	inputValue();
	BatteryNumOfCells = x;

	lcd.clear();
}

//------------------------Key input used for Battery Cut-Off and Transient Mode------------------------
void inputValue (void){

 while(customKey != 'F'){               //check if enter pressed (was previously #)
  
  customKey = customKeypad.getKey();
  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,r);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '>'){                                   //Decimal point
      if (decimalPoint != ('>')){                         //test if decimal point entered twice - if so ski
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,r);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('>');                               //used to indicate decimal point has been input
        }
      }

 if(customKey == '<'){                                    //clear entry
    index = 0;
    z = y;
    lcd.setCursor(y,r);
    lcd.print("     ");
    numbers[index] = '\0';                                //
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }

 }
 
  if(customKey == 'F') {                                  //enter value 
    x = atof(numbers);     
    index = 0;
    numbers[index] = '\0';
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }
}

//----------------------------------------Transient Mode--------------------------------------------
void transientMode (void) {

if(Mode != "TL"){

  y = 11;
  z = 11;
  
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Transient Mode");  
  lcd.setCursor(0,1);
  lcd.print("Set Low  I=");
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue();
  
  if(x >= CurrentCutOff){
    LowCurrent = CurrentCutOff;
  }else{
    LowCurrent = x;
  }
  lcd.setCursor(11,r);
  lcd.print(LowCurrent,3);
  
  customKey = '0';

  z = 11;

  lcd.setCursor(0,2);
  lcd.print("Set High I=");
  lcd.setCursor(19,2);
  lcd.print("A");
  r = 2;
  inputValue();
  if(x >= CurrentCutOff){
    HighCurrent = CurrentCutOff;
  }else{
    HighCurrent = x;
  }
  lcd.setCursor(11,r);
  lcd.print(HighCurrent,3);

  customKey = '0';

  if(Mode == "TC" || Mode == "TP"){
  z = 11;

  lcd.setCursor(0,3);
  lcd.print("Set Time  = ");
  lcd.setCursor(16,3);
  lcd.print("mSec");
  r = 3;
  inputValue();
  transientPeriod = x;
  lcd.setCursor(11,r);
  lcd.print(transientPeriod);
  }else{
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }

  lcd.clear();

  toggle = false;                                           //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");                                         //print on display OFF
    }else{  
  transientListSetup();
  lcd.clear();
 
  toggle = false;                                           //switch Load OFF
  lcd.setCursor(8,0);
  lcd.print("OFF");                                         //print on display OFF
  }
}

//----------------------------------------Transient Type Selection--------------------------------------------
void transientType (void) {
  toggle = false;                                         //switch Load OFF
  exitMode = 0;                                           //reset EXIT mode
  lcd.noCursor();                                         //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Transient Mode");  
  lcd.setCursor(0,1);
  lcd.print("1 = Continuous");
  lcd.setCursor(0,2);
  lcd.print("2 = Toggle");
  lcd.setCursor(11,2);                                    //
  lcd.print("3 = Pulse");                                 //
  lcd.setCursor(0,3);                                     //
  lcd.print("4 = List");                                  //
  lcd.setCursor(11,3);                                    //
  lcd.print("5 = Exit");                                  //

  customKey = customKeypad.waitForKey();                  //stop everything till the user press a key.

  if (customKey == '1'){
  Mode = ("TC"); 
    }

  if (customKey == '2'){
   Mode = ("TT");
    }

  if (customKey == '3'){
  Mode = ("TP");  
    }

  if (customKey == '4'){
  Mode = ("TL");  
    }

  if (customKey == '5'){                                  //Exit selection screen
  exitMode = 1;
    }

  if (customKey == '6' || customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#' || customKey == 'E' || customKey == 'F' || customKey == '<' || customKey == '>' ){
  transientType();                                                      //ignore other keys
  
    }
lcd.clear();

if (exitMode == 1){                                       //if NO Transient Mode type selected revert to CC Mode
    lcd.setCursor(8,0);
    lcd.print("OFF");
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
      }else{
    transientMode();
      }
 }
 
//----------------------------------------Transient--------------------------------------------
void transient (void) {
  
  if(Mode == "TC" || Mode == "TP" || Mode == "TT" || Mode == "TL"){
  lcd.noCursor();                                         //switch Cursor OFF for this menu 
  lcd.setCursor(0,0);
  lcd.print("DC LOAD");
  
  if(Mode != "TL"){
  lcd.setCursor(0,2);
  lcd.print("Lo=");
  lcd.setCursor(3,2);
  lcd.print(LowCurrent,3);
  lcd.setCursor(8,2);
  lcd.print("A");
  lcd.setCursor(11,2);
  lcd.print("Hi=");
  lcd.setCursor(14,2);                                    //
  lcd.print(HighCurrent,3);
  lcd.setCursor(19,2);
  lcd.print("A");
  }else{
    delay(1);
  }


  if(Mode == "TC" || Mode == "TP" || Mode == "TL"){
  lcd.setCursor(0,3);                                     //
  lcd.print("Time = ");
  lcd.setCursor(7,3);
  lcd.print(transientPeriod);
  lcd.setCursor(12,3);                                    //
  lcd.print("mSecs");
    }else{
     lcd.setCursor(0,3);
     lcd.print("  ");
    }
  }
delay(1);
}

//-------------------------------------Transcient List Setup-------------------------------------------
void transientListSetup(){

  lcd.noCursor();                                                 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setup Transient List");
  lcd.setCursor(0,1);
  lcd.print("Enter Number in List");
  lcd.setCursor(0,2);
  lcd.print("(between 2 to 10 max"); 
  y = 0;
  z = 0;
  r = 3;
  inputValue();
  total_instructions = int(x-1);
  customKey = '0';
  lcd.clear();
  
    for(int i=0; i<=(total_instructions); i++){
      lcd.setCursor(0,0);
      lcd.print("Set Current ");
      lcd.print(i+1);
      lcd.setCursor(16,1);
      lcd.print("A");
      y = 0;
      z = 0;
      r = 1;
      inputValue();            //get the users input value
      transientList[i][0] = x; //store the users entered value in the transient list 
      customKey = '0';
      lcd.setCursor(0,2);
      lcd.print("Set Time ");
      lcd.print(i+1);
      lcd.setCursor(16,3);
      lcd.print("mSec");
      y = 0;
      z = 0;
      r = 3;
      inputValue();            //get the users input value
      transientList[i][1] = x; //store the users entered value in the transient list
      customKey = '0';
      lcd.clear();   
  }
  current_instruction = 0;      //start at first instrution
}

//-------------------------------------Transcient Load Toggel-------------------------------------------
void transientLoadToggle(){

  if(Mode == "TC"){
  current_time = micros();                              //get the current time in micro seconds()
  if (last_time == 0){
    last_time = current_time;
  } else {
    switch (transient_mode_status){
      case (false):
          // we are in the low current setting
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
             transientSwitch(LowCurrent, true);  
          }
        break;
      case (true):
          // we are in the high current setting 
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
            transientSwitch(HighCurrent, true);            
          }
        break; 
      } 
     }
    }


  if(Mode == "TP"){
    current_time = micros();                            //get the current time in micro seconds()
    if (last_time == 0){
        last_time = current_time;
        transientSwitch(LowCurrent, true);
    }
    if (digitalRead(TriggerPulse) == LOW){
      // a trigger pluse is received
      // set to the high current
      transientSwitch(HighCurrent, true); 
    } else {
        if ((current_time - last_time) >= (transientPeriod * 1000.0)){
            transientSwitch(LowCurrent, true);            
        }
      }  
  }


 if(Mode == "TT"){          // this function will toggle between high and low current when the trigger pin is taken low
  if (digitalRead(TriggerPulse) == LOW){
    switch (transient_mode_status){
      case (false):
        transientSwitch(LowCurrent, true);
        delay(200);         //added to prevent key bounce when transient button used (date of addition 31-03-2018)
        break;
      case (true):
        transientSwitch(HighCurrent, true);
        delay(200);         //added to prevent key bounce when transient button used (date of addition 31-03-2018)   
        break;
      }
    }
  }


if(Mode == "TL"){
  if (Load == 1){                                                 //Only perform Transient List if Load is ON
   current_time = micros();                                       //get the current time in micro seconds()
    if (last_time == 0){
      last_time = current_time;
      transientPeriod = transientList[current_instruction][1];    //Time data for LCD display
      transientSwitch(transientList[current_instruction][0], false);
    }
    if((current_time - last_time) >= transientList[current_instruction][1] * 1000){     //move to next list instruction
        current_instruction++;
        if(current_instruction > total_instructions){
          current_instruction = 0;
        }
      transientPeriod = transientList[current_instruction][1];   //Time data for LCD display
      transientSwitch(transientList[current_instruction][0], false);
      }
    }
  }

}

//-------------------------------------Transcient Switch-------------------------------------------
void transientSwitch(float current_setting, boolean toggle_status){
  if (toggle_status){
  transient_mode_status = !transient_mode_status;
  }
  setCurrent = current_setting;
  //Serial.print("set current = ");                     //used for testing only
  //Serial.println(setCurrent);                         //used for testing only
  last_time = current_time;
}

//-------------------------------------User set up for limits-------------------------------------------------
void userSetUp (void) {
  y = 14;
  z = 14;
  
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("User Set-Up");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue();
  CurrentCutOff = x;
  EEPROM.write(0x00, CurrentCutOff);
  lcd.setCursor(14,r);
  lcd.print(CurrentCutOff);
  
  customKey = '0';

  z = 14;

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(19,2);
  lcd.print("W");
  r = 2;
  inputValue();
  PowerCutOff = x;
  EEPROM.write(0x20, PowerCutOff);              //
  lcd.setCursor(14,r);
  lcd.print(PowerCutOff);

  customKey = '0';

  z = 14;

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(18,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  r = 3;
  inputValue();
  tempCutOff = x;
  EEPROM.write(0x40, tempCutOff);
  lcd.setCursor(14,r);
  lcd.print(tempCutOff);

    lcd.clear();

    lcd.setCursor(8,0);
    lcd.print("OFF");
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
}

//------------------------------------------High Temperature Cut-Off--------------------------------------------------------------
void temperatureCutOff (void){
  if (temp >= tempCutOff){                                 //if Maximum temperature is exceeded
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("Over Temperature");
  lcd.setCursor(8,0);
  lcd.print("OFF");
  toggle = false;                                         //switch Load Off
  }
}

//-----------------------------------------Current Read Calibration for LCD Display --------------------------------------------
void currentDisplayCal (void){

  if(ActualCurrent <= 0){
    ActualCurrent = 0;
  }else if(Load == 0){
    ActualCurrent = 0;
  }else{
  ActualCurrent = ActualCurrent + displayCurrentCal;
  }

/*  if (ActualCurrent <= 0.5)
    {
    ActualCurrent = (ActualCurrent +(displayCurrentCal * 3));
    }
    else if (ActualCurrent >= 0.5 && ActualCurrent <1.0)
    {
    ActualCurrent = (ActualCurrent + (displayCurrentCal * 2));
    }
    else if (ActualCurrent >= 1.0 && ActualCurrent <= 1.4)
    {
    ActualCurrent = (ActualCurrent + (displayCurrentCal));
    }
    else 
    {
    ActualCurrent = ActualCurrent;
    }
*/
}
  
//-----------------------------Show limits Stored Data for Current, Power and Temp-----------------------------
  void setupLimits (void){
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Maximum Limits Set");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(17,1);
  lcd.print("A");
  lcd.setCursor(15,1);
  CurrentCutOff = EEPROM.read(0x00);
  lcd.print(CurrentCutOff);

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(17,2);
  lcd.print("W");
  lcd.setCursor(15,2);
  PowerCutOff = EEPROM.read(0x20);
  lcd.print(PowerCutOff);

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(17,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  tempCutOff = EEPROM.read(0x40);
  lcd.setCursor(15,3);
  lcd.print(tempCutOff);
  }
  
  //---------------------------------------------------------------------------------------------------------
