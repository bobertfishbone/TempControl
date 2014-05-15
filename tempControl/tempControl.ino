// Include libraries
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <Potentiometer.h>
#include <Adafruit_MAX31855.h>

// Define pins
#define powerPin 42
#define buttonPin 8
#define D0 43
#define CS 44
#define CLK 45
#define CS1 14
#define D01 15
#define CLK1 16

// Define hardware
Adafruit_MAX31855 inThermocouple(CLK, CS, D0);
Adafruit_MAX31855 exhaustThermocouple(CLK1, CS1, D01);
LiquidCrystal lcd(48,49,50,51,52,53);
Potentiometer potentiometer = Potentiometer(0);
static double Setpoint, Input, Output; 
PID myPID(&Input, &Output, &Setpoint, 1000,50,0, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime; 

// Initialize other global variables
byte stages = 0; // Number of stages
unsigned long duration[15]; // Duration of each stage
unsigned long stageduration[15]; // Captures how long the temp-based stages actually take
unsigned int maxtemp[15]; // Setpoint for each stage
unsigned int outtemp[15]; // When exhaust temp hits this temperature, move to next stage

void setup()
{
  lcd.begin(16,2); // Initialize LCD size
  //Initialize thermocouple pins
  pinMode(buttonPin, INPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(D0, INPUT);
  pinMode(CS, OUTPUT);
  pinMode(CLK1, OUTPUT);
  pinMode(D01, INPUT);
  pinMode(CS1, OUTPUT);

  //Allow sensors to stabilize, set powertail off
  digitalWrite(powerPin, LOW);
  lcd.clear();
  lcd.print(F("Loading..."));
  delay(1000);

  //Get all temp/duration values for stages
  stages = getPot("How many stages?", 15);

  // Loop through this for each stage
  for (byte i=1; i < stages + 1; i++)
  {
    String stageStr = "Stg";
    String stageHours = " Hours";
    String stageTemp = " Temp";
    String stageMins = " Minutes";
    String stageTimeTemp = " Time0 Temp1";

    //Is this a time-based or temp-based stage?
    boolean checkvar = getPot(stageStr + i + stageTimeTemp, 2);

    switch(checkvar)
    {

      // If this is a stage based on time duration:  
    case 0:
      {
        duration [i] = 1;
        // Get hours and minutes, assign to duration array for stage
        unsigned long hours = getPot(stageStr + i + stageHours, 24) * 3600000;
        unsigned long minutes = getPot(stageStr + i + stageMins, 60) * 60000;
        duration [i] = hours + minutes;

        // Get setpoint for stage, assign to temp array
        maxtemp [i] = getPot(stageStr + i + stageTemp, 500);
        outtemp [i] = 0; // No outtemp needed here!
        break;
      }

      // If this is a stage based on reaching a set temperature:
    case 1:
      {
        duration [i] = 0; //No duration needed here!

        // Get setpoint for stage; temp required to complete stage
        maxtemp [i] = getPot(stageStr + i + " air-in Temp", 500);
        outtemp [i] = getPot(stageStr + i + "exhaust Temp", 500);
        break;
      }

    }
  }
}



void loop()
{
  // Get initial temperature readings
  double inTemp = sampleFahrenheit(inThermocouple, 100);
  double exhaustTemp = sampleFahrenheit(exhaustThermocouple, 100);

  for(byte i = 1; i < stages + 1; i++)
  {
    unsigned long zeroedtime = millis(); // Initializes start time for stage
    stageduration[i] = millis() - zeroedtime; // Amount of time this stage is taking
    Setpoint = maxtemp[i]; //Assign max temperature to PID for this stage
    lcd.clear();
    // If duration > 0 (time-based)
    if(duration[i] > 0)
    {
      // Loops while elapsed time < max time for stage
      while(stageduration[i] < duration[i]) 
      {
        //Sample temperatures
        inTemp = sampleFahrenheit(inThermocouple, 100);
        exhaustTemp = sampleFahrenheit(exhaustThermocouple, 100);

        // Update LCD values
        updateLCD(i, stages, Setpoint, inTemp, exhaustTemp, stageduration[i], duration[i], 0);

        // Run PID
        pidCompute(inTemp);

        // Update stage duration
        stageduration[i] = millis() - zeroedtime;


      }
      // break;
    }

    // If duration = 0 (temp-based)
    else
    {
      while( exhaustTemp < outtemp[i])
      {
        // Sample temperatures
        inTemp = sampleFahrenheit(inThermocouple, 100);
        exhaustTemp = sampleFahrenheit(exhaustThermocouple, 100);

        // Update LCD values
        updateLCD(i, stages, Setpoint, inTemp, exhaustTemp, stageduration[i], 0, outtemp[i]);

        // Run PID
        pidCompute(inTemp);

        // Update stage duration
        stageduration[i] = millis() - zeroedtime;

      }
    }
  }

  // End of program
  while(1 < 2)
  {
    digitalWrite(powerPin, LOW); // Set powerpin Off
    lcd.clear();
    lcd.setCursor(0,0);

    // Loop through each stage; display duration of temp-based stages in minutes
    for(byte i = 1; i < stages+1; i++)
    {
      if (duration[i] = 0)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        String space = "  ";
        String minutestring = " min";
        lcd.print(i);
        lcd.print(space);
        lcd.print(stageduration[i]/60000);
        lcd.print(minutestring);
        delay(1000);
      } 
    }
  }
}



int getPot(String message, int sectors) {
  int sectorValue = 0;
  potentiometer.setSectors(sectors);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(message);
  lcd.setCursor(0,1);
  lcd.print(sectorValue);
  int buttonState = 0;
  while(buttonState == LOW) {
    sectorValue = potentiometer.getSector();
    lcd.setCursor(0,1);    
    lcd.print(sectorValue);
    if(sectorValue < 100)
    { 
      lcd.print("  "); 
    }
    buttonState = digitalRead(buttonPin);
  }
  delay(1000);
  return sectorValue;
}



void pidCompute(double temp) {
  //PID setup  
  windowStartTime = millis();  
  myPID.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);   //turn the PID on
  Input = temp; 
  myPID.Compute(); 
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize) { 
    windowStartTime += WindowSize; 
  } 
  if(Output > now - windowStartTime) { 
    digitalWrite(powerPin, HIGH); 
  } 
  else { 
    digitalWrite(powerPin, LOW); 
  }
}

double sampleFahrenheit(Adafruit_MAX31855 thermocouple, int maxSamples)
{
  double result = NAN;
  for (int i = 0; i < maxSamples; i++)
  {
    result = thermocouple.readFarenheit();
    if (!isnan(result)) // exit on first good reading.
      break;
    delay(10);
  }
  return result;
}

void updateLCD(byte i, byte stages, double setpoint, double inTemp, double exhaustTemp, long stageduration, long duration, int stagetemp) 
{
  lcd.setCursor(0,0); // Print current stage number
  lcd.print(i);
  lcd.setCursor(0,1); // Print total number of stages
  lcd.print(stages);
  lcd.setCursor(9,1); // Print temp setpoint
  lcd.print(setpoint, 0);
  lcd.setCursor(13,1); // Print in temperature
  lcd.print(inTemp, 0);
  lcd.setCursor(13, 0); // Print exhaust temperature
  lcd.print(exhaustTemp, 0);
  lcd.setCursor(4, 0); // Print elapsed time
  lcd.print(stageduration/60000);

  if(duration > 0)
  {
    lcd.setCursor(4, 1); // Print total stage time
    lcd.print(duration/60000);
    lcd.setCursor(9, 0); // Placeholder for temp-based setpoint
    lcd.print(F("N/A")); 
  }
  else
  {
    lcd.setCursor(4, 1); // Placeholder for time-based stage time
    lcd.print(F("N/A")); 
    lcd.setCursor(9, 0); // Print temp-based setpoint
    lcd.print(stagetemp); 
  }
}







