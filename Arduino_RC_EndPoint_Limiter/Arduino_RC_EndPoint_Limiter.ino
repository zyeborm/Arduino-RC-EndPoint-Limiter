// Limit switch processor, limits drive to prevent pushing into the end stop and blowing things up
// Jake Anderson 2018-11-11
// Don't kill yourself with it, let me know if you use it.
// This is set up for digital radios so it's very intolerant of bad signals
// Designed for an arduino nano, serial rate is 115200
// fast LED flashes during startup with RX of valid signal
// Slower LED flashes during operation with rx of valid signal
// Limit lights operate always regardless of signal but will flash if signal is good and limiting is being applied
// If both switches are closed output is disabled (no pulseout)
// designed for ~50 PPS servo signals 1-2ms long

#include <Servo.h> 
#include <EEPROM.h>

#define Version 0.002

// I/O setup
#define RCInputPin 12 // pin the reciever is connected to
#define LEDOutPin 13 // pin the status LED is connected to
#define LowSwitchLEDPin 6 // pin the Low switch active LED is connected to
#define HighSwitchLEDPin 7 // pin the High switch active LED is connected to
#define RCOutputPin 11 // pin the RC pulseout is sent on

//for both of these operation of the LED is enabled even without input signal, to allow for testing
//solid on for limit switch hit
//fast flashing for RC output enabled and a limit switch engaged
#define SwitchHighPin 3   // pin for LED indicating the high switch is triggered and output is limited as a result
#define SwitchLowPin 2  // pin for LED indicating the low switch is triggered and output is limited as a result


#define FilterMod 10 // Status update output Modulo division of pulse in count for output (10 = ~5Hz at 50PPS), requires pulses to operate or will only happen at the timeout rate (~.2hz)

//holds configuration settings, defaults first in comments

#define SettingsTypeVersion 1 //Increment this when changing the struct, this is compared with byte 0 if it's different (IE new struct format) defaults are loaded instead.
struct SettingsType {  
  unsigned int SettingsVersion; // version of the settings config, mainly used to check for 0xFFFF IE unconfigured eeprom to load default values, also cool to see

  //output limits
  unsigned int MaxOutputPulse; // 2000 cap output to value
  unsigned int MinOutputPulse; // 1000 cap output to value
  unsigned int CenterOutputPulse; // 1500 Central pulse width input

  //Safety Stuff
  byte StartupPulsesRequired; // 100 how may pulses between CenterPointInputMin and Max are needed before starting, ensures no movement when powered on
  unsigned int MinPulse; // 900 min valid signal pulse
  unsigned int MaxPulse; // 2300 max valid signal pulse
  byte SignalLimit; // 10 how many good servo pulses are needed to say we have good signal. 
  bool DisableOutputOnBadSignal; //True Disable output if bad/invalid signal for a while, will output a CenterOutputPulse first to zero the output sooner.
  
  //arming  
  byte CenterPulsesReq; //100 how many pulses between CenterPointInputMin and CenterPointInputMax are required to arm  
  unsigned int CenterPointInput; // 1500 Central pulse width input  
  unsigned int CenterPointInputMin; //1000 Central pulse width input minimum value considered as centered for safety (a tollerance essentially)
  unsigned int CenterPointInputMax; //2000 Central pulse width input maximum value considered as centered for safety (a tollerance essentially)
};

SettingsType Settings = {};

void ResetSettingsDefault()
{
  //As settings is global no issue with passing structs, lazyness wins again.
  //not reset, this should always come from eeprom Settings.SettingsVersion; // version of the settings config, mainly used to check for 0xFFFF IE unconfigured eeprom to load default values, also cool to see

  //output limits
  Settings.MaxOutputPulse = 2000; // 2000 cap output to value
  Settings.MinOutputPulse = 1000; // 1000 cap output to value
  Settings.CenterOutputPulse = 1500; // 1500 Central pulse width input

  //Safety Stuff
  Settings.StartupPulsesRequired = 100; // 100 how may pulses between CenterPointInputMin and Max are needed before starting, ensures no movement when powered on
  Settings.MinPulse = 900; // 900 min valid signal pulse
  Settings.MaxPulse = 2300; // 2300 max valid signal pulse
  Settings.SignalLimit = 10; // 10 how many good servo pulses are needed to say we have good signal. 
  Settings.DisableOutputOnBadSignal = true;

  //arming  
  Settings.CenterPulsesReq = 100; // how many pulses between CenterPointInputMin and CenterPointInputMax are required to arm  
  Settings.CenterPointInput = 1500; // 1500 Central pulse width input  
  Settings.CenterPointInputMin = 1400; //1000 Central pulse width input minimum value considered as centered for safety (a tollerance essentially)
  Settings.CenterPointInputMax = 1600; //2000 Central pulse width input maximum value considered as centered for safety (a tollerance essentially)
}
void SaveSettings()
{
  
}
void ReadSettings()
{
  //read settings from eeprom, filling with defaults if eeprom is blank (0xff) or it appears corrupt
  //just adding all the bytes individually allowing it to rollover and making a checksum at the end
  //bool SettingsTypeVersionCode = 0; // local var for the eeprom read
  if (EEPROM.read(0) == SettingsTypeVersion) //struct matches
  {    

    Serial.println("Reading eeprom");    
    EEPROM.get(1, Settings ); //start eeprom at 1 to give room for data format indication
    Serial.print("Got settings version :");
    Serial.println(Settings.SettingsVersion);
    if (Settings.SettingsVersion == 65535) 
    {
      //Settings is blank (or has been written 65534 times which is really getting up there
      Serial.println("Settings version indicates blank, setting settings to default");
      ResetSettingsDefault();
    }
  } else {
      Serial.println("Data storage version mismatch, setting settings to default");
      ResetSettingsDefault();
  }
  
}


void setupmenu() {
  SettingsType SettingsBuffer = {}; //buffers settings 
  SettingsType SettingsDefault = {}; //holds the default values
  unsigned int val = 0;
  
  //fill SettingsDefault with defaults by copying the actual values to the buffer, resetting then copying back
  // it's a hack because settings is global
  SettingsBuffer = Settings;
  ResetSettingsDefault();
  SettingsDefault = Settings;
  Settings = SettingsBuffer;

  

  Serial.println("Setup Menu");
  Serial.println("Enter 0 to accept the current value (first value) 1 for the default value or enter a new value");  
  Serial.println("0 [current value] 1 [default value]");  
  Serial.setTimeout(60000);
  while (true) {
    // statement(s)
    Serial.println("Setup Menu");
    Serial.println("Max Output Pulse");
    //Serial.println((String)"0 [" + Settings.MaxOutputPulse + "] 1 ["+ SettingsDefault.MaxOutputPulse +"]");
    val = Serial.parseInt();
    //Serial.println((String)" Set to : " + val);
    
  }
  
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}


byte ValidPulseTrain = 0; //how many good pulses have we recieved
bool SignalGood = false; // have we recieved enough good servo pulses to trust the input

Servo OutputServo;

void SwitchLowChange() //called every time the low switch is changed, no debounce or anything on here not needed debounce is done in main loop (kinda)
{
  digitalWrite(LowSwitchLEDPin, !digitalRead(SwitchLowPin));
}

void SwitchHighChange() {
  digitalWrite(HighSwitchLEDPin, !digitalRead(SwitchHighPin));  
}

void setup() {
  //spew serial stuff identifying the state and requirements for operation
  //then wait for StartupPulsesRequired (200) pulses between MinPulse (900) and TriggerPoint (1250) before leaving the init

  byte StartupPulses = 0;
  byte zeros = 0; // stop spewing zeros at startup with no signal

  int ch1; // Servo input values, in microseconds so use int
  
  unsigned long CurrentTime = 0; //stores millis()
  unsigned long LastTime = 0; //stores millis() used to for interval timing of slow (no signal) flash
  int RxData = 0; //used to recieve serial data

  //ResetSettingsDefault();
  pinMode(RCInputPin, INPUT_PULLUP); // Set our input pins as such and add a pullup in case it is disconnected from the RX

  pinMode(LowSwitchLEDPin, OUTPUT); // Set our output pins as such
  pinMode(SwitchLowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SwitchLowPin), SwitchLowChange , CHANGE);

  pinMode(HighSwitchLEDPin, OUTPUT);
  pinMode(SwitchHighPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SwitchHighPin), SwitchHighChange, CHANGE); 
     
  OutputServo.attach(RCOutputPin);
  
  
  Serial.begin(115200); // init serial high speed to minimise time in serial though it's meant to be non blocking now 2000000
  
  digitalWrite(LEDOutPin, LOW);
  digitalWrite(RCOutputPin, LOW);

  Serial.println("End point limiter starting");
  Serial.println((String)"Version : " + Version);
  //Serial.println(Version);

  ReadSettings();
  Serial.println((String)"Center stick for arming Pulses : " + (Settings.CenterPulsesReq)); 
  //Serial.println(); 
  //Serial.println();
  Serial.println("Pre Arm pulse Seq");
  serialFlush(); //dump anything recieved up until now.
  
  while (StartupPulses < Settings.StartupPulsesRequired) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      RxData = Serial.read();
      if( RxData == 83) {
        setupmenu();
      }
    }
            
    ch1 = pulseIn(RCInputPin, HIGH, 40000);
    if ((ch1 > Settings.CenterPointInputMin) && (ch1 < Settings.CenterPointInputMax)) { //signal is valid and in safe range
      StartupPulses++;
      if (StartupPulses % 5 == 0) {
        digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));  //Very fast toggle LED pin during startup with good signal %10 is too close to regular flashing to signal arming       
        Serial.println((String) "Press S for setup : Valid Pulses Rx : " + StartupPulses + "/" + Settings.StartupPulsesRequired);
        //Serial.print("/");
        //Serial.println(Settings.StartupPulsesRequired);
      }
    } else {
      // invalid signal recieved
      zeros++;
      if (ch1 == 0) {  // we get lots of zeros until the Rx has locked on        
        StartupPulses = 0;
        if (zeros % 64 == 0) {
                  Serial.println("No Pulse Recieved");
        }
      } else {
        StartupPulses = 0;
        if (zeros % 32 == 0) {        
          Serial.print("Press S for setup : Out of safe range Pulse Rx : ");
          Serial.println(ch1);
        }
      }
      
      // slow flash if we are in a bad signal state, might mess up (aliasing) with the fast flash but that could be usefull info anyway so wontfix
      CurrentTime = millis();
      if (CurrentTime - LastTime > 500) {        
        digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));
        //digitalWrite(LowSwitchLEDPin,!digitalRead(LowSwitchLEDPin));
        digitalWrite(LowSwitchLEDPin, !digitalRead(SwitchLowPin));
        LastTime = CurrentTime; 
      } 
    }
  }

  ValidPulseTrain = Settings.SignalLimit; // I mean you would hope so
  SignalGood = true;
  Serial.println("Armed");
}

void print_status(int PulseDuration,int PulseOutVal,bool SwitchLow,bool SwitchHigh) {
  //prints some internal state, takes ValidPulseTrain and SignalGood from globals

  Serial.print("InputPulseWidth : ");
  Serial.print(PulseDuration);
  Serial.print(" OutputPulseWidth : ");
  Serial.print(PulseOutVal);
  Serial.print(" SwitchLow : ");
  Serial.print(SwitchLow);
  Serial.print(" SwitchHigh : ");
  Serial.print(SwitchHigh);     
  Serial.print(" : ValidPulseTrain : ");
  Serial.print(ValidPulseTrain);

  if (SignalGood) {
    Serial.println(" Signal OK");
  } else {
    Serial.println(" Signal BAD");
  }
}

void loop() {
  static byte filter = 0; // used to only write out infrequently
  static int PulseOutVal = 1500; // output pulse value
  
  int ch1; // Servo input values, in microseconds so use int
  static bool DipSignalLights = false; // If true then black out the switch signal lights to indicate good signal being inhibited, stay oposite to status LED for clarity

  ch1 = pulseIn(RCInputPin, HIGH, 40000); // Read the pulse width of the servo, needs to be long because it spends most of it's time off.
  
  
  //2ms max on time * 50 pps = 100ms maximum on time per second.

  if ((ch1 > Settings.MinPulse) && (ch1 < Settings.MaxPulse)) {  // if signal is in range
    if (ValidPulseTrain < Settings.SignalLimit) {      // and if we are recovering from a bad pulse
      ValidPulseTrain++;                    // then add one to the valid pulse train
    }
  } else {
    SignalGood = false;                      //if the signal is out of range then kill everything for a while
    ValidPulseTrain = 0;
  }


  if (ValidPulseTrain == 10) {    //we have recieved at least SignalLimit of valid pulses in a row so signal is good
    SignalGood = true;
  }

  if (SignalGood) {
    //we are happy with the radio reception, do the actual processing

    PulseOutVal = ch1; //set output to input then do the limiting later
    
    if ((!digitalRead(SwitchLowPin)) & (ch1 < Settings.CenterOutputPulse)) // minimum switch is activated and input is still driving into the switch
    {
      PulseOutVal = Settings.CenterOutputPulse; //set the output to center to prevent driving into the switch more
    }

    if ((!digitalRead(SwitchHighPin)) & (ch1 > Settings.CenterOutputPulse)) // Maximum switch is activated and input is still driving into the switch
    {
      PulseOutVal = Settings.CenterOutputPulse; //set the output to center to prevent driving into the switch more
    }

    if ((!digitalRead(SwitchHighPin) & (!digitalRead(SwitchLowPin))))
    {
      PulseOutVal = Settings.CenterOutputPulse;  // both switches are activated, something is broken.
    }
  } else {
    PulseOutVal = Settings.CenterOutputPulse; //if signal is bad then output default signal     
    digitalWrite(LowSwitchLEDPin, !digitalRead(SwitchLowPin)); //reset signal LEDs
    digitalWrite(HighSwitchLEDPin, !digitalRead(SwitchHighPin)); //reset signal LEDs
  }
  OutputServo.write(PulseOutVal);
    //occasionally print status updates
  filter++;
  
  if (filter % FilterMod == 0) { // only print every nth time
    print_status(ch1,PulseOutVal,!digitalRead(SwitchLowPin),!digitalRead(SwitchHighPin));
    if (SignalGood) {      
      DipSignalLights = digitalRead(LEDOutPin);
      digitalWrite(LEDOutPin, !DipSignalLights);  //slower toggle LED pin during operation with good signal            
      digitalWrite(LowSwitchLEDPin, (!digitalRead(SwitchLowPin) & DipSignalLights) );
      digitalWrite(HighSwitchLEDPin, (!digitalRead(SwitchHighPin) & DipSignalLights));
    }
  }

}
