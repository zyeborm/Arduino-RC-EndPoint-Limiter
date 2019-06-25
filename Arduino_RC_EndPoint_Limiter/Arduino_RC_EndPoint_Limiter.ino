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

#define Version 0.001

// I/O setup
#define RCInputPin 12 // pin the reciever is connected to
#define LEDOutPin 13 // pin the status LED is connected to
#define LowSwitchLEDPin 6 // pin the Low switch active LED is connected to
#define HighSwitchLEDPin 7 // pin the High switch active LED is connected to
#define RCOutputPin 4 // pin the RC pulseout is sent on

//for both of these operation of the LED is enabled even without input signal, to allow for testing
//solid on for limit switch hit
//fast flashing for RC output enabled and a limit switch engaged
#define SwitchHighPin 3   // pin for LED indicating the high switch is triggered and output is limited as a result
#define SwitchLowPin 2  // pin for LED indicating the low switch is triggered and output is limited as a result


// Tuning
#define CenterPointInput 1500 // Central pulse width input
#define CenterPointInputMin 1400 //Central pulse width input minimum value considered as centered for safety (a tollerance essentially)
#define CenterPointInputMax 1600 //Central pulse width input maximum value considered as centered for safety (a tollerance essentially)

#define MaxOutputPulse 2000 // cap output to value
#define MinOutputPulse 1000 // cap output to value
#define CenterOutputPulse 1500 // Central pulse width input

//Safety Stuff, has defaults first in comments
#define StartupPulsesRequired 100 // 100 how may pulses between CenterPointInputMin and Max are needed before starting, ensures no movement when powered on
#define MinPulse 900 // 900 min valid signal pulse
#define MaxPulse 2300 // 2300 max valid signal pulse
#define SignalLimit 10 // 10 how many good servo pulses are needed to say we have good signal. 
#define CenterPulsesReq 100 // how many pulses between CenterPointInputMin and CenterPointInputMax are required to arm

#define FilterMod 10 // Status update output Modulo division of pulse in count for output (10 = ~5Hz at 50PPS), requires pulses to operate or will only happen at the timeout rate (~.2hz)

byte ValidPulseTrain = 0; //how many good pulses have we recieved
bool SignalGood = false; // have we recieved enough good servo pulses to trust the input

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


  
  pinMode(RCInputPin, INPUT_PULLUP); // Set our input pins as such and add a pullup in case it is disconnected from the RX

  pinMode(LowSwitchLEDPin, OUTPUT); // Set our output pins as such
  pinMode(SwitchLowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SwitchLowPin), SwitchLowChange , CHANGE);

  pinMode(HighSwitchLEDPin, OUTPUT);
  pinMode(SwitchHighPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SwitchHighPin), SwitchHighChange, CHANGE); 
     
  pinMode(RCOutputPin, OUTPUT); // Set our output pins as such
  
  
  
  Serial.begin(115200); // init serial high speed to minimise time in serial though it's meant to be non blocking now 2000000
  digitalWrite(LEDOutPin, LOW);
  digitalWrite(RCOutputPin, LOW);

  Serial.println("End point limiter starting");
  Serial.print("Version : ");
  Serial.println(Version);
  Serial.print("Center stick for arming Pulses : ");
  Serial.println(CenterPulsesReq + 1); // Base 0 correction
  Serial.println();
  Serial.println("Pre Arm pulse Seq");

  while (StartupPulses < StartupPulsesRequired) {
    ch1 = pulseIn(RCInputPin, HIGH, 40000);
    if ((ch1 > CenterPointInputMin) && (ch1 < CenterPointInputMax)) { //signal is valid and in safe range
      StartupPulses++;
      if (StartupPulses % 10 == 0) {
        digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));  //fast toggle LED pin during startup with good signal        
        Serial.print(StartupPulses);
        Serial.print("/");
        Serial.println(StartupPulsesRequired);
      }
    } else {
      // invalid signal recieved
      if (ch1 == 0) {  // we get lots of zeros until the Rx has locked on
        zeros++;
        StartupPulses = 0;
        if (zeros % 64 == 0) {
          Serial.println("Zero input");
        }
      } else {
        StartupPulses = 0;
        Serial.print("Bad Pulse Rx :");
        Serial.println(ch1);
      }
      
      // slow flash if we are in a bad signal state, might mess up (aliasing) with the fast flash but that could be usefull info anyway so wontfix
      CurrentTime = millis();
      if (CurrentTime - LastTime > 500) {
        digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));
        //digitalWrite(LowSwitchLEDPin,!digitalRead(LowSwitchLEDPin));
        digitalWrite(LowSwitchLEDPin, !digitalRead(SwitchLowPin));
        Serial.println(CurrentTime);
        LastTime = CurrentTime; 
      } 
    }
  }

  ValidPulseTrain = SignalLimit; // I mean you would hope so
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
  

  ch1 = pulseIn(RCInputPin, HIGH, 40000); // Read the pulse width of the servo, needs to be long because it spends most of it's time off.
  
  
  //2ms max on time * 50 pps = 100ms maximum on time per second.

  if ((ch1 > MinPulse) && (ch1 < MaxPulse)) {  // if signal is in range
    if (ValidPulseTrain < SignalLimit) {      // and if we are recovering from a bad pulse
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
    
    if ((!digitalRead(SwitchLowPin)) & (ch1 < CenterPointInput)) // minimum switch is activated and input is still driving into the switch
    {
      PulseOutVal = CenterPointInput; //set the output to center to prevent driving into the switch more
    }

    if ((!digitalRead(SwitchHighPin)) & (ch1 > CenterPointInput)) // Maximum switch is activated and input is still driving into the switch
    {
      PulseOutVal = CenterPointInput; //set the output to center to prevent driving into the switch more
    }

    if ((!digitalRead(SwitchHighPin) & (!digitalRead(SwitchLowPin))))
    {
      PulseOutVal = CenterPointInput;  // both switches are activated, something is broken.
    }
  } else {
    PulseOutVal = CenterPointInput; //if signal is bad then output default signal 
  }

    //occasionally print status updates
  filter++;
  if (filter % FilterMod == 0) { // only print every nth time
    print_status(ch1,PulseOutVal,!digitalRead(SwitchLowPin),!digitalRead(SwitchHighPin));
    if (SignalGood) {
      digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));  //slower toggle LED pin during operation with good signal
    }
  }

}
