// Limit switch processor, limits drive to prevent pushing into the end stop and blowing things up
// Jake Anderson 2018-11-11
// Don't kill yourself with it, let me know if you use it.
// This is set up for digital radios so it's very intolerant of bad signals
// Designed for an arduino nano, serial rate is 115200
// fast LED flashes during startup with RX of valid signal
// Slower LED flashes during operation with rx of valid signal
// designed for 50 PPS servo signals 1-2ms long

#define Version 0.001

// I/O setup
#define RCInputPin 2 // pin the reciever is connected to
#define LEDOutPin 13 // pin the status LED is connected to
#define ValveOutPin 4 // pin the fet switch to run the burket is connected to

// Tuning
#define TriggerPoint 1250 // 1250 what pulsewidth value do we need to fire (in microsconds - between 1000 and 2000 generally)
#define PulseOutTime 200 // how many ms we keep the output on for

//Safety Stuff, has defaults first in comments
#define FireSafetyReq 2 // 2 how many good pulses above the trigger value are needed (so a random high pulse won't trigger it)
#define LowPulsesReq 10 // 10 how many low pulses are needed between firings
#define FilterMod 32 // 32 at 50 pps servo rate 32 is ~2hz 
#define StartupPulsesRequired 100 // 100 how may pulses below the trigger point are needed before starting, ensures trigger is off before running
#define MinPulse 900 // 900 min valid signal pulse
#define MaxPulse 2300 // 2300 max valid signal pulse
#define SignalLimit 10 // 10 how many good servo pulses are needed to say we have good signal. 

byte ValidPulseTrain = 0; //how many good pulses have we recieved
bool SignalGood = false; // have we recieved enough good servo pulses to trust the input

void setup() {
  //spew serial stuff identifying the state and requirements for operation
  //then wait for StartupPulsesRequired (200) pulses between MinPulse (900) and TriggerPoint (1250) before leaving the init

  byte StartupPulses = 0;
  byte zeros = 0; // stop spewing zeros at startup with no signal

  int ch1; // Servo input values, in microseconds so use int

  pinMode(RCInputPin, INPUT); // Set our input pins as such
  pinMode(LEDOutPin, OUTPUT); // Set our output pins as such
  pinMode(ValveOutPin, OUTPUT); // Set our output pins as such

  Serial.begin(115200); // init serial high speed to minimise time in serial though it's meant to be non blocking now 2000000
  digitalWrite(LEDOutPin, LOW);
  digitalWrite(ValveOutPin, LOW);

  Serial.println("Servo switch starting");
  Serial.print("Version : ");
  Serial.println(Version);
  Serial.print("Trigger Point: ");
  Serial.println(TriggerPoint);
  Serial.print("LowPulses : ");
  Serial.println(LowPulsesReq + 1); // Base 0 correction
  Serial.print("FireSaftey Pulses Req : ");
  Serial.println(FireSafetyReq + 1); // Base 0 correction
  Serial.println();
  Serial.println("Pre Arm pulse Seq");

  while (StartupPulses < StartupPulsesRequired) {
    ch1 = pulseIn(RCInputPin, HIGH, 40000);
    if ((ch1 > MinPulse) && (ch1 < TriggerPoint)) { //signal is valid and below trigger
      StartupPulses++;
      if (StartupPulses % 10 == 0) {
        digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));  //fast toggle LED pin during startup
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
    }
  }

  ValidPulseTrain = SignalLimit; // I mean you would hope so
  SignalGood = true;
  Serial.println("Armed");
}

void print_status(int PulseDuration, byte LowTimes, byte FireSafetyVal) {
  //prints some internal state, takes ValidPulseTrain and SignalGood from globals

  Serial.print("PulseWidth : ");
  Serial.print(PulseDuration);
  Serial.print(" : LowTime : ");
  Serial.print(LowTimes);
  Serial.print(" : FireSafety : ");
  Serial.print(FireSafetyVal);
  Serial.print(" : ValidPulseTrain : ");
  Serial.print(ValidPulseTrain);

  if (SignalGood) {
    Serial.println(" Signal OK");
  } else {
    Serial.println(" Signal BAD");
  }
}

void loop() {
  static byte LowTime = 0; //Number of valid low (< TriggerPoint) pulses we have recieved
  static byte FireSafety = 0; //require 3 high servo pulses before firing, prevent misfires
  static byte filter = 0; // used to only write out infrequently
  static bool PreFireFlag = false; //used during prefire to ensure sequential pulses not just noise

  int ch1; // Servo input values, in microseconds so use int



  //occasionally print status updates
  filter++;
  if (filter % FilterMod == 0) { // only print every nth time
    print_status(ch1, LowTime, FireSafety);
    if (SignalGood) {
      digitalWrite(LEDOutPin, !digitalRead(LEDOutPin));  //slower toggle LED pin during operation with good signal
    }
  }


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

    if ((ch1 >= TriggerPoint) && (LowTime >= 10)) { // if it's been low for a while then allow fire
      Serial.print("PreFire! : Pulsewidth = ");
      Serial.println(ch1);

      if (FireSafety >= FireSafetyReq) { //we have had FireSafetyReq good fire pulses, watch for base 0, too many here makes a delay on firing
        Serial.print("Fire! : Pulsewidth = ");
        Serial.println(ch1);
        digitalWrite(LEDOutPin, HIGH);
        digitalWrite(ValveOutPin, HIGH);
        delay(PulseOutTime);
        digitalWrite(LEDOutPin, LOW);
        digitalWrite(ValveOutPin, LOW);
        LowTime = 0;
        FireSafety = 0;
      } else {
        FireSafety++;
      }
    }

    if ((LowTime < 10) && (ch1 < TriggerPoint)) {
      LowTime++;
    }

  }
}
