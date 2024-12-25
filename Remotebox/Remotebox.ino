#include <BleConnectionStatus.h>
//#include <Arduino.h>
#include <Bounce2.h>
//#include <Callback.h>
#include <BleCompositeHID.h>
//#include <KeyboardDevice.h>
//#include <MouseDevice.h>
#include <XboxGamepadDevice.h>
//#include <GamepadDevice.h>

// Debounce Objects
#define NumOfButtons 16
#define NumOfEncoder 3
#define NumOfMasks 3

// Define table
#define InsLED 26
#define NumOfIns 3
#define StatusUpdateInterval 30000
#define ErrorReportInterval 5000
#define DebonceInterval 25
#define DpadTriggeredInterval 150
#define StickOrigin 0
#define FilterFrame 5
#define Filterlimit 5


Bounce debouncers[NumOfButtons];

// Set Objects of BLE Devices
XboxGamepadDevice* gamepad;
//KeyboardDevice* keyboard;
//MouseDevice* mouse;
BleCompositeHID compositeHID("Remote Box", "Shimaru", 100); // deviceName, deviceManufacturer, batteryLevel

// Pin Mapping
byte BatSensor[2] = {36, 39};  //BAT+, BAT-
byte TopEncoder[2] = {14, 27};//CLK(A), DT(B)
byte MidEncoder[2] = {18, 19};//CLK(A), DT(B)
byte BotEncoder[2] = {4, 16};//CLK(A), DT(B)
byte physicBotton[16] = {34, 32, 35, 33, 25, 13, 12, 1, 22, 23, 3, 21, 5, 17, 2, 15}; //SW1 to SW16
uint16_t ButtonsOutput[16] = {
        XBOX_BUTTON_RB,   //SW1
        XBOX_BUTTON_Y,    //SW2
        XBOX_BUTTON_B,    //SW3
        XBOX_BUTTON_A,    //SW4
        XBOX_BUTTON_X,    //SW5
        XBOX_BUTTON_RS,     //SW6
        XBOX_BUTTON_RB,     //SW7
        XBOX_BUTTON_LB,   //SW8
        XBOX_BUTTON_Y,   //SW9
        XBOX_BUTTON_X,    //SW10
        XBOX_BUTTON_A,     //SW11
        XBOX_BUTTON_B,     //SW12
        XBOX_BUTTON_LS,     //SW13
        XBOX_BUTTON_LB,     //SW14
        XBOX_BUTTON_HOME,     //SW15
        XBOX_BUTTON_START};     //SW16
XboxDpadFlags directions[3][2] = {  
        {XboxDpadFlags::NORTH, XboxDpadFlags::SOUTH}, //Top clockwise, counter-CW
        {XboxDpadFlags::EAST, XboxDpadFlags::WEST},   //Middle clockwise, counter-CW
        { XboxDpadFlags((uint8_t)XboxDpadFlags::NORTH | (uint8_t)XboxDpadFlags::EAST),
          XboxDpadFlags((uint8_t)XboxDpadFlags::SOUTH | (uint8_t)XboxDpadFlags::WEST)}};  //Bottom clockwise, counter-CW
    


// Used Array
bool InputMask[NumOfMasks][NumOfButtons] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,}, //Full Input
  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1,}, //Left-four Mask
  {1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,}, //Right-four Mask
};

int InsInterval[NumOfIns][2] = { // OnTime, OffTime. Time refresh period 30000ms = 30s
  {3000, 3000},   //Working Normally
  {500, 500},     //Battery Low
  {6000, 0}       //Error Occured
};

int BatteryLvTable[4][2] = { // 100%, 80%, 60%, 40%, 20%
  {1342, 1179},
  {1331, 1210},
  {1321, 1241},
  {1312, 1272}  
};

// Status parameter
unsigned long Time;
unsigned long DpadTriggeredTime;
int16_t LeftThumbCount = 0;
byte MaskStatus = 0; // 0 Full Input, 1 Left-four Mask, 2 Right-four Mask
byte Status = 2; //0 Working Normally, 1 Battery Low, 2 Error Occured
byte BatteryPercentage = 100;
bool InpEventFlag = 0;
bool DpadReleaseNeedFlag = 0;
bool MsgFlag = 0; // Error occured message send delay used
Bounce InpBTNStatus[NumOfButtons]; //Status of Input pins
Bounce InpTopStatus[2]; //Status of Top encoder Input pins
Bounce InpMidStatus[2]; //Status of Middle encoder Input pins
Bounce InpBotStatus[2]; //Status of Bottom encoder Input pins

//Function definition
void HIDInitial(){
    // Set up gamepad
    XboxSeriesXControllerDeviceConfiguration* config = new XboxSeriesXControllerDeviceConfiguration();

    // The composite HID device pretends to be a valid Xbox controller via vendor and product IDs (VID/PID).
    // Platforms like windows/linux need this in order to pick an XInput driver over the generic BLE GATT HID driver. 
    BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
    /*
    Serial.println("Using VID source: " + String(hostConfig.getVidSource(), HEX));
    Serial.println("Using VID: " + String(hostConfig.getVid(), HEX));
    Serial.println("Using PID: " + String(hostConfig.getPid(), HEX));
    Serial.println("Using GUID version: " + String(hostConfig.getGuidVersion(), HEX));
    Serial.println("Using serial number: " + String(hostConfig.getSerialNumber()));
    */
    // Set up gamepad
    gamepad = new XboxGamepadDevice(config);
    compositeHID.addDevice(gamepad);

    // Start the composite HID device to broadcast HID reports
    //Serial.println("Starting composite HID device...");
    compositeHID.begin(hostConfig);
    gamepad->setRightThumb(-1, -1); //Initial unuse output

}

void SetDebouncePinMap(byte pins[], byte n){
    for(byte i=0; i < n; i++) {
      InpBTNStatus[i].attach(pins[i], INPUT);
      InpBTNStatus[i].interval(DebonceInterval);
    }
}

void SetPinMapTop(byte pins[], byte n){
    for(byte i=0; i < n; i++) {
      InpTopStatus[i].attach(pins[i], INPUT_PULLUP);
      InpTopStatus[i].interval(DebonceInterval);
    }
}
void SetPinMapMid(byte pins[], byte n){
    for(byte i=0; i < n; i++) {
      InpMidStatus[i].attach(pins[i], INPUT_PULLUP);
      InpMidStatus[i].interval(DebonceInterval);
    }
}
void SetPinMapBot(byte pins[], byte n){
    for(byte i=0; i < n; i++) {
      InpBotStatus[i].attach(pins[i], INPUT_PULLUP);
      InpBotStatus[i].interval(DebonceInterval);
    }
}

// Update Input status and set flag if check event occured
void UpdateInputStatus(){
  for(byte i=0; i < NumOfButtons; i++){
    InpBTNStatus[i].update();
    if (InpBTNStatus[i].changed()&& InputMask[MaskStatus][i]) {InpEventFlag = 1;}
  }
  for(byte i=0; i < 2; i++){
    InpTopStatus[i].update();
    InpMidStatus[i].update();
    InpBotStatus[i].update();
    if (InpTopStatus[i].changed()||InpMidStatus[i].changed()||InpBotStatus[i].changed()){
      InpEventFlag = 1;
      }
  }
};

void UpdateLTCount(bool Dir){// 0:plus count, 1: minus count
    int16_t Step = 8191; //int16 65536/8
    if(!Dir){
      if (LeftThumbCount < (XBOX_STICK_MAX - Step)){
        LeftThumbCount = LeftThumbCount + Step;
      }      
    }
    else{
      if (LeftThumbCount > (XBOX_STICK_MIN + Step)){
        LeftThumbCount = LeftThumbCount - Step;
      }
    }

}

void ControllerFunc(){
  UpdateInputStatus();
  if(InpEventFlag){
    //Buttons
    for(byte i=0; i < NumOfButtons; i++){
      if(InpBTNStatus[i].fell() && InputMask[MaskStatus][i]){   //Negative edge triggered
        gamepad->press(ButtonsOutput[i]);
      } 
      else if(InpBTNStatus[i].rose() && InputMask[MaskStatus][i]){  //Positive edge triggered
        gamepad->release(ButtonsOutput[i]);
      }
    }
    //Top Encoder
    if(!DpadReleaseNeedFlag && ((InpTopStatus[0].fell() && InpTopStatus[1].read()) || (InpTopStatus[1].rose() && InpTopStatus[0].read()))){   //A-phase Nagative edge triggered and B-phase high or B-phase Positive edge triggered and A-phase low
        gamepad->pressDPadDirectionFlag(directions[0][0]); //Top clockwise
        DpadTriggeredTime = millis();
        DpadReleaseNeedFlag = 1;
    }
    if(!DpadReleaseNeedFlag && ((InpTopStatus[1].fell() && InpTopStatus[0].read()) || (InpTopStatus[0].rose() && InpTopStatus[1].read()))){   //B-phase Nagative edge triggered and A-phase high or A-phase Positive edge triggered and B-phase low
        gamepad->pressDPadDirectionFlag(directions[0][1]); //Top counter-clockwise
        DpadTriggeredTime = millis();
        DpadReleaseNeedFlag = 1;
    }
    
    //Middle Encoder
    if(!DpadReleaseNeedFlag && ((InpMidStatus[0].fell() && InpMidStatus[1].read()) || (InpMidStatus[1].rose() && InpMidStatus[0].read()))){   //A-phase Nagative edge triggered and B-phase high or B-phase Positive edge triggered and A-phase low
        gamepad->pressDPadDirectionFlag(directions[1][0]); //Mid clockwise
        DpadTriggeredTime = millis();
        DpadReleaseNeedFlag = 1;
    }
    if(!DpadReleaseNeedFlag && ((InpMidStatus[1].fell() && InpMidStatus[0].read()) || (InpMidStatus[0].rose() && InpMidStatus[1].read()))){ //B-phase Nagative edge triggered and A-phase high or A-phase Positive edge triggered and B-phase low
        gamepad->pressDPadDirectionFlag(directions[1][1]); //Mid counter-clockwise
        DpadTriggeredTime = millis();
        DpadReleaseNeedFlag = 1;
    }
    
    //Bottom Encoder
    if(!DpadReleaseNeedFlag && ((InpBotStatus[0].fell() && InpBotStatus[1].read()) || (InpBotStatus[1].rose() && InpBotStatus[0].read()))){   //A-phase Nagative edge triggered and B-phase high or B-phase Positive edge triggered and A-phase low
        //gamepad->pressDPadDirectionFlag(directions[2][0]); //Top clockwise
        //DpadTriggeredTime = millis();
        //DpadReleaseNeedFlag = 1;
        UpdateLTCount(0); //0 for Plus count
        gamepad->setLeftThumb(StickOrigin, LeftThumbCount);
    }
    if(!DpadReleaseNeedFlag && ((InpBotStatus[1].fell() &&  InpBotStatus[0].read()) || (InpBotStatus[0].rose() && InpBotStatus[1].read()))){  //B-phase Nagative edge triggered and A-phase high or A-phase Positive edge triggered and B-phase low
        //gamepad->pressDPadDirectionFlag(directions[2][1]); //Top counter-clockwise
        //DpadTriggeredTime = millis();
        //DpadReleaseNeedFlag = 1;
        UpdateLTCount(1); //1 for Minus count
        gamepad->setLeftThumb(StickOrigin, LeftThumbCount);

    }
    
    gamepad->sendGamepadReport();
    InpEventFlag = 0;
  }
  else if(millis()-DpadTriggeredTime > DpadTriggeredInterval && DpadReleaseNeedFlag == 1){
    gamepad->releaseDPad();
    DpadReleaseNeedFlag = 0;

    gamepad->sendGamepadReport();
  }
}

void InstructionLED(unsigned long T){   //Base on Status flag to drive LED
  unsigned long Tmp = millis()-T;
  while (Tmp >= (InsInterval[Status][0]+InsInterval[Status][1])){
    Tmp = Tmp - InsInterval[Status][0] - InsInterval[Status][1];
  }
  if (Tmp < InsInterval[Status][0]){
    digitalWrite(InsLED, false); //Set LED bright
  }
  else{
    digitalWrite(InsLED, true); //Set LED off
  }

}

int GetBatteryFiltedValue(){
    int ReadValue[FilterFrame];
    //byte OverlimitCount = 0;
    byte DropIndex = FilterFrame;
    int mean, sum = 0;
    //sampling FilterFrame rows raw data and calculate mean
    for (byte i=0; i<FilterFrame;i++){
      ReadValue[i] = analogRead(BatSensor[0]);
      sum = sum + ReadValue[i];
    }
    mean = sum / FilterFrame;
    //delete 
    for (byte i=0; i<FilterFrame; i++){
      if (abs(ReadValue[i]-mean) > Filterlimit) {   // if there is Value over limit
          if (DropIndex != FilterFrame){   //check if DropIndex pointer legal (First overlimit may)
            if( abs(ReadValue[DropIndex]-mean)< abs(ReadValue[i]-mean)){  
              DropIndex = i;
              //OverlimitCount++;
            }
          }
          else{
            DropIndex = i;
           // OverlimitCount++;
          } 
      }
    }
    if(DropIndex != FilterFrame){
      mean = (sum - ReadValue[DropIndex]) / (FilterFrame-1);
    }
    return mean;
}



void UpdateBatteryStatus(){ //Estimate battery percentage and update
  int RawValue;
  RawValue = GetBatteryFiltedValue();
  Serial.println("Rawvalue = "+ String(RawValue));

  if (RawValue>BatteryLvTable[0][0]||RawValue>BatteryLvTable[0][1]) BatteryPercentage = 20;
  else if (RawValue>BatteryLvTable[1][0]||RawValue>BatteryLvTable[1][1]) BatteryPercentage = 40;
  else if (RawValue>BatteryLvTable[2][0]||RawValue>BatteryLvTable[2][1]) BatteryPercentage = 60;
  else if (RawValue>BatteryLvTable[3][0]||RawValue>BatteryLvTable[3][1]) BatteryPercentage = 80;
  else BatteryPercentage = 100;

  compositeHID.setBatteryLevel(BatteryPercentage);

}

void ErrorOccured(unsigned long T){ //Send serial messsage after each interval
  unsigned long Tmp = millis()-T;
  while (Tmp >= 2*ErrorReportInterval){ //remove complete period
    Tmp = Tmp - 2*ErrorReportInterval;
  }
  // Do something every ErrorReportInterval(ms)
  if (Tmp < ErrorReportInterval && MsgFlag){  
    //Serial.println("Connection is not built. Please connect Bluetooth device: <RemoteBox>");
    MsgFlag = 0;
  }
  else if (Tmp >= ErrorReportInterval && ~MsgFlag){
    //Serial.println("Connection is not built. Please connect Bluetooth device: <RemoteBox>");
    MsgFlag = 1;
  }
}

void UpdateStatus(byte EntryPoint){ //Base on EnrtyPoint update Working Status flag
  switch(EntryPoint){
    case 0:
      if (Status == 2) {Status = 0;}
      break;
    case 1:
      if (Status == 0 && BatteryPercentage < 30) {Status = 1;}
      break;
    case 2:
      if (Status !=2) {Status = 2;}
      break;
  }

}

void setup() {
    Serial.begin(115200);
    SetDebouncePinMap(physicBotton, sizeof(physicBotton));
    SetPinMapTop(TopEncoder, sizeof(TopEncoder));
    SetPinMapMid(MidEncoder, sizeof(MidEncoder));
    SetPinMapBot(BotEncoder, sizeof(BotEncoder));
    pinMode(InsLED, OUTPUT_OPEN_DRAIN);
    UpdateInputStatus();
    UpdateBatteryStatus();
    InpEventFlag = 0;   //Reset Event for setup step 
    Time = millis();    //Initial Time tag
    HIDInitial();
    delay(3000);
}

void loop() {
    if(compositeHID.isConnected()){
      ControllerFunc();
      UpdateStatus(0); //Normal working thread tag = 0 

      if(millis()-Time >= StatusUpdateInterval) { //
      UpdateBatteryStatus();
      UpdateStatus(1); //Battery check thread tag = 1
      Time = millis(); //Refresh Time tag
    }
    }
    else {
      ErrorOccured(Time);
      UpdateStatus(2); //Error thread tag = 2
    }
    InstructionLED(Time);
    
}