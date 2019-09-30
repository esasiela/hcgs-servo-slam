#include <Wire.h>

// #include <LiquidCrystal_I2C.h>

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>


#include <Servo.h>
#include <EEPROM.h>
#include <NeoSWSerial.h>

#include <HC_BouncyButton.h>

#include "HCGS_MidiOut.h"

// set this to write EEPROM with hardcoded test data in SLAM button onPress
//#define __HARDCODED_EEPROM_TEST_DATA__ 1

// set this to include a VIEW mode to intspect the EEPROM values
#define __VIEW_MODE__ 1

/*
   HC MIDI SysEx:
   0x12 address/data eeprom update
   0x13 servo position
   0x14 servo tap
*/


// LiquidCrystal_I2C lcd(0x27, 20, 4);
hd44780_I2Cexp lcd(0x27, 20, 4);


const int PIN_SERVO = 2;

// device select toggle switch
const int PIN_DEV_DN = 3;
const int PIN_DEV_UP = 4;

// motor select toggle switch
const int PIN_MOT_DN = 5;
const int PIN_MOT_UP = 6;

// MIDI rx not connected, MIDI Out is tx only, but software serial requires a valid rx pin
const int PIN_MIDI_TX = 7;
const int PIN_MIDI_RX = 13;

// connected to HC ISP ECHO default
const int PIN_CONFIG_TX = 12;
const int PIN_CONFIG_RX = 11;

const int PIN_MODE_BUTTON = 8;
const int PIN_SLAM_BUTTON = 9;
const int PIN_SAVE_BUTTON = 10;

const int PIN_POT_UTIL = A0;
const int PIN_POT_STRIKE = A1;
const int PIN_POT_DELAY = A2;
const int PIN_POT_REST = A3;


NeoSWSerial midiSerial(PIN_MIDI_RX, PIN_MIDI_TX); // Rx/Tx

NeoSWSerial configSerial(PIN_CONFIG_RX, PIN_CONFIG_TX); // rx/tx

HcgsMidiOut midiOut(&midiSerial);

BouncyButton btnMode = BouncyButton(PIN_MODE_BUTTON);
BouncyButton btnSlam = BouncyButton(PIN_SLAM_BUTTON);
BouncyButton btnSave = BouncyButton(PIN_SAVE_BUTTON);

BouncyButton btnDevUp = BouncyButton(PIN_DEV_UP);
BouncyButton btnDevDn = BouncyButton(PIN_DEV_DN);
BouncyButton btnMotUp = BouncyButton(PIN_MOT_UP);
BouncyButton btnMotDn = BouncyButton(PIN_MOT_DN);


Servo tap;

unsigned long lastDisplayMillis = 0;
const int DISPLAY_DURATION_MILLIS = 100;

unsigned long lastServoMillis = 0;
const int SERVO_DURATION_MILLIS = 100;

unsigned long lastPotMillis = 0;
const int POT_DURATION_MILLIS = 50;

unsigned long lastLoopMillis = 0;

unsigned long loopBlinkMillis = 0;
const int LOOP_BLINK_DURATION_MILLIS = 150;


const int EEPROM_ADDR_START = 20;

const int SYSMODE_SLAM = 0;
const int SYSMODE_LOOP = 1;
const int SYSMODE_SPIN = 2;
const int SYSMODE_KNOB = 3;

#ifdef __VIEW_MODE__
const int SYSMODE_VIEW = 4;
#endif

const int SYSMODE_DEFAULT = SYSMODE_SLAM;
const int SYSMODE_MIN = SYSMODE_SLAM;

#ifdef __VIEW_MODE__
const int SYSMODE_MAX = SYSMODE_VIEW;

int viewAddr = EEPROM_ADDR_START;

#else
const int SYSMODE_MAX = SYSMODE_KNOB;
#endif

uint8_t sysMode = SYSMODE_DEFAULT;

bool sysArmed = false;

unsigned long nowMillis = 0;


int potPosUtil = 0;
int potPosStrike = 0;
int potPosDelay = 0;
int potPosRest = 0;

int potPrevUtil = 0;
int potPrevStrike = 0;
int potPrevDelay = 0;
int potPrevRest = 0;





// signed values because we might wrap when they hit the toggle button, gets corrected in the setXXX() functions
int8_t deviceIndex = 0;
int8_t motorIndex = 0;

char deviceName[17];
char motorName[17];


uint8_t mapServoPot(int potPos) {
  return map(potPos, 0, 1023, 0, 180);
}

uint16_t mapDelayPot(int potPos) {
  // allow a delay from 0-1 sec
  return map(potPos, 0, 1023, 0, 1000);
}

uint16_t mapTempoPot(int potPos) {
  // allow BPM 1-500
  return map(potPos, 0, 1023, 1, 300);
}



void readPots() {
  potPrevUtil = potPosUtil;
  potPosUtil = analogRead(PIN_POT_UTIL);

  potPrevStrike = potPosStrike;
  potPosStrike = analogRead(PIN_POT_STRIKE);

  potPrevDelay = potPosDelay;
  potPosDelay = analogRead(PIN_POT_DELAY);

  potPrevRest = potPosRest;
  potPosRest = analogRead(PIN_POT_REST);
}


void clearSpin() {
  clearKnob();
}

void displaySpin() {
  clearSpin();

  if (sysMode == SYSMODE_SPIN) {
    displayFourDigits(mapServoPot(potPosUtil), 0, 3);
  }
}

void clearLoop() {
  clearSlam();
}

void displayLoop(boolean fullWipe = true) {
  if (fullWipe) {
    clearLoop();
  }

  if (sysMode == SYSMODE_LOOP) {

    if (fullWipe || mapTempoPot(potPosUtil) != mapTempoPot(potPrevUtil)) {
      displayFourDigits(mapTempoPot(potPosUtil), 0, 3);
    }

    if (fullWipe || mapServoPot(potPosStrike) != mapServoPot(potPrevStrike)) {
      displayFourDigits(mapServoPot(potPosStrike), 5, 3);
    }

    if (fullWipe || mapDelayPot(potPosDelay) != mapDelayPot(potPrevDelay)) {
      displayFourDigits(mapDelayPot(potPosDelay), 10, 3);
    }

    if (fullWipe || mapDelayPot(potPosRest) != mapDelayPot(potPrevRest)) {
      displayFourDigits(mapServoPot(potPosRest), 15, 3);
    }
  }
}

void clearSlam() {
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
}

void displaySlam(boolean fullWipe = true) {
  if (fullWipe) {
    clearSlam();
  }

  if (sysMode == SYSMODE_SLAM) {
    if (fullWipe || potPosStrike != potPrevStrike) {
      displayFourDigits(mapServoPot(potPosStrike), 5, 3);
    }

    if (fullWipe || potPosDelay != potPrevDelay) {
      displayFourDigits(mapDelayPot(potPosDelay), 10, 3);
    }

    if (fullWipe || potPosRest != potPrevRest) {
      displayFourDigits(mapServoPot(potPosRest), 15, 3);
    }
  }
}

void clearKnob() {
  //lcd.setCursor(0, 2);
  //lcd.print(F("    "));
  lcd.setCursor(0, 3);
  lcd.print(F("    "));
}

void displayKnob() {
  clearKnob();

  if (sysMode == SYSMODE_KNOB) {
    //displayFourDigits(potPosUtil, 0, 2);
    displayFourDigits(mapServoPot(potPosUtil), 0, 3);
  }
}

void displayAction(const char* action = "    ") {
  lcd.setCursor(0, 0);
  lcd.print(action);
}

void displayFourDigits(uint16_t val, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.print(F("    "));
  lcd.setCursor(col, row);
  lcd.print(val);
}

void displayDevice(const char* name = "                ") {
  lcd.setCursor(4, 1);
  lcd.print(name);
}

void displayMotor(const char* name = "                ") {
  lcd.setCursor(4, 2);
  lcd.print(name);
}

#ifdef __VIEW_MODE__
void clearView() {
  clearLoop();
}

void displayView() {
  clearView();

  if (sysMode == SYSMODE_VIEW) {
    lcd.setCursor(0, 3);
    lcd.print(F("ADDR="));
    lcd.print(viewAddr, DEC);
    lcd.setCursor(8, 3);
    lcd.print(F(" VAL="));
    uint8_t val = EEPROM.read(viewAddr);
    if (val < 0x10) {
      lcd.print(F("0"));
    }
    lcd.print(val, HEX);

    // printable ascii
    if (val > 31 && val < 127) {
      lcd.print(F(" ["));
      lcd.print((char)val);
      lcd.print(F("]"));
    }
  }
}
#endif


void displayUpdate() {

  lcd.setCursor(5, 0);
  if (sysArmed) {
    lcd.print(F("ARMED "));
  } else {
    lcd.print(F("DISRM "));
  }

  //  lcd.setCursor(11, 0);
  lcd.print(F("MODE="));
  switch (sysMode) {
    case SYSMODE_SLAM :
      lcd.print(F("SLAM"));
      break;
    case SYSMODE_LOOP :
      lcd.print(F("LOOP"));
      break;
    case SYSMODE_SPIN :
      lcd.print(F("SPIN"));
      break;
    case SYSMODE_KNOB :
      lcd.print(F("KNOB"));
      break;
#ifdef __VIEW_MODE__
    case SYSMODE_VIEW :
      lcd.print(F("VIEW"));
      break;
#endif
    default :
      lcd.print(F("????"));
      break;
  }


  if (sysMode == SYSMODE_KNOB) {
    lcd.setCursor(0, 1);
    lcd.print(F("    "));
    lcd.setCursor(0, 2);
    lcd.print(F("    "));

    displayDevice();
    displayMotor();
  } else {
    lcd.setCursor(0, 1);
    lcd.print(F("DEV="));
    lcd.setCursor(0, 2);
    lcd.print(F("MOT="));

    displayDevice(deviceName);
    displayMotor(motorName);
  }
}



void modeStop(uint8_t stoppingMode) {

  sysArmed = false;

  if (stoppingMode == SYSMODE_SLAM) {
    clearSlam();

  } else if (stoppingMode == SYSMODE_LOOP) {
    clearLoop();

  } else if (stoppingMode == SYSMODE_SPIN) {
    clearSpin();

  } else if (stoppingMode == SYSMODE_KNOB) {
    tap.detach();
    clearKnob();
#ifdef __VIEW_MODE__
  } else if (stoppingMode == SYSMODE_VIEW) {
    clearView();
#endif
  }

}

void modeStart(uint8_t startingMode) {

  if (startingMode == SYSMODE_SLAM) {
    displaySlam();

  } else if (startingMode == SYSMODE_LOOP) {
    displayLoop();

  } else if (startingMode == SYSMODE_SPIN) {
    displaySpin();

  } else if (startingMode == SYSMODE_KNOB) {
    tap.attach(PIN_SERVO);
    displayKnob();
#ifdef __VIEW_MODE__
  } else if (startingMode == SYSMODE_VIEW) {
    viewAddr = EEPROM_ADDR_START;
    displayView();
#endif
  }
}


int getDeviceStartAddr() {
  int addr = EEPROM_ADDR_START;
  for (int x = 0; x < deviceIndex; x++) {
    addr += EEPROM.read(addr);
  }
  return addr;
}

int getMotorStartAddr() {

  int addr = 0;

  // not really a helpful situation if motor count is 0, but the display will handle that elsewhere.

  if (getMotorCount() > 0) {
    addr = getDeviceStartAddr() + 3;

    // first, need to wade through the device name
    addr += EEPROM.read(addr);

    for (uint8_t idx = 0; idx < motorIndex; idx++) {
      addr += EEPROM.read(addr);
    }
  }
  return addr;
}


uint8_t getDeviceID() {
  // this is different than index, it is a data attribute for the device that is indicated by deviceIndex
  return EEPROM.read(getDeviceStartAddr() + 1);
}

uint8_t getModelID() {
  return EEPROM.read(getDeviceStartAddr() + 2);
}

uint8_t getMotorID() {
  return EEPROM.read(getMotorStartAddr() + 1);
}

int8_t getDeviceCount() {

  uint8_t count = 0;
  uint8_t len = 0;

  // start at the beginning
  int addr = EEPROM_ADDR_START;

  // read the device size, if zero then we're done
  while ((len = EEPROM.read(addr)) != 0x00) {
    // if non-zero, increment count and advance to next addr
    count++;
    addr += len;
  }

  return count;
}

int8_t getMotorCount() {
  // get the number of bytes in the device
  int startAddr = getDeviceStartAddr();
  uint8_t devSize = EEPROM.read(startAddr);

  // consume devId, modelId, and devName
  int addr = startAddr + 3;
  addr += EEPROM.read(addr);

  uint8_t motorCount = 0;

  while ((addr - startAddr) < devSize) {
    motorCount++;
    addr += EEPROM.read(addr);
  }

  return motorCount;
}

// signed value, because they might wrap to negative, limits us to 127 devices
void setDevice(int8_t newIndex) {
  deviceIndex = newIndex;
  int8_t maxIndex = getDeviceCount() - 1;

  if (deviceIndex < 0) {
    deviceIndex = maxIndex;
  } else if (deviceIndex > maxIndex) {
    deviceIndex = 0;
  }

  // set the device name
  uint8_t startAddr = getDeviceStartAddr() + 3;
  uint8_t len = EEPROM.read(startAddr++) - 1;
  uint8_t idx = 0;
  for (idx = 0; idx < len; idx++) {
    deviceName[idx] = EEPROM.read(startAddr + idx);
  }
  deviceName[idx] = '\0';

  // update the display
  displayDevice();

  // reset the motor to 0
  setMotor(0);
}

// signed value, because they might wrap to negative, limits us to 127 motors per device
void setMotor(int8_t newIndex) {
  motorIndex = newIndex;
  int8_t maxIndex = getMotorCount() - 1;

  if (motorIndex < 0) {
    motorIndex = maxIndex;
  } else if (motorIndex > maxIndex) {
    motorIndex = 0;
  }

  // set the motor name
  uint8_t startAddr = getMotorStartAddr() + 2;
  uint8_t len = EEPROM.read(startAddr++) - 1;
  uint8_t idx = 0;
  for (idx = 0; idx < len; idx++) {
    motorName[idx] = EEPROM.read(startAddr + idx);
  }
  motorName[idx] = '\0';

  // update the display
  displayMotor();
}


void sendSysEx14() {
  // uint8_t deviceId, uint8_t modelId, uint8_t instructionId, const byte* rawData, uint8_t rawLen
  const uint8_t sysExLen = 5;
  uint16_t delayVal = mapDelayPot(potPosDelay);
  byte sysExData[sysExLen] = { motorIndex, mapServoPot(potPosStrike), highByte(delayVal), lowByte(delayVal), mapServoPot(potPosRest) };
  midiOut.sendSysEx(getDeviceID(), getModelID(), 0x14, sysExData, sysExLen);
}

void sendSysEx13() {
  // uint8_t deviceId, uint8_t modelId, uint8_t instructionId, const byte* rawData, uint8_t rawLen
  const uint8_t sysExLen = 2;
  byte sysExData[sysExLen] = { motorIndex, mapServoPot(potPosUtil) };
  midiOut.sendSysEx(getDeviceID(), getModelID(), 0x13, sysExData, sysExLen);
}


/*************************************************************
   setup()
 *************************************************************/
void setup() {

  configSerial.begin(19200);
  configSerial.println("hcgs-servo-slam");

  midiSerial.begin(31250);


  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.home();

  btnMode.init();
  btnSlam.init();
  btnSave.init();

  btnDevUp.init();
  btnDevDn.init();
  btnMotUp.init();
  btnMotDn.init();

  // give the LCD a moment to wake up before going into the loop
  delay(1);

  readPots();

  sysMode = SYSMODE_DEFAULT;
  modeStart(SYSMODE_DEFAULT);

  setDevice(0);
}





/*************************************************************
   loop()
 *************************************************************/
void loop() {
  nowMillis = millis();



  /***
     Check for configuration data on the serial port
  */

  if (configSerial.available()) {

    // disarm any MIDI transmissions
    sysArmed = false;


    displayAction("DATA");

    // start at the beginning of the EEPROM address space
    int eepromPointer = EEPROM_ADDR_START;

    uint8_t dataSize = 0;
    while ((dataSize = configSerial.read()) != 0) {
      // read a single byte (value is SIZE of next write)
      uint8_t dataSize = configSerial.read();

      // save to EEPROM
      EEPROM.write(eepromPointer++, dataSize);

      // respond ready
      configSerial.print(0x01);

      // read SIZE bytes
      while (configSerial.available() < dataSize) {

      }

      // save to EEPROM
      for (int i = 0; i < dataSize; i++) {
        EEPROM.write(eepromPointer++, configSerial.read());
      }

      // respond ready
      configSerial.print(0x01);
    }

    // received a 0x00 byte as the dataSize, so exited loop, write a final 0x00 to EEPROM to mark EOD
    EEPROM.write(eepromPointer, 0x00);

    // reset device/motor to beginnning
    setDevice(0);

    displayAction();
  }




  if (btnMode.update()) {

    if (!btnMode.getState()) {
      displayAction("MODE");

      uint8_t oldMode = sysMode++;
      if (sysMode > SYSMODE_MAX) {
        sysMode = SYSMODE_MIN;
      }

      // stop the current mode
      modeStop(oldMode);

      // start the new mode
      modeStart(sysMode);

    } else {
      displayAction();
    }

  }

  if (btnSlam.update()) {

#ifdef __HARDCODED_EEPROM_TEST_DATA__
    uint16_t p = EEPROM_ADDR_START;

    /******************************
       TEST DEVICE 1
     ******************************/

    // device data size (26, incl size byte)
    EEPROM.write(p++, 0x1A); // 20

    // device id
    EEPROM.write(p++, 0x01); // 21

    // model id
    EEPROM.write(p++, 0x00); // 22

    // device name size (incl size byte)
    EEPROM.write(p++, 0x07); // 23

    // device name bytes
    EEPROM.write(p++, 'f'); // 24
    EEPROM.write(p++, 'r'); // 25
    EEPROM.write(p++, 'o'); // 26
    EEPROM.write(p++, 'w'); // 27
    EEPROM.write(p++, 'n'); // 28
    EEPROM.write(p++, 'y'); // 29

    // motor data size (incl size byte)
    EEPROM.write(p++, 0x06); // 30

    // motor name
    EEPROM.write(p++, 'r'); // 31
    EEPROM.write(p++, 'i'); // 32
    EEPROM.write(p++, 'g'); // 33
    EEPROM.write(p++, 'h'); // 34
    EEPROM.write(p++, 't'); // 35

    // motor data size (incl size byte)
    EEPROM.write(p++, 0x05); // 36

    // motor name
    EEPROM.write(p++, 'l'); // 37
    EEPROM.write(p++, 'e'); // 38
    EEPROM.write(p++, 'f'); // 39
    EEPROM.write(p++, 't'); // 40

    // motor data size (incl size byte)
    EEPROM.write(p++, 0x05); // 41

    // motor name
    EEPROM.write(p++, 't'); // 42
    EEPROM.write(p++, 'a'); // 43
    EEPROM.write(p++, 'i'); // 44
    EEPROM.write(p++, 'l'); // 45

    /******************************
       TEST DEVICE 2
     ******************************/

    // device data size (20, incl size byte)
    EEPROM.write(p++, 0x14); // 46

    // device id
    EEPROM.write(p++, 0x05); // 47

    // model id
    EEPROM.write(p++, 0x01); // 48

    // device name size (incl size byte)
    EEPROM.write(p++, 0x08); // 49

    // device name bytes
    EEPROM.write(p++, 't'); // 50
    EEPROM.write(p++, 'a'); // 51
    EEPROM.write(p++, 'p'); // 52
    EEPROM.write(p++, 'i'); // 53
    EEPROM.write(p++, 'o'); // 54
    EEPROM.write(p++, 'c'); // 55
    EEPROM.write(p++, 'a'); // 56

    // motor data size (incl size byte)
    EEPROM.write(p++, 0x05); // 57

    // motor name
    EEPROM.write(p++, 'g'); // 58
    EEPROM.write(p++, 'o'); // 59
    EEPROM.write(p++, 'a'); // 60
    EEPROM.write(p++, 't'); // 61

    // motor data size (incl size byte)
    EEPROM.write(p++, 0x04); // 62

    // motor name
    EEPROM.write(p++, 'r'); // 63
    EEPROM.write(p++, 'a'); // 64
    EEPROM.write(p++, 'm'); // 65

    /******************************
       TERMINATOR
     ******************************/

    // finish it off with 0x00
    EEPROM.write(p++, 0x00); // 66
#endif



    if (!btnSlam.getState()) {
      if (sysMode == SYSMODE_SLAM) {
        displayAction("SLAM");

        sendSysEx14();

      } else if (sysMode == SYSMODE_LOOP || sysMode == SYSMODE_SPIN || sysMode == SYSMODE_KNOB) {
        sysArmed = !sysArmed;
        displayUpdate();

#ifdef __VIEW_MODE__
      } else if (sysMode == SYSMODE_VIEW) {
        viewAddr++;
        displayView();
#endif
      }

    } else {
      displayAction();
    }
  }


  if (btnSave.update()) {

    if (!btnSave.getState()) {
      displayAction("SAVE");

      // TODO if mode is loop or slam form one sysex, if mode is spin do another

    } else {
      displayAction();
    }
  }

  if (btnDevUp.update()) {

    if (!btnDevUp.getState()) {
      displayAction("DEVU");
      sysArmed = false;
      setDevice(deviceIndex + 1);
    } else {
      displayAction();
    }
  }

  if (btnDevDn.update()) {

    if (!btnDevDn.getState()) {
      displayAction("DEVD");
      sysArmed = false;
      setDevice(deviceIndex - 1);
    } else {
      displayAction();
    }
  }


  if (btnMotUp.update()) {

    if (!btnMotUp.getState()) {
      displayAction("MOTU");
      sysArmed = false;
      setMotor(motorIndex + 1);
    } else {
      displayAction();
    }
  }

  if (btnMotDn.update()) {

    if (!btnMotDn.getState()) {
      displayAction("MOTD");
      sysArmed = false;
      setMotor(motorIndex - 1);
    } else {
      displayAction();
    }
  }



  if (sysMode == SYSMODE_SLAM || sysMode == SYSMODE_LOOP) {
    if ((nowMillis - lastServoMillis) > SERVO_DURATION_MILLIS) {
      lastServoMillis = nowMillis;

      uint8_t mappedStrikeVal = mapServoPot(potPosStrike);
      uint16_t mappedDelayVal = mapDelayPot(potPosDelay);
      uint8_t mappedRestVal = mapServoPot(potPosRest);
      uint16_t mappedTempoVal = mapTempoPot(potPosUtil);

      if (sysMode == SYSMODE_SLAM) {
        displaySlam(false);
      } else {
        displayLoop(false);
      }
    }

  } else if (sysMode == SYSMODE_KNOB || sysMode == SYSMODE_SPIN) {

    if ((nowMillis - lastServoMillis) >= SERVO_DURATION_MILLIS) {
      lastServoMillis = nowMillis;

      uint8_t mappedServoVal = mapServoPot(potPosUtil);

      if (mappedServoVal != mapServoPot(potPrevUtil)) {
        if (sysMode == SYSMODE_KNOB) {
          displayKnob();
        } else {
          displaySpin();
        }

        if (sysArmed) {
          if (sysMode == SYSMODE_KNOB) {
            tap.write(mappedServoVal);
          } else {
            sendSysEx13();
          }
        }
      }
    }
  }


  if (sysMode == SYSMODE_LOOP) {

    if ((loopBlinkMillis > 0) && ((nowMillis - loopBlinkMillis) >= LOOP_BLINK_DURATION_MILLIS)) {
      loopBlinkMillis = 0;
      lcd.setCursor(19, 3);
      lcd.print(F(" "));
    }


    if (sysArmed) {
      // UTIL has a BPM value.  How many millis between notes?
      if ((nowMillis - lastLoopMillis) >= (60000 / mapTempoPot(potPosUtil))) {
        lastLoopMillis = nowMillis;


        lcd.setCursor(19, 3);
        lcd.print(F("X"));
        loopBlinkMillis = nowMillis;

        sendSysEx14();
      }
    }

  }


  if ((nowMillis - lastPotMillis) >= POT_DURATION_MILLIS) {
    lastPotMillis = nowMillis;

    readPots();
  }

  if ((nowMillis - lastDisplayMillis) >= DISPLAY_DURATION_MILLIS) {
    lastDisplayMillis = nowMillis;

    displayUpdate();
  }
}
