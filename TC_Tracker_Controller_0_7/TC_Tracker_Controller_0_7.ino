/* Tracker Controller-  Seeed Xiao nRf52840 and/or nRF52840 Sense    Charles Hobbs 2023
Gamepad controller can be configured as a USB or BLE HID device :
gamepad, keyboard... Midi Controller or Midi pass-thru device for DIN, BLE and USB Midi: Converter/Merge Hub, midi filter, etc
Presets selected by holding a button on startup.    
Uses the Xiao's built in battery management to charge and monitor LiPo battery.   
Red Warning LED will flash when battery is low and not connected to USB or external power.
Uses nRf52840's low power modes when no use is detected.  Sleep and Standby times can be customized.  
Press "User" button to wake from Sleep... 
Waking from sleep triggers a full reset; you will need to hold down the appropriate button to wake and select the desired (non default) preset.   

resources:
battery reading: https://www.unmannedtechshop.co.uk/product/seeed-xiao-ble-nrf52840/
detect ext Power: https://forum.seeedstudio.com/t/how-to-detect-disconnection-from-usb-and-how-to-keep-the-radio-active-on-battery/279388/10
*/

#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <Arduino.h>
#include <Adafruit_FlashTransport.h>
Adafruit_FlashTransport_QSPI flashTransport;

Adafruit_USBD_MIDI usb_midi;

BLEDis bledis;
BLEHidGamepad blegamepad;
BLEHidAdafruit blekeyboard;
BLEMidi blemidi;


struct MySettings : public midi::DefaultSettings {
  static const unsigned SysExMaxSize = 1026;  // Accept SysEx messages up to 256 bytes long.
};

MIDI_CREATE_BLE_INSTANCE(blemidi);
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, USBMIDI);
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, DINMIDI, MySettings);
//MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, usb_midi, USBMIDI, MySettings);
//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, DINMIDI);

uint8_t const desc_gamepad_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};
uint8_t const desc_keyboard_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD()
};

Adafruit_USBD_HID usb_gamepad(desc_gamepad_report, sizeof(desc_gamepad_report), HID_ITF_PROTOCOL_NONE, 2, false);
Adafruit_USBD_HID usb_keyboard(desc_keyboard_report, sizeof(desc_keyboard_report), HID_ITF_PROTOCOL_KEYBOARD, 2, false);

//pin mapping
byte a = D0;
byte b = D1;
byte up = D2;
byte right = D3;
byte down = D4;
byte left = D5;
byte starT = D8;
byte selecT = D9;
byte User = D10;

uint8_t button[8] = { a, b, up, right, down, left, starT, selecT };
int previousPreset;


///////////////////////////////User Config VariableS //////////////////////////////////////////////////
#define Batt true              //do you have a battery?
bool USBsleep = true;          //do you want to sleep when connected to usb and BT or only BT?  false= never sleep when USB is connected (and mounted)
bool ChargingSleep = true;     //do you want to sleep when charging?
int defaultPreset = 0;         //default preset 0 = 1st preset, 1 = 2nd, etc
uint8_t SleepTime = 2;         //minutes of inactivity before sleep
uint8_t StandbyTime = 1;       //minutes of inactivity before BLE standby
int debounceTime = 20;         //switch debounce/ delay in reading
bool ThruToBLE = false;            //default setting for sending midi thru(usb & DIN) to BT.  True = on ,false = off (can be changed durring use w/ double click of the user button.)  ( Sending thru to BLE midi can cause sever lag on DIN and USB! better to leave it off and just turn t on when you need it! BLE midi still can be sent as a controller)
int DoubleClickTimeOut = 750;  //max time for detecting a double click on the User button

///////////////////Preset/Device Config/////////////////

//preset controller type:  joystick,keyboard, or midi?   gamepad = 0, keyboard = 1, midi controller= 2
//deviceMode[number of presets] {preset 0, preset 1, preset 2...}
byte deviceMode[7] = { 0, 1, 1, 2, 2, 2, 0 };  //deviceMode[8] = {0, 1, 1, 2, 2, 2, 0, 0 }; for 8 presets, etc


//7 rows, 8 columns   row=preset;  column = command for button or key to be sent.  for midi, command equals note or CC #
uint8_t command[7][8] = {
  { 1, 0, 12, 15, 13, 14, 9, 8 },                                                                                                              //preset 0   generic NES-style  game controller for M8, LSDJ, LPGT, etc
  { HID_KEY_S, HID_KEY_A, HID_KEY_ARROW_UP, HID_KEY_ARROW_RIGHT, HID_KEY_ARROW_DOWN, HID_KEY_ARROW_LEFT, HID_KEY_X, HID_KEY_Z },               //preset 1   M8C keyboard mapping
  { HID_KEY_X, HID_KEY_Z, HID_KEY_ARROW_UP, HID_KEY_ARROW_RIGHT, HID_KEY_ARROW_DOWN, HID_KEY_ARROW_LEFT, HID_KEY_SPACE, HID_KEY_SHIFT_LEFT },  //preset 2   M8.run keyboard mapping
  { 12, 24, 36, 48, 56, 40, 30, 54 },                                                                                                          //preset 3  //demo of possible midi options  (cc,note, program change, and transport (start/stop)
  { 14, 13, 12, 17, 16, 15, 19, 18 },                                                                                                          //preset 4 = midi m8 mute track
  { 22, 21, 20, 25, 24, 23, 27, 26 },                                                                                                          //preset 5 = midi m8 solo track
  { 1, 0, 12, 15, 13, 14, 9, 8 },                                                                                                              //preset 6
};

/*
extra layouts:
M8.run keyboard mapping:
command:    { HID_KEY_X, HID_KEY_Z, HID_KEY_ARROW_UP, HID_KEY_ARROW_RIGHT, HID_KEY_ARROW_DOWN, HID_KEY_ARROW_LEFT, HID_KEY_SPACE, HID_KEY_SHIFT_LEFT },  //  M8.run keyboard mapping

midi demo :
command:    { 12, 24, 36, 48, 56, 40, 30, 54 }
midiType:    { 1, 1, 1, 1, 0, 0, 3, 2 }
midiChannel:   { 3, 2, 1, 4, 6, 5, 8, 7 }
midiMaxValue:    { 10, 100, 0, 30, 0, 20, 0, 0 }
midiMinValue:    { 1, 50, 0, 10, 0, 0, 0, 0 }

*/

//gamepad options
byte DPad[7]{ 0, 0, 0, 0, 0, 0, 1 };  //Some devices/programs expect a gamepad to send Dpad/hat commands (min/max analog values) for the directional buttons, rather than button press commands (boolean).   To send Dpad/hat commands, put a 1 in the position that correspond to your gamepad preset

//keyboard options

//complete list of all key and modifier defininitions = https://github.com/hathach/tinyusb/blob/master/src/class/hid/hid.h
uint8_t keyModifier[7][8] = {
  //keyboard modifier for key combos- leave 0 if not used
  { 0, 0, 0, 0, 0, 0, 0, 0 },                                                      //preset 0 = gamepad
  { KEYBOARD_MODIFIER_LEFTSHIFT, 0, 0, 0, 0, 0, 0, KEYBOARD_MODIFIER_LEFTSHIFT },  //preset 1 = keyboard m8c
  { 0, 0, 0, 0, 0, 0, 0, 0 },                                                      //preset 2 = keyboard
  { 0, 0, 0, 0, 0, 0, 0, 0 },                                                      //preset 3 = demo midi preset
  { 0, 0, 0, 0, 0, 0, 0, 0 },                                                      //preset 4 = midi m8 mute track - all noteon/off
  { 0, 0, 0, 0, 0, 0, 0, 0 },                                                      //preset 5 = midi m8 solo track - all noteon/off
  { 0, 0, 0, 0, 0, 0, 0, 0 }                                                       //preset 6 =gamepad w/dpad commmands
};

uint8_t keyModifier2[7][8];

//midi controller options

//Type of medi message to be sent   0= noteOn/noteoff, 1= CC, 2= program Change, 3= Clock Start/Stop // leave 0 for gamepad or keyboard
uint8_t midiType[7][8] = {
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 0 = gamepad
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 1 = keyboard
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 2 = keyboard
  { 1, 1, 1, 1, 0, 0, 3, 2 },  //preset 3 = demo midi preset
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 4 = midi m8 mute track - all noteon/off
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 5 = midi m8 solo track - all noteon/off
  { 0, 0, 0, 0, 0, 0, 0, 0 }   //preset 6 = gamepad w/dpad commmands
};



//Arrays for sending Midi messages: channel, command, value
//if using midi, put the data in the correspond row per preset. 1-16.  preset 4 is midi?  put the midi info in the 4th row.  leave 0 for gamepad or keyboard
uint8_t midiChannel[7][8] = {
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 0
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 3, 2, 1, 4, 6, 5, 8, 7 },          //preset 3 =demo midi preset
  { 10, 10, 10, 10, 10, 10, 10, 10 },  //preset 4 = midi m8 mute track   control map channel 10
  { 10, 10, 10, 10, 10, 10, 10, 10 },  //preset 5 = midi m8 solo track   control map channel 10
  { 0, 0, 0, 0, 0, 0, 0, 0 }
};

//Value for specifing a specific non-127 velocity or cc value when the button is pressed.  0-127.  will send 127 for the max value if left 0  -  can be left at 0 otherwise
uint8_t midiMaxValue[7][8] = {
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 0
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 10, 64, 0, 30, 0, 20, 0, 0 },  //preset 3 = demo midi preset
  { 0, 0, 0, 0, 0, 0, 0, 0 },      //preset 4 = midi m8 mute track  no specific velocity command necessary
  { 0, 0, 0, 0, 0, 0, 0, 0 },      //preset 5 = midi m8 solo track, no specific velocity command necessary
  { 0, 0, 0, 0, 0, 0, 0, 0 }
};

//Value for specifing a specific non-0 cc value when the button is released.  0-127.    will send 0 value if left 0  -  can be left at 0 otherwise
uint8_t midiMinValue[7][8] = {
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 0
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 125, 0, 10, 0, 0, 0, 0 },  //preset 3 = demo midi preset
  { 0, 0, 0, 0, 0, 0, 0, 0 },     //preset 4 = midi m8 mute track  no specific velocity command necessary
  { 0, 0, 0, 0, 0, 0, 0, 0 },     //preset 5 = midi m8 solo track, no specific velocity command necessary
  { 0, 0, 0, 0, 0, 0, 0, 0 }
};

uint8_t midiToggle[7][8] = {   // toggle mode for for note and cc messages.    0 = not toggle- - command sent on press and release.  1= Toggle- commands alternate (E.g., 0,127 or noteOn, noteOff) with each press
  { 0, 0, 0, 0, 0, 0, 0, 0 },  //preset 0
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 1, 0, 0, 0, 0, 0, 0 },  //preset 3 = demo midi preset
  { 1, 1, 1, 1, 1, 1, 1, 1 },  //preset 4 = midi m8 mute track. Toggle on for all keys
  { 1, 1, 1, 1, 1, 1, 1, 1 },  //preset 5 = midi m8 solo track. Toggle on for all keys
  { 0, 0, 0, 0, 0, 0, 0, 0 }
};
///////// end Preset Config//////////////////////////////////////

uint8_t midiToggleState[7][8];  //blank array for saving toggle states when in use. do not change- not a user config
int preset = defaultPreset;     //do not change- not a user config

////// preset selection button assignment ////////////////////////

void selectPreset() {
  // set a preset by holding a button at startup
  previousPreset = preset;
  if (digitalRead(selecT) == LOW) {  //hold a button to select a preset
    preset = 0;                      // gamepad
  } else if (digitalRead(b) == LOW) {
    preset = 1;  //keyboard m8c
  } else if (digitalRead(a) == LOW) {
    preset = 2;  // keyboard m8.run
  } else if (digitalRead(up) == LOW) {
    preset = 0;  // gamepad
  } else if (digitalRead(right) == LOW) {
    preset = 3;  // midi controller
  } else if (digitalRead(down) == LOW) {
    preset = 4;  // midi controller
  } else if (digitalRead(left) == LOW) {
    preset = 5;  // midi controller
  } else if (digitalRead(starT) == LOW) {
    preset = 6;  // dhat
  } else
    preset = previousPreset;
}

////end user preset config/////////////////////////////////////////


hid_gamepad_report_t gp;
bool activeState = false;
int buttonState[8] = {};
int previousButtonState[8] = { HIGH };
long lastActivity = 0;
static bool playing = false;
long lastSwitchTime;
long UserButtonCount;
int UserbuttonState;
int previousUserButtonState = HIGH;


int ClickCount = 0;
long ButtonDoubleClickTime;
long ButtonHoldTime;
bool StandbyOn = 0;

////

#define BAT_READ 14                     // P0_14 = 14  Reads battery voltage from divider on signal board. (PIN_VBAT is reading voltage divider on XIAO and is program pin 32 / or P0.31)
#define CHARGE_LED 23                   // P0_17 = 17  D23   YELLOW CHARGE LED
#define HICHG 22                        // P0_13 = 13  D22   Charge-select pin for Lipo for 100 mA instead of default 50mA charge
const double vRef = 3.3;                // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024;  // 10-bit ADC readings 0-1023, so the factor is 1024
volatile int BAT_CHRG;                  //is the battery connected and charging?   5V connected/charging = 0; disconnected = 1
double vBat;                            //battery voltage

long lastBatteryCheck;
bool ExtPower = 0;  // is external power connected

long BlinkTime = 0;
bool BlinkState = HIGH;
bool menu = false;

bool was_mounted = false;  //is USB connected and mounted
void tud_umount_cb(void) {
  was_mounted = true;
}

void setup() {
  Serial.begin(115200);
  delay(300);                  // helps with usb connection
  pinMode(CHARGE_LED, INPUT);  //sets to detetct if charge LED is on or off to see if USB is plugged in and battery charging
  pinMode(HICHG, OUTPUT);
  digitalWrite(HICHG, LOW);  //100 mA charging current if set to LOW and 50mA (actually about 20mA) if set to HIGH
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(button[i], INPUT_PULLUP);
  }
  pinMode(User, INPUT_PULLUP);
  selectPreset();
  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  // Configure and Start Device Information Service
  bledis.begin();
  USBMIDI.begin(MIDI_CHANNEL_OMNI);
  USBMIDI.turnThruOff();   // USB midi should be off by default but I was having midi loops!
  DINMIDI.begin(MIDI_CHANNEL_OMNI);
  //MIDI.turnThruOff();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();    // BLE midi should be off by default but I was having midi loops!
  selectDevice();
  startAdv();
  LEDs();
}


void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  if (deviceMode[preset] == 0) {
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
    Bluefruit.Advertising.addService(blegamepad);
  }
  if (deviceMode[preset] == 1) {
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
    Bluefruit.Advertising.addService(blekeyboard);
  }
  if (deviceMode[preset] >= 2) {
    Bluefruit.Advertising.addService(blemidi);
  }

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}


void selectDevice() {
  switch (deviceMode[preset]) {
    case 0:
      //USBDevice.setManufacturerDescriptor("CHobbs                          ");
      //USBDevice.setProductDescriptor("Tracker Controller              ");
      USBDevice.setManufacturerDescriptor("LazyDreamer                       ");
      USBDevice.setProductDescriptor("TC                              ");
      usb_gamepad.begin();  //comment out to use serial.print for debugging
      bledis.setManufacturer("LazyDreamer");
      bledis.setModel("nRf52840");
      Bluefruit.setName("TC");
      blegamepad.begin();
      break;

    case 1:
      USBDevice.setProductDescriptor("TC                              ");
      USBDevice.setManufacturerDescriptor("LazyDreamer                       ");
      usb_keyboard.begin();  //comment out to use serial.print for debugging
      bledis.setManufacturer("LazyDreamer");
      bledis.setModel("nRf52840");
      Bluefruit.setName("TC");
      blekeyboard.begin();
      break;

    case 2:
      //USBDevice.setManufacturerDescriptor("CHobbs                          ");
      //USBDevice.setProductDescriptor("Tracker Controller              ");
      USBDevice.setProductDescriptor("TC                              ");
      USBDevice.setManufacturerDescriptor("LazyDreamer                       ");
      bledis.setManufacturer("LazyDreamer");
      bledis.setModel("nRf52840");
      Bluefruit.setName("TC");
  }
}



void loop() {

  MidiPass();

  if (menu == false) {  //dont send button commands if preset selection menu is active
    ReadButtons();
  }
  ReadUserButton();

  if ((millis() - lastBatteryCheck) > 1000) {  //check battery and usb connection  only once per second
    if (Batt == true) {
      batteryCheck();
    }
    USBCheck();
    ExtPowerCheck();
    lastBatteryCheck = millis();
  }
  LowBatteryWarning();

  if (StandbyOn == 1) {
    sd_app_evt_wait();
  }
}


void Sleep() {
  //turn off LEDS
  LEDsOff();
  //before you put the NRF to sleep put the flash to sleep
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  flashTransport.end();

  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  __WFE();
  __WFI();
  //add pin interrupts to wake
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D0], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //a
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D1], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //b
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D2], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //up
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D3], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //right
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D4], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //down
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D5], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //left
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D8], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //start
  //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D9], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //select
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D10], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);  //secret user button

  sd_app_evt_wait();  //SOFT DEVICE “SYSTEM ON” POWER SAVE
  delay(2000);
  NRF_POWER->SYSTEMOFF = 1;
}


void ReadButtons() {
  //read buttons & send commands
  switch (deviceMode[preset]) {
    case 0:  //gamepad
      {
        gp.buttons = 0;
        gp.hat = 0;
        uint8_t count = 0;
        static bool keyPressedPreviously = false;

        if (millis() - lastSwitchTime >= debounceTime) {
          lastSwitchTime = millis();
          for (int i = 0; i < 8; i++) {
            buttonState[i] = digitalRead(button[i]);
            if (buttonState[i] == LOW) {  //button pressed
              count++;
              if (DPad[preset] == 1) {
                if (i < 2 || i > 5) {  //send button commands for the "buttons"
                  gp.buttons |= 1 << command[preset][i];
                }
              } else gp.buttons |= 1 << command[preset][i];
            }
            //lastActivity = millis();
            previousButtonState[i] = buttonState[i];
          }

          if (count) {
            if (DPad[preset] == 1) {
              ReadDpad();
            }
            keyPressedPreviously = true;
            if (!Bluefruit.connected()) {
              usb_gamepad.sendReport(0, &gp, sizeof(gp));
            } else {
              blegamepad.report(&gp);
              lastActivity = millis();
            }
          } else {
            // Send All-zero report to indicate there is no keys pressed
            // Most of the time, it is, though we don't need to send zero report
            // every loop(), only a key is pressed in previous loop()
            if (keyPressedPreviously) {
              keyPressedPreviously = false;
              if (!Bluefruit.connected()) {
                usb_gamepad.sendReport(0, &gp, sizeof(gp));
              } else {
                blegamepad.report(&gp);
              }
              lastActivity = millis();
            }
          }
        }
        break;
      }

    case 1:  //keyboard
      {
        uint8_t count = 0;
        uint8_t keycode[6] = { 0 };
        static bool keyPressedPreviously = false;
        uint8_t modifier = 0;

        if (millis() - lastSwitchTime >= debounceTime) {
          lastSwitchTime = millis();
          for (int i = 0; i < 8; i++) {
            buttonState[i] = digitalRead(button[i]);
            if (buttonState[i] == LOW) {
              keycode[count++] = command[preset][i];
              if (keyModifier[preset][i] != 0) {
                modifier = keyModifier[preset][i];
              }
              if (keyModifier2[preset][i] != 0) {
                modifier = modifier | keyModifier2[preset][i];  //?
              }
              if (count == 6) break;
            }
            previousButtonState[i] = buttonState[i];
          }

          if (count) {
            uint8_t const report_id = 0;
            //uint8_t const modifier = 0;
            keyPressedPreviously = true;
            if (!Bluefruit.connected()) {
              usb_keyboard.keyboardReport(report_id, modifier, keycode);
            } else {
              blekeyboard.keyboardReport(report_id, modifier, keycode);
            }
            lastActivity = millis();
          } else {
            // Send All-zero report to indicate there is no keys pressed
            // Most of the time, it is, though we don't need to send zero report
            // every loop(), only a key is pressed in previous loop()
            if (keyPressedPreviously) {
              keyPressedPreviously = false;
              if (!Bluefruit.connected()) {
                usb_keyboard.keyboardRelease(0);
              } else {
                blekeyboard.keyRelease(0);
              }
              lastActivity = millis();
            }
          }
        }
        break;
      }

    case 2:  //midi controller
      {
        if (millis() - lastSwitchTime >= debounceTime) {
          lastSwitchTime = millis();
          for (int i = 0; i < 8; i++) {
            buttonState[i] = digitalRead(button[i]);
            if (buttonState[i] == LOW && previousButtonState[i] == HIGH) {
              uint8_t value;
              if (midiMaxValue[preset][i] != 0) {
                value = midiMaxValue[preset][i];
              } else value = 127;
              switch (midiType[preset][i]) {
                case 0:  //Note on
                  {
                    if ((midiToggle[preset][i] == 0) || (midiToggle[preset][i] == 1 && midiToggleState[preset][i] == 0)) {
                      if (was_mounted == true) {
                        USBMIDI.sendNoteOn(command[preset][i], value, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendNoteOn(command[preset][i], value, midiChannel[preset][i]);
                      }
                      DINMIDI.sendNoteOn(command[preset][i], value, midiChannel[preset][i]);
                      midiToggleState[preset][i] = 1;

                    } else if (midiToggle[preset][i] == 1 && midiToggleState[preset][i] == 1) {
                      if (was_mounted == true) {
                        USBMIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                      }
                      DINMIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                      midiToggleState[preset][i] = 0;
                    }
                    break;
                  }
                case 1:  //Control Change
                  {
                    if ((midiToggle[preset][i] == 0) || (midiToggle[preset][i] == 1 && midiToggleState[preset][i] == 0)) {
                      if (was_mounted == true) {
                        USBMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      DINMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      midiToggleState[preset][i] = 1;

                    } else if (midiToggle[preset][i] == 1 && midiToggleState[preset][i] == 1) {
                      uint8_t value;
                      if (midiMinValue[preset][i] != 0) {
                        value = midiMinValue[preset][i];
                      } else value = 0;

                      if (was_mounted == true) {
                        USBMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      DINMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);

                      midiToggleState[preset][i] = 0;
                    }
                    break;
                  }

                case 2:  //program change
                  {
                    if (was_mounted == true) {
                      USBMIDI.sendProgramChange(command[preset][i], midiChannel[preset][i]);
                    }
                    if (Bluefruit.connected()) {
                      MIDI.sendProgramChange(command[preset][i], midiChannel[preset][i]);
                    }
                    DINMIDI.sendProgramChange(command[preset][i], midiChannel[preset][i]);

                    break;
                  }
                case 3:  //send start/stop
                  {
                    if (!playing) {
                      using namespace midi;
                      if (was_mounted == true) {
                        USBMIDI.sendRealTime(Start);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendRealTime(Start);
                      }
                      DINMIDI.sendRealTime(Start);
                      playing = true;
                    } else if (playing) {
                      using namespace midi;
                      if (was_mounted == true) {
                        USBMIDI.sendRealTime(Stop);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendRealTime(Stop);
                      }
                      DINMIDI.sendRealTime(Stop);
                      playing = false;
                    }
                    break;
                  }
              }
              lastActivity = millis();
            }

            if (buttonState[i] == HIGH && previousButtonState[i] == LOW) {  //button released
              switch (midiType[preset][i]) {
                case 0:  //note off
                  {
                    if (midiToggle[preset][i] == 0) {
                      if (was_mounted == true) {
                        USBMIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                      }
                      DINMIDI.sendNoteOff(command[preset][i], 0, midiChannel[preset][i]);
                    }
                    break;
                  }
                case 1:  //Control Change release
                  {
                    if (midiToggle[preset][i] == 0) {
                      uint8_t value;
                      if (midiMinValue[preset][i] != 0) {
                        value = midiMinValue[preset][i];
                      } else value = 0;

                      if (was_mounted == true) {
                        USBMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      if (Bluefruit.connected()) {
                        MIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                      }
                      DINMIDI.sendControlChange(command[preset][i], value, midiChannel[preset][i]);
                    }
                    break;
                  }
              }
              lastActivity = millis();
            }
            previousButtonState[i] = buttonState[i];
          }
        }
        break;
      }
  }
}

void MidiPass() {
  if (USBMIDI.read()) {
    // Forward the message to out DIN and BLE.
    

    DINMIDI.send(USBMIDI.getType(),
                 USBMIDI.getData1(),
                 USBMIDI.getData2(),
                 USBMIDI.getChannel());

    if (ThruToBLE == true) {
      MIDI.send(USBMIDI.getType(),
                USBMIDI.getData1(),
                USBMIDI.getData2(),
                USBMIDI.getChannel());
    }
    //}
    lastActivity = millis();
  }

  if (DINMIDI.read()) {
    // Thru on DIN has already pushed the input message to out DIN.
    // Forward the message to out USB and BLE as well.
    /*
   
    midi::MidiType msgType = DINMIDI.getType();
    if (msgType == midi::SystemExclusive) {
      USBMIDI.sendSysEx(DINMIDI.getSysExArrayLength(),
                        DINMIDI.getSysExArray());
      if (ThruToBLE == true) {
        MIDI.sendSysEx(DINMIDI.getSysExArrayLength(),
                       DINMIDI.getSysExArray());
      }
    } else {
      // */
   
    USBMIDI.send(DINMIDI.getType(),
                 DINMIDI.getData1(),
                 DINMIDI.getData2(),
                 DINMIDI.getChannel());

    if (ThruToBLE == true) {
      MIDI.send(DINMIDI.getType(),
                DINMIDI.getData1(),
                DINMIDI.getData2(),
                DINMIDI.getChannel());
    }
    //}

    lastActivity = millis();
  }

  if (MIDI.read()) {
    // if ((ThruToBLE == true) && (MIDI.read())) {
    // Thru on BLE has already pushed the input message to out BLE.
    // Forward the message to out USB and DIN as well.
    USBMIDI.send(MIDI.getType(),
                 MIDI.getData1(),
                 MIDI.getData2(),
                 MIDI.getChannel());

    DINMIDI.send(MIDI.getType(),
                 MIDI.getData1(),
                 MIDI.getData2(),
                 MIDI.getChannel());

    lastActivity = millis();
  }
}

void batteryCheck() {

  //check charging status and give it a variable
  BAT_CHRG = digitalRead(CHARGE_LED);  // BAT_CHRG == 1: USB/charger IS NOT plugged in.     BAT_CHRG == 0: USB/charger IS plugged in.

  readBAT();  // loops back to lipo BAT read routine

  ////////////////// go to sleep if there is no activity
  if ((ExtPower == false) || ((ExtPower == true) && (BAT_CHRG == 0) && (ChargingSleep == true)) || ((ExtPower == true) && (was_mounted == true) && (USBsleep == true))) {  //if the device is not connected to power or USB connected, sleep.
    if (((millis() - lastActivity) / 1000) >= (StandbyTime * 60)) {
      if (StandbyOn == 0) {  /// only do this the first time
                             // sd_app_evt_wait();
        LEDsOff();
        StandbyOn = 1;
      }
      if (((millis() - lastActivity) / 1000) >= (SleepTime * 60)) {
        Sleep();
      }
    } else if (StandbyOn == 1) {  //if the last activity time has changed during standby( standby wake up), reset the standby flag and leds
      LEDs();
      StandbyOn = 0;
    }
  } else if (StandbyOn == 1) {  //if a usb/ charging/ power connection changed during standby, reset the standby flag and leds
    LEDs();
    StandbyOn = 0;
  }
}


void readBAT() {
  unsigned int adcCount = analogRead(PIN_VBAT);  //  PIN_VBAT is reading voltage divider on XIAO and is program pin 32 or P0.31??
  double adcVoltage = (adcCount * vRef) / numReadings;
  vBat = adcVoltage * 1675.0 / 1341.3;  //1510.0;  // Voltage divider from Vbat to ADC  // was set at 1510.0...set your own value to calibrate to actual voltage reading by your voltmeeter
  //printf("adcCount = %3u = 0x%03X, adcVoltage = %.3fV, vBat = %.3f\n", adcCount, adcCount, adcVoltage, vBat);
}

void USBCheck() {
  //check to see if USB is connected...or not
  tud_umount_cb();
  if (tud_mounted && tud_ready()) {
    was_mounted = true;
  } else {
    was_mounted = false;
  }
}

void LEDs() {
  // set LEDs per Device type (LOW = On, HIGH = Off)
  switch (deviceMode[preset]) {  //Green = Gamepad
    case 0:
      digitalWrite(LED_GREEN, LOW);
      break;

    case 1:
      digitalWrite(LED_GREEN, LOW);  //Orange = Keyboard
      digitalWrite(LED_RED, LOW);
      break;

    case 2:
      digitalWrite(LED_RED, LOW);  //Red= Midi Device
      break;

    case 3:
      digitalWrite(LED_RED, LOW);
      break;
  }
}

void LEDsOff() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void LowBatteryWarning() {
  if (Batt == true && was_mounted == false) {
    if (vBat <= 3.3 && BAT_CHRG == 1) {  //Low battery and not connecting to USB power source
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      if (millis() - BlinkTime > 500) {
        BlinkState = digitalRead(LED_RED);
        digitalWrite(LED_RED, !BlinkState);
        BlinkTime = millis();
      }
    }
  }
}

void ReadUserButton() {
  UserbuttonState = digitalRead(User);
  if (UserbuttonState == LOW && previousUserButtonState == HIGH) {
    ButtonHoldTime = millis();
    ClickCount++;
    lastActivity = millis();
    if (ClickCount == 1) {
      ButtonDoubleClickTime = millis();
    }
  }
  if (UserbuttonState == LOW && previousUserButtonState == LOW) {
    if (millis() - ButtonHoldTime > 1000) {
      //button hold event
      menu = 1;
      changeDevice();
    }
  } else menu = 0;

  if (ClickCount >= 2) {
    //double Click event
    ToggleThruToBLE();
  }
  if ((millis() - ButtonDoubleClickTime) > DoubleClickTimeOut) {
    ClickCount = 0;  //double clicks have to be within this time
  }

  previousUserButtonState = UserbuttonState;
}



void changeDevice() {  //click and hold
  selectPreset();
  if (previousPreset != preset) {
    selectDevice();
    startAdv();
    //turn off LEDS
    LEDsOff();
    LEDs();
  }
}

void ToggleThruToBLE() {
  ThruToBLE = !ThruToBLE;
  ClickCount = 0;
  //turn off LEDS
  LEDsOff();
  // flash LEDs to indicate if BT Midi is on or off
  if (ThruToBLE == false) {
    for (int i = 0; i < 6; i++) {
      BlinkState = digitalRead(LED_RED);
      digitalWrite(LED_RED, !BlinkState);
      delay(250);
    }
  }
  if (ThruToBLE == true) {
    for (int i = 0; i < 6; i++) {
      BlinkState = digitalRead(LED_GREEN);
      digitalWrite(LED_GREEN, !BlinkState);
      delay(250);
    }
  }
  //return LEDs to original state
  LEDs();
}




void ReadDpad() {
  if (digitalRead(up) == LOW && digitalRead(right) == HIGH && digitalRead(down) == HIGH && digitalRead(left) == HIGH) {
    gp.hat = 1;  // GAMEPAD_HAT_UP;
    //Serial.println("Hat/DPAD UP");
  }

  if (digitalRead(up) == LOW && digitalRead(right) == LOW && digitalRead(down) == HIGH && digitalRead(left) == HIGH) {
    gp.hat = 2;  // GAMEPAD_HAT_UP_RIGHT;
    //Serial.println("Hat/DPAD UP RIGHT");
  }

  if (digitalRead(up) == HIGH && digitalRead(right) == LOW && digitalRead(down) == HIGH && digitalRead(left) == HIGH) {
    gp.hat = 3;  // GAMEPAD_HAT_RIGHT;
    //Serial.println("Hat/DPAD RIGHT");
  }

  if (digitalRead(up) == HIGH && digitalRead(right) == LOW && digitalRead(down) == LOW && digitalRead(left) == HIGH) {
    gp.hat = 4;  // GAMEPAD_HAT_DOWN_RIGHT;
                 // Serial.println("Hat/DPAD DOWN RIGHT");
  }

  if (digitalRead(up) == HIGH && digitalRead(right) == HIGH && digitalRead(down) == LOW && digitalRead(left) == HIGH) {
    gp.hat = 5;  // GAMEPAD_HAT_DOWN;
                 // Serial.println("Hat/DPAD DOWN");
  }

  if (digitalRead(up) == HIGH && digitalRead(right) == HIGH && digitalRead(down) == LOW && digitalRead(left) == LOW) {
    gp.hat = 6;  // GAMEPAD_HAT_DOWN_LEFT;
    //Serial.println("Hat/DPAD DOWN LEFT");
  }

  if (digitalRead(up) == HIGH && digitalRead(right) == HIGH && digitalRead(down) == HIGH && digitalRead(left) == LOW) {
    gp.hat = 7;  // GAMEPAD_HAT_LEFT;
    //Serial.println("Hat/DPAD LEFT");
  }

  if (digitalRead(up) == LOW && digitalRead(right) == HIGH && digitalRead(down) == HIGH && digitalRead(left) == LOW) {
    gp.hat = 8;  // GAMEPAD_HAT_UP_LEFT;
    //Serial.println("Hat/DPAD UP LEFT");
  }
  // else { gp.hat = 0; // GAMEPAD_HAT_CENTERED;
  //  Serial.println("Hat/DPAD CENTER");
  //  }
}

void ExtPowerCheck() {
  if (!(bitRead(NRF_POWER->USBREGSTATUS, 0)) && ExtPower == 1) {
    ExtPower = false;
  } else if (bitRead(NRF_POWER->USBREGSTATUS, 0)) {
    ExtPower = true;
  }
}



//void sysex() {}




  ////////////////////////graveyard/////////////////////////////
  /*


void USBmountCheck() {
    if (was_mounted && tud_ready()) ) { 
        // Do nothing here - PC is connected
    } else {
        was_mounted = false;
        // and do normal tasks here: measure, display... 
    }
}




#include "MIDIUSB.h"

byte message[] = {0xF0,0x00, 0x00, 0x66, 0x05, 0x00, 0x10, 0x00, 0x6D, 0x61, 0x72, 0x65, 0xF7};

void setup() {
  while(!Serial) {};
}

void sysex() {
  
  
  
  byte x = getSysExArrayLength();
  byte message[x] = DINMIDI.getSysExArray();
  
  MidiUSB_sendSysEx(message,sizeof(message));
  MidiUSB.flush(); 
  delay(1000);
}

void MidiUSB_sendSysEx(const uint8_t *data, size_t size)
{
    if (data == NULL || size == 0) return;

    size_t midiDataSize = (size+2)/3*4;
    uint8_t midiData[midiDataSize];
    const uint8_t *d = data;
    uint8_t *p = midiData;
    size_t bytesRemaining = size;

    while (bytesRemaining > 0) {
        switch (bytesRemaining) {
        case 1:
            *p++ = 5;   // SysEx ends with following single byte
            *p++ = *d;
            *p++ = 0;
            *p = 0;
            bytesRemaining = 0;
            break;
        case 2:
            *p++ = 6;   // SysEx ends with following two bytes
            *p++ = *d++;
            *p++ = *d;
            *p = 0;
            bytesRemaining = 0;
            break;
        case 3:
            *p++ = 7;   // SysEx ends with following three bytes
            *p++ = *d++;
            *p++ = *d++;
            *p = *d;
            bytesRemaining = 0;
            break;
        default:
            *p++ = 4;   // SysEx starts or continues
            *p++ = *d++;
            *p++ = *d++;
            *p++ = *d++;
            bytesRemaining -= 3;
            break;
        }
    }
    MidiUSB.write(midiData, midiDataSize);
}




static void onUSBSysEx(byte * array, unsigned size)
{
    Serial.printf("SysEx:\r\n");
    unsigned multipleOf8 = size/8;
    unsigned remOf8 = size % 8;
    for (unsigned idx=0; idx < multipleOf8; idx++) {
        for (unsigned jdx = 0; jdx < 8; jdx++) {
            Serial.printf("%02x ", *array++);
        }
        Serial.printf("\r\n");
    }
    for (unsigned idx = 0; idx < remOf8; idx++) {
        Serial.printf("%02x ", *array++);
    }
    Serial.printf("\r\n");
}






*/
