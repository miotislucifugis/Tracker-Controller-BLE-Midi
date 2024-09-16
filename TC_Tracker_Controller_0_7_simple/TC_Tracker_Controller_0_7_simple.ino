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
battery reading :https://www.unmannedtechshop.co.uk/product/seeed-xiao-ble-nrf52840/
*/

#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <Arduino.h>
//#include <Adafruit_FlashTransport.h>

Adafruit_USBD_MIDI usb_midi;

//Adafruit_FlashTransport_QSPI flashTransport;

BLEDis bledis;
BLEHidGamepad blegamepad;
BLEHidAdafruit blekeyboard;
BLEMidi blemidi;

MIDI_CREATE_BLE_INSTANCE(blemidi);
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, USBMIDI);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, DINMIDI);

uint8_t const desc_gamepad_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};


Adafruit_USBD_HID usb_gamepad(desc_gamepad_report, sizeof(desc_gamepad_report), HID_ITF_PROTOCOL_NONE, 2, false);

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


//user preset configs //////////////////////////////////////////////////
#define Batt true
bool USBsleep = false;          // do you want to sleep when connected to usb and BT or only BT?  false= never sleep when USB is connected
int defaultPreset = 0;          //default preset 0 = 1st preset, 1 = 2nd, etc
uint8_t SleepTime = 2;          //minutes of inactivity before sleep
uint8_t StandbyTime = 1;        //minutes of inactivity before standby
int debounceTime = 20;          //switch debounce
bool BTMidi = false;            //default for  BT midi .  True = on ,false = off (can be changed durring use w/ double click of the user button.)  ( Sending to BLE midi can cause sever lag on DIN and USB! better to leave it off and just turn t on when you need it!)
int DoubleClickTimeOut = 1000;  //max time for detecting a double click on the User button



//3 rows, 8 columns   row=preset;  column = command for button or key to be sent.  for midi, command equals note or CC #
uint8_t command[8] = { 1, 0, 12, 15, 13, 14, 9, 8 };  //preset 0   generic NES-style  game controller for M8, LSDJ, LPGT, etc


int preset = defaultPreset;


////end user preset config/////////////////////////////////////////


hid_gamepad_report_t gp;
bool activeState = false;
int buttonState[8] = {};
int previousButtonState[8] = { HIGH };
long lastActivity = 0;
static bool playing = false;
long lastSwitchTime;

int UserbuttonState;
int previousUserButtonState = HIGH;
int ClickCount = 0;
long ButtonDoubleClickTime;
long ButtonHoldTime;
bool StandbyOn = 0;

////battery stuff
#define BAT_READ 14                     // P0_14 = 14  Reads battery voltage from divider on signal board. (PIN_VBAT is reading voltage divider on XIAO and is program pin 32 / or P0.31)
#define CHARGE_LED 23                   // P0_17 = 17  D23   YELLOW CHARGE LED
#define HICHG 22                        // P0_13 = 13  D22   Charge-select pin for Lipo for 100 mA instead of default 50mA charge
const double vRef = 3.3;                // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024;  // 10-bit ADC readings 0-1023, so the factor is 1024
volatile int BAT_CHRG;                  //is the battery connected and charging?   5V connected/charging = 0; disconnected = 1
double vBat;

long lastBatteryCheck;
bool ExtPower = 0;

long BlinkTime = 0;
bool BlinkState = HIGH;

bool was_mounted = false;
void tud_umount_cb(void) {
  was_mounted = true;
}

void setup() {
  Serial.begin(115200);
  delay(300);                  // helps with usb connection
  pinMode(CHARGE_LED, INPUT);  //sets to detetct if charge LED is on or off to see if USB is plugged in
  pinMode(HICHG, OUTPUT);
  digitalWrite(HICHG, LOW);  //100 mA charging current if set to LOW and 50mA (actually about 20mA) if set to HIGH
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(button[i], INPUT_PULLUP);
  }
  pinMode(User, INPUT_PULLUP);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  // Configure and Start Device Information Service
  bledis.begin();
  USBMIDI.begin(MIDI_CHANNEL_OMNI);
  DINMIDI.begin(MIDI_CHANNEL_OMNI);
  //MIDI.begin(MIDI_CHANNEL_OMNI);
  selectDevice();
  startAdv();
  LEDs();
}


void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
  Bluefruit.Advertising.addService(blegamepad);

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
  //USBDevice.setManufacturerDescriptor("CHobbs                          ");
  //USBDevice.setProductDescriptor("Tracker Controller              ");
  USBDevice.setManufacturerDescriptor("LazyDreamer                       ");
  USBDevice.setProductDescriptor("TC                              ");
  usb_gamepad.begin();  //comment out to use serial.print
  bledis.setManufacturer("LazyDreamer");
  bledis.setModel("nRf52840");
  Bluefruit.setName("TC");
  blegamepad.begin();
}



void loop() {
  MidiPass();
  ReadButtons();
  ReadUserButton();
  if ((millis() - lastBatteryCheck) > 1000) {
    if (Batt == true) {
      batteryCheck();
    }
    USBCheck();
    ExtPowerCheck();
    lastBatteryCheck = millis();
  }
  LowBatteryWarning();
}


void ReadButtons() {
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
        gp.buttons |= 1 << command[i];
      }
      //lastActivity = millis();
      previousButtonState[i] = buttonState[i];
    }
    if (count) {
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
}


void Sleep() {
  //turn off LEDS
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  //before you put the NRF to sleep put the flash to sleep
  //flashTransport.begin();
  // flashTransport.runCommand(0xB9);
  // flashTransport.end();

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


void MidiPass() {
  if (USBMIDI.read()) {
    // Thru on USB has already pushed the input message to out USB.
    // Forward the message to out DIN and BLE as well.
    DINMIDI.send(USBMIDI.getType(),
                 USBMIDI.getData1(),
                 USBMIDI.getData2(),
                 USBMIDI.getChannel());

    if (BTMidi == true) {
      MIDI.send(USBMIDI.getType(),
                USBMIDI.getData1(),
                USBMIDI.getData2(),
                USBMIDI.getChannel());
    }
    lastActivity = millis();
  }

  if (DINMIDI.read()) {
    // Thru on DIN has already pushed the input message to out DIN.
    // Forward the message to out USB and BLE as well.
    USBMIDI.send(DINMIDI.getType(),
                 DINMIDI.getData1(),
                 DINMIDI.getData2(),
                 DINMIDI.getChannel());

    if (BTMidi == true) {
      MIDI.send(DINMIDI.getType(),
                DINMIDI.getData1(),
                DINMIDI.getData2(),
                DINMIDI.getChannel());
    }
    lastActivity = millis();
  }
  if (MIDI.read()) {
    //if ((BTMidi == true) && (MIDI.read())) {
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
  //check charging status and give it a variable  - not currently used 
  /*BAT_CHRG = digitalRead(CHARGE_LED);  //
  if (BAT_CHRG == 1) {                 //if USB/charger IS NOT plugged in - debugging messages
                                       // Serial.print("  Battery Charge status =  "); Serial.println(BAT_CHRG);
                                       // Serial.println(" USB -IS NOT- plugged in ");  //do whatever you want here once USB is detected, in order to make sure user knows that battery-read values are now USB-generate
  }
  if (BAT_CHRG == 0) {  ////if USB/charger IS plugged in  - debugging messages
                        // Serial.print("  Battery Charge status =  ");
                        // Serial.println(BAT_CHRG);
                        // Serial.println(" USB -IS- plugged in ");  //do whatever you want here once USB is detected, in order to make sure user knows that battery-read values are now USB-generated
  }
  */

  readBAT();  // loops back to lipo BAT read routine
              //////////////////  go to sleep if there has been no activity

  if ((ExtPower == false) || (ExtPower == true) && (was_mounted == true) && (USBsleep == true)) {  //if the device is not wired to power or USB connected, sleep.
    if (((millis() - lastActivity) / 1000) >= (StandbyTime * 60)) {
      if (StandbyOn == 0) {
        LEDsOff();
        sd_app_evt_wait();
        StandbyOn = 1;
      }
      if (((millis() - lastActivity) / 1000) >= (SleepTime * 60)) {
        Sleep();
      }
    } else if (StandbyOn == 1) {
      LEDs();
      StandbyOn = 0;
    }
  }
}


void readBAT() {
  unsigned int adcCount = analogRead(PIN_VBAT);  //  PIN_VBAT is reading voltage divider on XIAO and is program pin 32 or P0.31??
  double adcVoltage = (adcCount * vRef) / numReadings;
  vBat = adcVoltage * 1675.0 / 1341.3;  //510.0;  // Voltage divider from Vbat to ADC  // was set at 1510.0...set your own value to calibrate to actual voltage reading by your voltmeeter
  //printf("adcCount = %3u = 0x%03X, adcVoltage = %.3fV, vBat = %.3f\n", adcCount, adcCount, adcVoltage, vBat);
  //delay(10);
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
  digitalWrite(LED_GREEN, LOW);
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
    if (ClickCount == 1) {
      ButtonDoubleClickTime = millis();
    }
  }
  if (UserbuttonState == LOW && previousUserButtonState == LOW) {
    if (millis() - ButtonHoldTime > 1000) {
      //button hold event   not currently used
    }
  }
  if (ClickCount >= 2) {
    //double Click event
    ToggleBTMidi();
  }
  if ((millis() - ButtonDoubleClickTime) > DoubleClickTimeOut) {
    ClickCount = 0;  //double clicks have to be within this time
  }

  previousUserButtonState = UserbuttonState;
}

void ToggleBTMidi() {
  BTMidi = !BTMidi;
  ClickCount = 0;
  //turn off LEDS
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  // flash LEDs to indicate if BT Midi is on or off
  if (BTMidi == false) {
    for (int i = 0; i < 6; i++) {
      BlinkState = digitalRead(LED_RED);
      digitalWrite(LED_RED, !BlinkState);
      delay(250);
    }
  }
  if (BTMidi == true) {
    for (int i = 0; i < 6; i++) {
      BlinkState = digitalRead(LED_GREEN);
      digitalWrite(LED_GREEN, !BlinkState);
      delay(250);
    }
  }
  //return LEDs to original state
  LEDs();
}


void ExtPowerCheck() {
  if (!(bitRead(NRF_POWER->USBREGSTATUS, 0)) && ExtPower == 1) {
    ExtPower = false;
  } else if (bitRead(NRF_POWER->USBREGSTATUS, 0)) {
    ExtPower = true;
  }
}






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
*/
