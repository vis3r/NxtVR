#include "src/hid_hmd/hid_hmd.h"

#include <Wire.h>

#include "src/sketch_utils.h"

/* -- Sensor selection
   see sensor_board.h file for explanation
*/

//#define SENSOR_BMX055
#define SENSOR_MPU6050
//#define SENSOR_DUMMY
#include "src/sensor_board.h"
// -- Sensor selection end

// unified sensor interface
SensorBoard sensor;

HIDHeadMountedDisplay HIDhmd;

void setup()
{
  USBComposite.setManufacturerString("Vis3r");
  USBComposite.setProductString("NxtVR HMD");
  USBComposite.setSerialString("nxt0.1");

  USBComposite.setVendorId(0x1209);
  USBComposite.setProductId(0x9D0F);

  pinMode(LED_BUILTIN, OUTPUT);
  // led is on in LOW
  //  if your led doesn't power off after
  //  some time and stays on
  //  then this has blocked

  //link report pointers
  HIDhmd.motion_p = &sensor.motion;

  HIDhmd.begin();
  delay(100);

  sensor.init();
  sensor.configure();
  
  delay(100);
  
  led_state != led_state;
  digitalWrite(LED_BUILTIN, led_state);
  delay(300);
}

void loop()
{
  // get new data into sensor.motion report
  sensor.refresh();
  // does what is needed to parse (shared) motion report
  // and sends it down the USB
  HIDhmd.sendMotion();

  flip_led();
  //delay(50);
}
