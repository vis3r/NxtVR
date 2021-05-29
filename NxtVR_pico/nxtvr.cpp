#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "sensors/mpu6050/mpu6050.h"
#include "stdint.h"
#include "bsp/board.h"
#include "tusb.h"

MPU6050 mpu6050;
typedef struct TU_ATTR_PACKED
{
   int32_t ax;
   int32_t ay;
   int32_t az;
   int32_t gx;
   int32_t gy;
   int32_t gz;
   int32_t mx;
   int32_t my;
   int32_t mz;
}hid_hmd_report_t;

enum  {
   BLINK_NOT_MOUNTED = 250,
   BLINK_MOUNTED = 1000,
   BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
   (void) report_id;
   (void) report_type;
   (void) buffer;
   (void) reqlen;

   return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
   (void) report_id;
   (void) report_type;
   (void) buffer;
   (void) bufsize;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
   static uint32_t start_ms = 0;
   static bool led_state = false;

   // Blink every interval ms
   if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
   start_ms += blink_interval_ms;

   board_led_write(led_state);
   led_state = 1 - led_state; // toggle
}

void hid_task(void)
{
   // Poll every 10ms
   const uint32_t interval_ms = 10;
   static uint32_t start_ms = 0;

   if (board_millis() - start_ms < interval_ms) return; // not enough time
   start_ms += interval_ms;

   // Remote wakeup
   if (tud_suspended()) {
      // Wake up host if we are in suspend mode
      // and REMOTE_WAKEUP feature is enabled by host
      tud_remote_wakeup();
   }

   // set the values of our report 
   hid_hmd_report_t report = 
   {
      .ax = mpu6050.accel[0], .ay = mpu6050.accel[1], .az = mpu6050.accel[2],
      .gx = mpu6050.gyro[0], .gy = mpu6050.gyro[2], .gz = mpu6050.gyro[2],
      .mx = 0, .my = 0, .mz = 0
   };
   //send the report if the device is ready for sending the data
   if (tud_hid_ready()) {
      tud_hid_report(0x01, &report, 32);
   }
}
int main()
{
   int16_t bias[6] = {OFFSET_AX, OFFSET_AY, OFFSET_AZ, OFFSET_GX, OFFSET_GY, OFFSET_GZ};
   board_init();
   tusb_init();
   stdio_init_all();
   i2c_init(I2C_PORT, 400 * 1000);
   gpio_set_function(4, GPIO_FUNC_I2C);
   gpio_set_function(5, GPIO_FUNC_I2C);
   gpio_pull_up(4);
   gpio_pull_up(5); 
   mpu6050.begin();
   mpu6050.configure(bias);
   while(1)
   { 
      tud_task();
      mpu6050.readAccel(mpu6050.accel);
      mpu6050.readGyro(mpu6050.gyro);
      hid_task(); 
      led_blinking_task();
   }

   return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
   blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
   blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
   (void) remote_wakeup_en;
   blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
   blink_interval_ms = BLINK_MOUNTED;
}
