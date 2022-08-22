/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include <string.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

#define LIS2DH12_ADDR         0x19

#define LIS2DH12_CTRL_REG1    0x20
#define LIS2DH12_CTRL_REG2    0x21
#define LIS2DH12_CTRL_REG3    0x22
#define LIS2DH12_CTRL_REG4    0x23
#define LIS2DH12_CTRL_REG5    0x24
#define LIS2DH12_CTRL_REG6    0x25

#define LIS2DH12_INT2_THS     0x36
#define LIS2DH12_INT2_DURATION 0x37
#define LIS2DH12_INT2_CFG     0x34
#define LISDH12_INT2_SRC      0x35

#define LISDH12_OUT_X         0x28
#define LISDH12_OUT_X_AUTO_INC LISDH12_OUT_X | (1 << 7) //N.B. to have multi byte reads with auto increment address, must specify 1 in MSB of SUb address.

/**
* Helper function to configure LISDH12 accelerometer
* to detect free falls
*/
void configure_LISDH12_free_fall(void) {
  //See https://www.st.com/resource/en/application_note/dm00365457-lis2dh12-mems-digital-output-motion-sensor-ultralowpower-highperformance-3axis-nano-accelerometer-stmicroelectronics.pdf
  //page 27 for details
  uint8_t data[2];

  data[0] = LIS2DH12_CTRL_REG1;
  data[1] = 0x57;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_CTRL_REG2;
  data[1] = 0x00;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_CTRL_REG6;
  data[1] = 0x40;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_CTRL_REG4;
  data[1] = 0x00;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_CTRL_REG5;
  data[1] = 0x02;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_INT2_THS;
  data[1] = 0x1A;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);

  data[0] = LIS2DH12_INT2_DURATION;
  data[1] = 0x0A;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);
 
  data[0] = LIS2DH12_INT2_CFG;
  data[1] = 0x95;
  dwm_i2c_write(LIS2DH12_ADDR, data, 2, 0);
}

/*
**Helper function to clear INT1 free fall interrupt
* by reading INT2 SRC register
*/
bool LISDH12_check_free_fall(void) {
  uint8_t data = LISDH12_INT2_SRC;
  dwm_i2c_write(LIS2DH12_ADDR, &data, 1, 0);
  dwm_i2c_read(LIS2DH12_ADDR, &data, 1);

  //if IA bit (7th bit of response data) is set then
  //free fall event generated. Otherwise no free fall.
  return data >> 6;
}

//Helper to read acceleration values via i2c.
//must contain 3 int16s to store data
void LISDH12_read_acc_values(int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t buf[6];
  uint8_t sub_address = LISDH12_OUT_X_AUTO_INC;
  dwm_i2c_write(LIS2DH12_ADDR, &sub_address, 1, 0);
  dwm_i2c_read(LIS2DH12_ADDR, buf, 6);
  
  *ax = (buf[1] << 8) | buf[0];
  *ay = (buf[3] << 8) | buf[2];
  *az = (buf[5] << 8) | buf[4];
}

#define FREE_FALL_POLL_INTERVAL   1000000U //us = every second (1Hz)
#define POS_POLL_INTERVAL         100000U //us = every 100ms (10Hz)
#define ACC_POLL_INTERVAL         20000U //us = every 20ms (50Hz)

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
        configure_LISDH12_free_fall();
        dwm_upd_rate_set(1,1);
        uint32_t last_fall_poll = 0, last_pos_poll = 0, last_acc_poll = 0;
        bool fall = false;
        dwm_pos_t pos;
        int16_t ax, ay, az;

	while (1) {
		/* Thread loop */
                if (dwm_systime_us_get() - last_fall_poll >= FREE_FALL_POLL_INTERVAL) {
                  last_fall_poll = dwm_systime_us_get();
                  if (LISDH12_check_free_fall()) {
                    //printf("Free fall!\n");
                    fall = true;
                  }
                  else {
                    //printf("Not\n");
                    fall = false;
                  }
                }
                
                if (dwm_systime_us_get() - last_pos_poll >= POS_POLL_INTERVAL) {
                   last_pos_poll = dwm_systime_us_get();
                   dwm_pos_get(&pos);
                }

                if (dwm_systime_us_get() - last_acc_poll >= ACC_POLL_INTERVAL) {
                   last_acc_poll = dwm_systime_us_get();
                   LISDH12_read_acc_values(&ax, &ay, &az);
                   //Data order:
                   //fall status, px, py, qf, ax, ay, az
                   printf("%d,%ld,%ld,%d,%d,%d,%d\n",
                     fall, pos.x, pos.y, pos.qf, ax, ay, az);
                   fall = false; //clear fall bool after transmission
                }
	}
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	dwm_shell_compile();
	dwm_ble_compile();
	dwm_le_compile();
        //N.B. don't need the uart interface as we are only using the shell style mode
        //to printf data.
        //dwm_serial_uart_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
