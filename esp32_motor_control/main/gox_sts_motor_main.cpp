#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "SCServo.h"

#include <iostream>
#include <thread>
#include <chrono>

#include "esp_timer_cxx.hpp"
#include "esp_exception.hpp"

using namespace std;
using namespace idf;
using namespace idf::esp_timer;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19


// Camera servo ID = 1
// Base servo ID = 2
// Hard limits for the cameras after homing with st.CalibrationOfs(1/2) : 
constexpr int kCamServoUpLimit = 1600;
constexpr int kCamServoDownLimit = 2300;
constexpr int kBaseServoLeftLimit = 1200; // assuming forward of the robot is looking opposite to usb ports
constexpr int kBaseServoRightLimit = 2700;

constexpr int kCamServoId{1};
constexpr int kBaseServoId{2};

extern "C" void app_main(void)
{
    try {

    	int cam_pos, base_pos;

    	SMS_STS st;
	    ESP32Serial esp_serial(UART_NUM_1); 
	    esp_serial.begin(1000000, S_RXD, S_TXD);
	    st.pSerial = &esp_serial;    	

		printf("Hello world!\n");

		// st.WritePosEx(1, 1750, 30, 10);//servo(ID1) speed=3400，acc=50，move to position=4095.

		// st.WritePosEx(2, 2000, 30, 10);//servo(ID1) speed=3400，acc=50，move to position=4095.


		int prev_pos = 0;
		bool going_right = false;

		// constexpr int kYawServoOffset
		while(true)
		{

		  cam_pos = st.ReadPos(kCamServoId);
		  if(cam_pos!=-1){
		    printf("camera servo position: %d\n",cam_pos);
		    vTaskDelay(100 / portTICK_PERIOD_MS);
		  }else{
		    printf("read position err\n");
		    vTaskDelay(1000 / portTICK_PERIOD_MS);
		  }	

		  base_pos = st.ReadPos(kBaseServoId);
		  if(base_pos!=-1){
		    printf("base servo position: %d\n",base_pos);

		    // Is within legal limits
		    if((base_pos>=kBaseServoLeftLimit) && (base_pos<=kBaseServoRightLimit))
		    {
		    	if(!going_right)
		    	{
			    	if(base_pos>=(kBaseServoLeftLimit+200))
			    	{
			    		printf("going left!\n");
			    		st.WritePosEx(kBaseServoId, kBaseServoLeftLimit+200, 1000, 50);
			    	}
			    	if(base_pos-prev_pos==0)
			    	{
			    		going_right = (!going_right);
			    	}
		    	}
		    	else
		    	{
			    	if(base_pos<=(kBaseServoRightLimit-200))
			    	{
			    		printf("going right!\n");		    		
			    		st.WritePosEx(kBaseServoId, kBaseServoRightLimit-200, 1000, 50);		    		
			    	}
			    	if(base_pos-prev_pos==0)
			    	{
			    		going_right = (!going_right);
			    	}			    	    		
		    	}


		    }

		    prev_pos = base_pos;
		    vTaskDelay(100 / portTICK_PERIOD_MS);
		  }
		  else{
		    printf("read position err\n");
		    vTaskDelay(1000 / portTICK_PERIOD_MS);
		  }			  			

			vTaskDelay(100 / portTICK_PERIOD_MS);
		}


        printf("Setting up timer to trigger in 500ms\n");
        ESPTimer timer([]() { printf("timeout\n"); });
        timer.start(chrono::microseconds(200 * 1000));

        this_thread::sleep_for(std::chrono::milliseconds(550));

        printf("Setting up timer to trigger periodically every 200ms\n");
        ESPTimer timer2([]() { printf("periodic timeout\n"); });
        timer2.start_periodic(chrono::microseconds(200 * 1000));

        this_thread::sleep_for(std::chrono::milliseconds(1050));
    } catch (const ESPException &e) {
        printf("Exception with error: %d\n", e.error);
    }
    printf("Finished\n");
}