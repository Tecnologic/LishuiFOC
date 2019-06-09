//place all your personal configurations here and keep this file when updating!
#ifndef CONFIG_H
#define CONFIG_H
#include "stdint.h"

#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_901U

#define wheel_circumference 2.202 	//wheel circumference in m
#define THROTTE_OFFSET 200//1210 			//ADC-value at closed throttle 670 for throttle, 1255 for TS
#define TS_COEF 2000				//coefficient for torque-sensor-mode
//#define TS_MODE						//Torquesensor-Mode
#define PAS_TIMEOUT 12000			//time tics @ 16kHz untils motor stops
#define RAMP_END 4000					//time tics @ 16kHz where motor reaches full level power
#define PH_CURRENT_MAX 300			//iq value (phase current in rotating frame), not calibrated yet

#define P_FACTOR_I_Q 1L				//proportional factor for PI control of iq
#define I_FACTOR_I_Q 0.1L			//integral factor for PI control of iq
#define P_FACTOR_I_D 1L				//proportional factor for PI control of id
#define I_FACTOR_I_D 1L				//integral factor for PI control of id

#define P_FACTOR_E_D 0.0001L				//proportional factor for PI control of ed
#define I_FACTOR_E_D 0.00001L			//integral factor for PI control of ed

#define SPEC_ANGLE  -357913941L		//motor specific angle, refer to chapter 8.3.3 of UM1052.shengyi:-715827882L, 715827882 536870912 357913941L; //357913941 298261617 119304647L // 30� BionX IGH3 motor specific angle, refer to chapter 8.8.3 of UM1052 180� maped to 2^31

#define OFFSET_A 1025 				//Offset of current sensing phase A
#define OFFSET_B 1022				//Offset of current sensing phase B
#define OFFSET_C 1042				//Offset of current sensing phase C

#define CAL_V 7LL//  *10^6  0.000007				// 1V / 40 digits ADC, *1/Sqrt(3) /2048 max DutyCycle
#define CAL_I 5LL// *10^2   0.05					// 1A / 20 digits ADC

//Constants for Motor model of observer an speed PLL
#define INDUCTANCE 12LL//  *10^4 0.0012			//H = V*s/A
#define RESISTANCE 3500LL// *10^4  0.35				//Ohm = V/A
#define FLUX_LINKAGE 17000LL// *10^6  0.017			//V*s/rad
#define GAMMA 16LL


#define _T 2048

#endif
