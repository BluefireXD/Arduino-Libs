/*
 Name:		Stepper_A4988.h
 Created:	09.06.2019 14:29:35
 Author:	gpatr
 Editor:	http://www.visualmicro.com
*/

#pragma once
#include <Arduino.h>
#include <digitalWriteFast.h>

#ifndef _Stepper_A4988_h
#define _Stepper_A4988_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif


#define _CStepper_A4988_maxCountOfSteppers (100)  //Set here the max. of Steppers you want to rtun at the same timeAcc. The lower you set this count the faster this lib can be
//A4988 Step-Settings
//MS1 MS2 MS3   Microstep Resolution	Excitation Mode
// L   L   L	Full Step				2 Phase
// H   L   L	Half Step				1 - 2 Phase
// L   H   L	Quarter Step			W1 - 2 Phase
// H   H   L	Eighth Step				2W1 - 2 Phase
// H   H   H	Sixteenth Step			4W1 - 2 Phase

typedef enum stepResolution { FULLSTEP = 1, HALFSTEP = 2, QUARTERSTEP = 4, EIGHTHSTEP = 8, SIXTEENTHSTEP = 16 };

//Arduino UNO,Nano;Mega
//THis part has to be modified if your Board is not Supported. Finoud out in which Register your Controler Stores the Prescaler and what the value mean 
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__)
#define prescalerBin (TCCR0B & 0b00000111)   //TCCR0B is the register for Timer0 and there the last 3 bits declares the prescaler. 
#define getPrescalerDez(p) ((p == 2)? 8 : (p == 3)? 64 : (p == 4)? 256 : (p == 5)? 1024 : 0) // the values are found on page 142: http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
#endif 
#define TIMER0_Resolution_uSec (uint_fast16_t)(getPrescalerDez(prescalerBin)/clockCyclesPerMicrosecond())  //Time_uSec = prescaler* (1/frequenz_MHz) = prescaler/frequenz_Mhz  // frequenz_MHz = frequenz_Hz/1000000  //Attention! this value is fix. If you change prescaler while runntime, it will not update this value!

extern uint_fast16_t SteppersLoopTime;
//extern uint_fast16_t _CStepper_A4988_stepPinMap[_CStepper_A4988_maxCountOfSteppers];



class CStepper_A4988 {

private:
	unsigned long timeLastStep, timeLastAccTick, timeLastDeAccTick, dummy1;
	long currentPos, targetPos, stepsToDo, countOfStepsToDo;		//64 Bit StepPos value	
	uint_fast32_t stepsPerRevolution, speed_StepsPerSec, stepDelay_uSec, finalStepDelay_uSec, accTickTime, deAccTickTime, acceleration, deAcceleration, accSpeedValue, deAccSpeedValue, stepCountForDeAcceleration, doneStepsWhileAcceleration, tempSpeedAcc_StepsPerSec, tempSpeedDeAcc_StepsPerSec;
	uint_fast16_t stepPin, enablePin, dirPin, MS1pin, MS2pin, MS3pin, stepperID;
	int16_t dir;
	unsigned char accState; //8 Bit value 0= no acceleration,1 isaccelerating, 2 = acceleration done
	double stepsPerMM, mmPerRevolution, radiusMM, anglePerStep;
	float timeAcc, timeDeAcc, startTime, dummy2;
	bool useMM_Mode, holdTorque, fastAcc, running, useAcc, useDeAcc;
	stepResolution stepperResolution;
	static uint_fast32_t stepperLoopTime, stepperCount;  //Del
	static CStepper_A4988* steppers[]; //to make it dynamic use map,vecotr etc. 	

public:
	//Constructors and Init
	CStepper_A4988(int_fast16_t enablePin, int_fast16_t stepPin, int_fast16_t dirPin, int_fast16_t MS1pin, int_fast16_t MS2pin, int_fast16_t MS3pin, stepResolution res, double anglePerStep, double speed_revolutionPerMin, bool holdTorque);
	/*This Constructor stepper who drive a wheel (for ex. of  a car) The radius is the radisu of the wheel*/
	CStepper_A4988(int_fast16_t enablePin, int_fast16_t stepPin, int_fast16_t dirPin, int_fast16_t MS1pin, int_fast16_t MS2pin, int_fast16_t MS3pin, stepResolution res, double anglePerStep, double speed_revolutionPerMin, bool holdTorque, double radius);
	void init();

	//Setters
	void setStepperResolution(stepResolution res);
	void setSpeed_StepsPerSec(uint_fast32_t steps);
	void setSpeed_RevolutionPerMin(double rev);
	void setSpeed_RevolutionPerSec(double rev);
	void setSpeed_mmPerSec(double mm);
	void setSpeed_cmPerSec(double cm);
	void setSpeed_mPerSec(double m);
	void setSpeed_kmPerHour(double km);
	void setStepsPerMM(int_fast32_t steps);
	void setAnglePerStep(double angle);
	void setMilimeterMode(bool stateMM, double wheelRadius); //Is for using steppers as car motor etc
	void setRadius_mm(double r);

	void setHoldTorque(bool t);
	void setCurrentPos(long pos);
	void setCurrentPosMM(double pos);
	void setfastAcc(bool fastAcc);
	void setAcceleration(uint_fast32_t acc, bool useDeAcc);
	void setAcceleration_MM(uint_fast32_t acc, bool useDeAcc);
	void setAcceleration_CM(uint_fast32_t acc, bool useDeAcc);
	void setAcceleration_M(uint_fast32_t acc, bool useDeAcc);
	void setAcceleration_Sec(double sec, bool useDeAcc);
	void setAcceleration_uSec(double usec, bool useDeAcc);
	//Not supported yet
	/*void setDeAcceleration(uint_fast32_t deAcc);
	void setDeAcceleration_uSec(double deAcc);
	void setDeAcceleration_MM(uint_fast32_t deAcc);
	void setDeAcceleration_CM(uint_fast32_t deAcc);
	void setDeAcceleration_M(uint_fast32_t deAcc);
	*/


	//Getters
	uint_fast32_t getSpeed_StepsPerSec();
	double getSpeed_mmPerSec();
	double getSpeed_cmPerSec();
	double getSpeed_mPerSec();
	double getSpeed_kmPerSec();
	uint_fast32_t getSpeed_RevolutionPerSec();
	uint_fast32_t getSpeed_RevolutionPerMin();
	long getCurrentPos();
	long getCurrentPosMM();
	double getAnglePerStep();
	uint_fast32_t getStepsPerRevolution();
	int_fast16_t getDir();
	bool getHoldTorque();
	double getRadiusMM();
	long getStepsToGo();
	double getstepsPerMM();
	double getMMPerRevolution();
	unsigned long getStepDelay_uSec();
	uint_fast16_t getStepperID();
	bool getfastAcc();
	stepResolution getResolutiuon();

	//Others
	static void updateSteppers();
	void runOneStep(uint16_t stepPin, int16_t dir);
	void run(const uint_fast16_t stepPin_);
	void drive_mm(double mm, bool holdTorque);
	void drive_cm(double cm, bool holdTorque);
	void drive_m(double m, bool holdTorque);
	void runSteps(long steps, bool holdTorque);
	void goToPos(long pos, bool holdTorque);
	void calcStepDelay();
	bool isRunning();
	void updateCurrentPos();
	void powerOn();
	void powerOff();
	void demoRevolution();
	void calcValues();
	void stop();
	void start();
	void continue_();
	void playGCodeMusic(String gCode);

private:
	void setDir(int_fast16_t dir);

};
