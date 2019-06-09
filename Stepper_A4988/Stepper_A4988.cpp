/*
 Name:		Stepper_A4988.cpp
 Created:	09.06.2019 14:29:35
 Author:	gpatr
 Editor:	http://www.visualmicro.com
*/

#include "Stepper_A4988.h"

uint_fast32_t CStepper_A4988::stepperCount = 0;
uint_fast32_t CStepper_A4988::stepperLoopTime = 20; //testvalue = 30. calculate real value with runing 1 timeAcc trought demorun()

CStepper_A4988* CStepper_A4988::steppers[_CStepper_A4988_maxCountOfSteppers];

/*default constructor with all needed params*/
CStepper_A4988::CStepper_A4988(int_fast16_t enablePin, int_fast16_t stepPin, int_fast16_t dirPin, int_fast16_t MS1pin, int_fast16_t MS2pin, int_fast16_t MS3pin, stepResolution res, double anglePerStep, double speed_revolutionPerMin, bool holdTorque) {

	this->enablePin = enablePin;
	this->stepPin = stepPin;
	this->dirPin = dirPin;
	this->MS1pin = MS1pin;
	this->MS2pin = MS2pin;
	this->MS3pin = MS3pin;
	this->stepperResolution = res;
	this->anglePerStep = anglePerStep;
	this->holdTorque = holdTorque;
	this->stepsPerRevolution = (360.0f / anglePerStep) * (int_fast16_t)stepperResolution;
	stepperID = stepperCount;  //use the stepper count as unique stepperId
	stepperCount++;	//increase stepper count 
	useMM_Mode = false;
	//If a loop is use to update stepper add the every stepper to the array. Attention! Using a loop with references is slower then calling the stepper directly from the loop() Funktion via static variable					
	steppers[stepperCount] = this;

}

/*Use this constructor if your stepper is driving a wheel. It will enable the calulcations for distances (mm,cm,km) instdead of steps.
After using this constructor you can use methods like "drive_mm(double mm, bool holdTorque)" etc.*/
CStepper_A4988::CStepper_A4988(int_fast16_t enablePin, int_fast16_t stepPin, int_fast16_t dirPin, int_fast16_t MS1pin, int_fast16_t MS2pin, int_fast16_t MS3pin, stepResolution res, double anglePerStep, double speed_revolutionPerMin, bool holdTorque, double radius) : CStepper_A4988(enablePin, stepPin, dirPin, MS1pin, MS2pin, MS3pin, res, anglePerStep, speed_revolutionPerMin, holdTorque) { //use speed_mmPerSec instead of speed_StepsPerSec because it will be recalulated  after init() function anyway

	this->radiusMM = radius;
	useMM_Mode = true;  //If MM_Mode is used (MM = millimeter) 

}


void CStepper_A4988::init() {

	Serial.println("Do Init");

	//init pins
	Serial.println("Running Steppers: " + String(stepperCount));
	pinModeFast(enablePin, OUTPUT);
	pinModeFast(stepPin, OUTPUT);
	pinModeFast(dirPin, OUTPUT);
	pinModeFast(MS1pin, OUTPUT);
	pinModeFast(MS2pin, OUTPUT);
	pinModeFast(MS3pin, OUTPUT);

	//Important to set all Output Pins to LOW!!! else the wire will work as antenna and steppers will do crazy stuff by receiving frequenzy stuff overe the cable antenna
	digitalWriteFast(enablePin, LOW);
	digitalWriteFast(stepPin, LOW);
	digitalWriteFast(dirPin, LOW);
	digitalWriteFast(MS1pin, LOW);
	digitalWriteFast(MS2pin, LOW);
	digitalWriteFast(MS3pin, LOW);
	//Init varaibles
	useAcc = useDeAcc = running = false;
	accState = 0;
	fastAcc = true;
	stepsToDo = 0;
	targetPos = 0;
	currentPos = 0; //Imnportant to init this with correct pos. default is 0 but if the stepper is already on a speical pos then use after init() the setCurrentPos() or setCurrentPosMM() methode

	//Setup some basic stuff
	setHoldTorque(holdTorque); //First powerOff or powerOn steppers by torque value. To save power disable torque!	
	setDir(1); //set a init default value forward. Function Call needeed to set pin High/LOW	
	setStepperResolution(stepperResolution); //calls calcValues() call it the very first timeAcc only in intit() after initialize pins ...before it makes no sense! 
}

/*
Using this method instead of calling the run() of every steper in loop() is slower because it's using stepPin as a variable which is slow in digitalWriteFast(). Literals are faster which the run use.
The fastest way is using the run(stepPin) method
*/
void CStepper_A4988::updateSteppers() {
	unsigned long startTime = micros();
	for (int_fast16_t i = 0; i <= stepperCount - 1; i++)
	{
		steppers[i]->run(steppers[i]->stepPin);  //Slow way using variables instead of literals,
	}
	stepperLoopTime = micros() - startTime;
	Serial.println("LoopTime: " + String(stepperLoopTime));
}

/*
uint32_t maxSpeed = 54400; // Steps/s
uint32_t acc = 54400; //Steps/s²
uint32_t tickTime2 = 10000; //µS
float startSpeed = 1000.0;
float timeAcc = startSpeed/maxSpeed;//0.0;	//µS
float tickTimeToAdd = tickTime2 / 1000000.0;
uint16_t speedToAdd = 32;*/
bool LED13enabled = false;
unsigned long firstTime = 0;
unsigned long time2 = 0;
uint32_t counter = 0;
uint32_t max = 0;
uint32_t delayThere = 0;


void CStepper_A4988::run(const uint_fast16_t stepPin_) {

	unsigned long currentTimeMicros = micros(); // micros(); //save timeAcc here so that calcualtions timeAcc will not be included in our current micros and make wrong ticktimes etc... Remind that this call already need 2-4 uSec somehow on Arduino Uno.

	if (stepsToDo > 0) {
		//Doing Steps
		if (currentTimeMicros - timeLastStep >= stepDelay_uSec) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {		//Serial.println("Run: " + String(stepDelay_uSec));
			digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
			stepsToDo--;						//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
			timeLastStep = currentTimeMicros;	//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
			if (accState == 1)
				doneStepsWhileAcceleration++;
			digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?		
		}
		//Live Acceleration Calculation
		if (accState == 1) {
			//unsigned long start = micros();
			//FastAcc is ~2 times faster then not FastAcc is used because low MicroControlers with 8Bit CPU and no FPU will be fucked up with 1/f -> 1000000 / V calculations
			if (fastAcc && currentTimeMicros - timeLastAccTick >= accTickTime) {  //fastAcceleration uses increases every x µS the Speed
				tempSpeedAcc_StepsPerSec += accSpeedValue;//1/V = StepDelay in Sec.
				timeLastAccTick = currentTimeMicros; //timeLastAccTick = micros();
				stepDelay_uSec = 1000000 / tempSpeedAcc_StepsPerSec;	//t = 1/f	(f = V)	
				//Todo: A S-curve would be nice
			}
			else if (!fastAcc) {
				timeAcc += (currentTimeMicros - timeLastAccTick) / 1000000.0; //tickTimeToAdd;																		   //Serial.println("time1: " + String(timeAcc * 100));
				timeLastAccTick = currentTimeMicros;
				if (timeAcc > startTime) //because this acceleration stuff is executed imidiatly and isnt waiting until the timeAcc is over of the fist stepdelay, we have to set the init value of timeAcc negative to force here the wait for beginning. So if timeAcc gets bigger 0 it has reached the timeAcc which was needed for the first step 
					stepDelay_uSec = 1000000 / (acceleration * timeAcc);// V = a*t; d = 1000000/V; -> d = 1000000/(a*t)
				//Todo: A S-curve would be nice			
			}
			//MAKES PROBLEMS. test again if fixed
			if (stepDelay_uSec <= finalStepDelay_uSec) { //because the calculated stepdelay depends on timeAcc and can be calculated a bit to late means its also possible that we accelerate to much. to avoid this we check this here and assinged the correct value ifg it happends
				stepDelay_uSec = finalStepDelay_uSec;
				accState = 2;  //2 = Acc is done	
				Serial.println("doneStepsWhileAcceleration " + String(doneStepsWhileAcceleration));
			}
		}
		//Live DeAcceleration Calculation
		if (useDeAcc) {
			if (accState == 1 && stepsToDo < stepCountForDeAcceleration) {
				accState = 2;  //is needed to be set here too because if stepsToDo <= stepCountForDeAcceleration then acceleration has to be stoped and deAcc has to start
				tempSpeedDeAcc_StepsPerSec = 1000000 / stepDelay_uSec;
				Serial.println("notenoughtSteps " + String(doneStepsWhileAcceleration));
				doneStepsWhileAcceleration = stepCountForDeAcceleration;
			}

			if (stepsToDo < doneStepsWhileAcceleration) {
				if (fastAcc && currentTimeMicros - timeLastDeAccTick >= deAccTickTime) {  //fastAcceleration uses increases every x µS the Speed
					tempSpeedDeAcc_StepsPerSec -= deAccSpeedValue;//1/V = StepDelay in Sec.
					timeLastDeAccTick = currentTimeMicros; //timeLastAccTick = micros();
					stepDelay_uSec = 1000000 / tempSpeedDeAcc_StepsPerSec;	//t = 1/f	(f = V)	
					//if(stepDelay_uSec < 250)
					//	Serial.println("stepDelay_uSecssss " + String(stepDelay_uSec));
					//Todo: A S-curve would be nice
				}
				else if (!fastAcc) { //Todo: Fix because stepDelay_uSec calculation takes more time then in Acc and also if µC differs there will be always wrong values. Use fastAcc always for DeAcc!!!! 
					if (timeLastDeAccTick == 0)
						timeLastDeAccTick = currentTimeMicros;  //init here the fist time when Acc starts
					timeDeAcc += (currentTimeMicros - timeLastDeAccTick) / 1000000.0; //tickTimeToAdd;																		   //Serial.println("time1: " + String(timeAcc * 100));
					timeLastDeAccTick = currentTimeMicros;
					stepDelay_uSec = 1000000 / (speed_StepsPerSec - (deAcceleration * timeDeAcc));// V = a*t; d = 1000000/V; -> d = 1000000/(a*t)
					//Serial.println("StepsToDO " + String(stepsToDo));
				}
			}
		}
		//else
			//Serial.println("Problem");

	}

	//Stop after all steps are done
	else if (stepsToDo == 0 && running) {   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
		stop(); //calcs also the currentPos		
		Serial.println("NoStepDelayChange x times: " + String(counter));
		Serial.println("Max AccValue: " + String(max));
		Serial.println("Delay: " + String(delayThere));
	}

}



void CStepper_A4988::calcValues() {
	stepsPerRevolution = (360.0f / anglePerStep) * (int_fast16_t)stepperResolution; /*Attention double in int conversion: data loss! But currently no other way*/
	Serial.println("StepsPerRevolution: " + String(stepsPerRevolution));
	if (useMM_Mode) {
		mmPerRevolution = radiusMM * 2.0f * PI;
		stepsPerMM = stepsPerRevolution / mmPerRevolution;
		Serial.println("mmPerRevolution: " + String(mmPerRevolution));
		Serial.println("stepsPerMM: " + String(stepsPerMM));
	}
	updateCurrentPos();
	calcStepDelay();  //optional needed to allowe a live angle or stepper-resolution update in cases where motor is running
}




int32_t Percent = 30;
void CStepper_A4988::calcStepDelay() {
	unsigned long start = micros();
	finalStepDelay_uSec = (speed_StepsPerSec > 0) ? 1000000 / speed_StepsPerSec : 1000000;	//slowest speed = 1 Step/s! If a lower speed is needed then use double for speed_StepsPerSec 

	if (useAcc) { 	//Check if an acceleration/deacceleration has to be calculated			
		/*Notify that modifications while "running" are unfinished and not supported yet. We need to find out how many steps are done by an deacceleration. Without knowing how many steps are needed its not possible to find out on which step count the deacceleration has to start. currently workarround is to count the steps of an acceleration and use this stepcount for the deacceleration too therefore its not possible to do accelerations/deaccelerations at runntime of stepermotors*/
		doneStepsWhileAcceleration = 0;
		accState = 1;
		stepDelay_uSec = (running) ? stepDelay_uSec : 1000000 * sqrt(1.0 / acceleration); //check if stepper is already running. if yes then take the current speed and calcualte the accerleration to the next higher speed 
		Serial.println("stepDelay_uSecxxxxxxxxxx: " + String(running));
		if (fastAcc) {
			tempSpeedAcc_StepsPerSec = 1000000 / stepDelay_uSec;
			accSpeedValue = 1;
			/*Notify that modifications while "running" are unfinished and not supported yet. We need to find out how many steps are done by an deacceleration. Without knowing how many steps are needed its not possible to find out on which step count the deacceleration has to start. currently workarround is to count the steps of an acceleration and use this stepcount for the deacceleration too therefore its not possible to do accelerations/deaccelerations at runntime of stepermotors*/
			if (running)
				accTickTime = (1000000 - (1000000 / stepDelay_uSec)) / acceleration; //find t where V = 1 so that we can increase the speed +1 on every tick means 1 = a*t+V0 -> t= (1-V0)/a     (V0 = 1000000 / stepDelay_uSec) 
			else
				accTickTime = 1000000 / acceleration; //find t where V = 1 means 1 = a*t -> t= 1/a //acceleration*(accTickTime / 10000000.0);
			if (accTickTime < stepperCount * stepperLoopTime) {  //check if accTickTime is smaller then the stepperCount*stepperLoopTime. If yes then multiply the TickTIme and  accSpeedValue x times until accTickTime > stepperCount*stepperLoopTime
				uint32_t multi = ((stepperCount * stepperLoopTime) / accSpeedValue) + 1;  //1 because we want to use int and not floats we will cut the floting numbers and add +1 to be always higher then the cutted floating number -> float 1,999 -> 1 -> 1+1 = 2
				accSpeedValue *= multi;
				accTickTime *= multi;
				Serial.println("multi: " + String(multi));
			}
			Serial.println("accSpeedValue: " + String(accSpeedValue));
			Serial.println("accTickTime: " + String(accTickTime));
		}
		else {
			timeAcc = 0;
			startTime = stepDelay_uSec / 1000000.0;
		}
	}
	else {  //If no Acceleration			
		stepDelay_uSec = 1000000 / speed_StepsPerSec;
		accState = 0;
	}
	if (useDeAcc) {
		//Currentelly using acceleration in same way for deacceleration beacuse there is no way implemented to find out ho many steps the deacceleration need Means we dont know on which stepcount we have to start with the deacceleration. Currently the Seps of acceleration is counted and used for the deaccelration too. Therefore deacc = acc here
		timeLastDeAccTick = 0;  //inportant. init here with 0 so that i cant be re-init later in the run() if deAcc starts
		stepCountForDeAcceleration = stepsToDo / 2;
		if (fastAcc) {
			deAcceleration = acceleration;
			deAccTickTime = accTickTime;
			deAccSpeedValue = accSpeedValue;
			//set here half of stepsToDo  because if the acceleration needs mor steps then the half, the deacceleration has already to start or there will be not enought steps left for DeAcc
			tempSpeedDeAcc_StepsPerSec = 1000000 / finalStepDelay_uSec;
		}
		else {
			timeDeAcc = 0.0;
			deAcceleration = acceleration / 2;  //dirty fix! Needed to make deAcc (in mode without fastAcc) always slower then Acc else not all steps will be done because DeAcc takes more time :/ -> Checkout run().. if still not working increase the divisor
		}
	}
	//} else {
		//stepDelay_uSec = 1000; //set a default lowest dummy value if there are no steps to go and no motor is running.. not good but better then unhandeled
		//Serial.println("Error! Could not update speed (delay)! Missing stepsToDo");
		//Serial.println("stepsToDo: " + String(stepsToDo));
	//}
	Serial.println("finalStepDelay_uSec: " + String(finalStepDelay_uSec));
	Serial.println("stepDelay_uSec: " + String(stepDelay_uSec));
	unsigned short a = 258;
	unsigned short b = 250;

	//	Serial.println("Clac: " + String((unsigned char)(a-b)));

			//Serial.println("accTicksPerProfile: " + String(accTicksPerProfile));	
			//Serial.println("pow: " + String(pow(multi, profileCount)));
			//Serial.println((int_fast32_t)stepDelay_uSec);
}

/*
A4988 Step-Settings
Call it the first timeAcc only after init() was called. init() will set up the ports to Inputs/Outputs. Without port initialisation it makes no sense to set StepperResolution!
MS1 MS2 MS3   Microstep Resolution	Excitation Mode
 L   L   L	Full Step				2 Phase
 H   L   L	Half Step				1 - 2 Phase
 L   H   L	Quarter Step			W1 - 2 Phase
 H   H   L	Eighth Step				2W1 - 2 Phase
 H   H   H	Sixteenth Step			4W1 - 2 Phase
*/
void CStepper_A4988::setStepperResolution(stepResolution res) {


	if (res == FULLSTEP || res == HALFSTEP || res == QUARTERSTEP || res == SIXTEENTHSTEP) {
		stepperResolution = res;
	}
	switch (res) {
	case FULLSTEP:
		digitalWriteFast(MS1pin, LOW); digitalWriteFast(MS2pin, LOW); digitalWriteFast(MS3pin, LOW);
		break;
	case HALFSTEP:
		digitalWriteFast(MS1pin, HIGH); digitalWriteFast(MS2pin, LOW); digitalWriteFast(MS3pin, LOW);
		break;
	case QUARTERSTEP:
		digitalWriteFast(MS1pin, LOW); digitalWriteFast(MS2pin, HIGH); digitalWriteFast(MS3pin, LOW);
		break;
	case EIGHTHSTEP:
		digitalWriteFast(MS1pin, HIGH); digitalWriteFast(MS2pin, HIGH); digitalWriteFast(MS3pin, LOW);
		break;
	case SIXTEENTHSTEP:
		digitalWriteFast(MS1pin, HIGH); digitalWriteFast(MS2pin, HIGH); digitalWriteFast(MS3pin, HIGH);
		break;
	default:  //Use 16 Step mode as default.. it's in general the smoothest
		digitalWriteFast(MS1pin, HIGH); digitalWriteFast(MS2pin, HIGH); digitalWriteFast(MS3pin, HIGH);
		break;
	}

	if (isRunning()) {
		stop();  //is it an good idea to stop there? or better change values while running?
		calcValues();
		continue_();
	}
	else
		calcValues();

}

void CStepper_A4988::runOneStep(uint16_t stepPin, int16_t dir) {
	setDir((dir > 0) ? 1 : -1);//set the right direction by checking if a positive or negative step count is given
	//powerOn();
	digitalWriteFast(stepPin, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
	currentPos += this->dir;
	digitalWriteFast(stepPin, LOW); //ggf über Bitmask arbeiten da schneller?	
	//if (!holdTorque)
	//	powerOff();
}

void CStepper_A4988::runSteps(long steps, bool holdTorque) {
	//	/ToDO: if(!running){  //Hier nur StepsToDO erhöhen wenn schon am laufen
	updateCurrentPos();
	targetPos += currentPos + steps; //also needed to be update so that later the currentPos gets updated too;  if there is no postogo set the current pos wil not get updated on finishing run() or calling stop()
	setDir((steps > 0) ? 1 : -1);//set the right direction by checking if a positive or negative step count is given
	stepsToDo += abs(steps);
	this->holdTorque = holdTorque;
	if (!running)
		start();
	//isRunning = true;  not needed anymore ...//this will let the run method start^^
}

void CStepper_A4988::goToPos(long pos, bool holdTorque) {
	runSteps(abs(pos - currentPos), holdTorque);
}

void CStepper_A4988::drive_mm(double mm, bool holdTorque) {
	runSteps(mm * stepsPerMM, holdTorque);
}

void CStepper_A4988::drive_cm(double cm, bool holdTorque) {
	runSteps(cm * 10 * stepsPerMM, holdTorque);
}

void CStepper_A4988::drive_m(double m, bool holdTorque) {
	runSteps(m * 1000 * stepsPerMM, holdTorque);
}

void CStepper_A4988::setSpeed_RevolutionPerMin(double rev) {
	setSpeed_StepsPerSec((rev / 60.0f) * stepsPerRevolution);
}

void CStepper_A4988::setSpeed_RevolutionPerSec(double rev) {
	setSpeed_StepsPerSec(rev * stepsPerRevolution);
}

/*Max speed_StepsPerSec = 1000000 StepsPerSecond = 1µS delay between 1 Step
  Min speed_StepsPerSec = 1 If lower Speed is needed then use double for speed_StepsPerSec and finalspeed_StepsPerSec*/
void CStepper_A4988::setSpeed_StepsPerSec(uint_fast32_t steps) {
	speed_StepsPerSec = steps;	//never alow 0 it will make trouble later wehre 1000000/speed_StepsPerSec is calculated
	calcStepDelay();
}

void CStepper_A4988::setSpeed_mmPerSec(double mm) {
	setSpeed_StepsPerSec(mm * stepsPerMM);
}

void CStepper_A4988::setSpeed_cmPerSec(double cm) {
	setSpeed_mmPerSec(cm * 10);
}

void CStepper_A4988::setSpeed_mPerSec(double m) {
	setSpeed_mmPerSec(m * 1000);
}

void CStepper_A4988::setSpeed_kmPerHour(double km) {
	setSpeed_mmPerSec((km / 216000.0f) * 1000000.0f);
}

void CStepper_A4988::setAnglePerStep(double angle) {
	anglePerStep = angle;
	calcValues();
}

void CStepper_A4988::setStepsPerMM(int_fast32_t steps) {
	stepsPerMM = steps;
	calcValues();
}

void CStepper_A4988::setHoldTorque(bool t) {
	holdTorque = t;
	if (holdTorque)  //enable/disable stepper
		powerOn();
	else
		powerOff();
}

/*Set direction of rotation.
-1 = backwards, 1 = forward*/
void CStepper_A4988::setDir(int_fast16_t dir) {
	if (this->dir != dir) { //Only Update if dir changed. this function gets not onyl called if dir changes)
		if (running)
			stop(); //cant change dir if he is running. so stop and update pos
		else
			updateCurrentPos();//update current stepPos before change direction
		this->dir = (dir < 0) ? -1 : 1; //make sure that dir is always the number 1 or -1 because its used for currentPos calculation
		digitalWriteFast(dirPin, (dir == 1) ? HIGH : LOW);
		Serial.println("setDir " + String(this->dir));
	}
}

/*USe this if you are powering wheels with your stepper and want to drive a distance in milimeters (use later drive_mm() drive_cm() etc functions)*/
void CStepper_A4988::setMilimeterMode(bool stateMM, double wheelRadius) {
	useMM_Mode = stateMM;
	radiusMM = wheelRadius;
	calcValues();  //recalculate important values
}

/*If you stepper is direcly driving a wheel then you cna set here the radius of the wheel*/
void CStepper_A4988::setRadius_mm(double r) {
	radiusMM = r;
	calcValues();  //Also update important values which are used in calculations!!! 
}

void CStepper_A4988::setCurrentPos(long pos) {
	stop();  //not really needed but if somone change the currentpos while stepper is running it's maybe better to stop the stepper so that the user can see that he changed something. else a change in currentpos will not affect the values because the currentpos gets fist calculcated if the run() is finish
	currentPos = pos;
}

/*Attention double in int conversion: data loss!*/
void CStepper_A4988::setCurrentPosMM(double pos) {
	setCurrentPos(pos * stepsPerMM);
}

/*Sets Accleration in Steps/s². 0=no acceleration. Means steppers start with full speed which you set. Notify! Currently DeAcceleration use the same Values as Acceleration.*/
void CStepper_A4988::setAcceleration(uint_fast32_t acc, bool useDeAcc) {
	if (acc > 0) {
		useAcc = true;
		this->useDeAcc = useDeAcc;
	}
	acceleration = acc;
	deAcceleration = acceleration;//currently deAcc is same like Acc because no other calulation is implemented yet
	calcStepDelay();
}

/*Set Acceleration in MilliMeter/s²*/
void CStepper_A4988::setAcceleration_MM(uint_fast32_t acc, bool useDeAcc) {
	setAcceleration(stepsPerMM * acc, useDeAcc);
}

/*Set Acceleration in CentiMeter/s²*/
void CStepper_A4988::setAcceleration_CM(uint_fast32_t acc, bool useDeAcc) {
	setAcceleration(stepsPerMM * 10 * acc, useDeAcc);
}
/*Set Acceleration in Meter/s²*/
void CStepper_A4988::setAcceleration_M(uint_fast32_t acc, bool useDeAcc) {
	setAcceleration(stepsPerMM * 1000 * acc, useDeAcc);
}

/*Set the duration for an acceleration in Sec.*/
void CStepper_A4988::setAcceleration_Sec(double sec, bool useDeAcc) {
	setAcceleration(speed_StepsPerSec / sec, useDeAcc);				//double->int!!!! data loose
}

//Set the duration for an acceleration in uSec.*/
void CStepper_A4988::setAcceleration_uSec(double usec, bool useDeAcc) {
	setAcceleration(speed_StepsPerSec / (usec / 1000000.0f), useDeAcc);  //double->int!!!! data loose
}

/*Enables Deacceleration. Notify! Currently DeAcceleration use the same Values as Acceleration. Meansif there is no Acceleration already set Deacceleration will be dissabled always. Equal if it gets enabled here*/



/*
Not Supported yet
//Set DeAcceleration in MilliMeter/s²
void CStepper_A4988::setDeAcceleration(uint_fast32_t deAcc) {
	deAcceleration = deAcc;
}
void CStepper_A4988::setDeAcceleration_uSec(double usec) {
	setDeAcceleration(speed_StepsPerSec / (usec / 1000000.0f));  //double->int!!!! data loose
}
void CStepper_A4988::setDeAcceleration_MM(uint_fast32_t deAcc) {
	setDeAcceleration(stepsPerMM*deAcc);
}

//Set DeAcceleration in CentiMeter/s²
void CStepper_A4988::setDeAcceleration_CM(uint_fast32_t deAcc) {
	setDeAcceleration(stepsPerMM * 10 * deAcc);
}
//Set DeAcceleration in Meter/s²
void CStepper_A4988::setDeAcceleration_M(uint_fast32_t deAcc) {
	setDeAcceleration(stepsPerMM * 1000 * deAcc);
}
*/
void CStepper_A4988::setfastAcc(bool fastAcc) {
	this->fastAcc = fastAcc;
}

void CStepper_A4988::powerOn() {
	digitalWriteFast(enablePin, LOW);
}

void CStepper_A4988::powerOff() {
	digitalWriteFast(enablePin, HIGH);
}

/*Important function!
While a stepper is running we are not updating the current pos because it's a calculation of datatype "long" which is very slow on 8 Bit controllers like the Arduino Uno!
So before using the currentPos Variable make always sure that it has beed updated with this function!
*/
void CStepper_A4988::updateCurrentPos() {
	currentPos += (abs(targetPos - stepsToDo) * dir);
}


bool CStepper_A4988::isRunning() {
	return running;
}


void CStepper_A4988::start() {
	calcStepDelay();  //important do reset all acceleration stuff etc
	running = true; //do this after calcStepDelay!!!!!!!! This bool is indicator if stepper is already running. if start() is called then stepper was not running before. so set it after!
	powerOn();
	unsigned long micros_ = micros();
	timeLastStep = micros_;//micros() + stepDelay_uSec; //Very important to do that here!!!! here the new timeLastStep must be set else it will count wrong values in the run() method 
	timeLastAccTick = micros_; //(acceleration > 0 && fastAcc) ? micros_ + stepDelay_uSec : micros_;  //If fastAcc is used then 
}

void CStepper_A4988::stop() {
	running = false;
	updateCurrentPos();
	//Todo: deacceleration!
	//setHoldTorque(holdTorque); //set motor power on or off
	if (!holdTorque) //set motor power off if no torque is used
		powerOff();
	stepsToDo = 0; //reset after currentStepPos is calculated

}

void CStepper_A4988::continue_() {
	runSteps((abs(currentPos - targetPos) * dir), holdTorque);
	//Todo: reset acceleration? Maybe add param to this method so that user can decide if he want accelerate the continue
}

void CStepper_A4988::demoRevolution() {
	runSteps(getStepsPerRevolution(), false);
}

/*Plays Music on the Steppers. Takes Midi-Music which is converted into G-Code. To convert Midi to G-Code use: https://www.ultimatesolver.com/de/midi2gcode*/
void CStepper_A4988::playGCodeMusic(String gCode) {
	//todo. Write G_Code Interpreter: To convert Midi to G-Code use: https://www.ultimatesolver.com/de/midi2gcode
}

long CStepper_A4988::getCurrentPos() {
	updateCurrentPos();
	return currentPos;
}


/* */
long CStepper_A4988::getCurrentPosMM() { return (getCurrentPos() / stepsPerMM); }  //Rechnung urrentStepPos/stepsPerRevolution *mmPerRevolution   achtung wegen int64 Überlauf!!! Gerade wenn die Stepper zum Fahren benutzt werden wird hier müll zurückgegeben!
uint_fast32_t CStepper_A4988::getSpeed_StepsPerSec() { return speed_StepsPerSec; }
double CStepper_A4988::getSpeed_mmPerSec() { return stepsPerMM * speed_StepsPerSec; }
double CStepper_A4988::getSpeed_cmPerSec() { return speed_StepsPerSec * stepsPerMM * 10; }
double CStepper_A4988::getSpeed_mPerSec() { return speed_StepsPerSec * stepsPerMM * 1000; }
double CStepper_A4988::getSpeed_kmPerSec() { return speed_StepsPerSec * stepsPerMM * 1000000; }
double CStepper_A4988::getAnglePerStep() { return anglePerStep; }
uint_fast32_t CStepper_A4988::getStepsPerRevolution() { return stepsPerRevolution; }
double CStepper_A4988::getRadiusMM() { return radiusMM; }
int_fast16_t CStepper_A4988::getDir() { return dir; }
uint_fast32_t CStepper_A4988::getSpeed_RevolutionPerSec() { return speed_StepsPerSec / stepsPerRevolution; }
uint_fast32_t CStepper_A4988::getSpeed_RevolutionPerMin() { return (speed_StepsPerSec * 60) / stepsPerRevolution; }
long CStepper_A4988::getStepsToGo() { return stepsToDo; }
double CStepper_A4988::getstepsPerMM() { return stepsPerMM; }
double CStepper_A4988::getMMPerRevolution() { return mmPerRevolution; }
unsigned long CStepper_A4988::getStepDelay_uSec() { return stepDelay_uSec; }
bool  CStepper_A4988::getHoldTorque() { return holdTorque; }
uint_fast16_t CStepper_A4988::getStepperID() { return stepperID; }
bool CStepper_A4988::getfastAcc() { return fastAcc; }
stepResolution CStepper_A4988::getResolutiuon() { return stepperResolution; };

/*Do a simulation of run() method to get the adjustment delay
Attention!!!TestPin = unused Pin or one of the Pins which are used for the StepperDriver A4988!!!
This pin will get a HIGH level (mostly 5V) for a few uSec!!!
*/
/*
void CStepper_A4988::calcAdjustmentDelay(int_fast16_t testPin) {
	unsigned long dummy = 0, currentTimeMicros = 0;
	pinModeFast(testPin, OUTPUT);
	currentTimeMicros  = micros(); //this call has a resolution of 4 uSec!!! values for delay are 2-6 uSec. We use the average of 3 uSec
	if (dummy == 0) { dummy++; }  //do a unsinged long compare and counting like in run() methode
	digitalWriteFast(testPin, HIGH);  //do a digitalWirte simlulation on a test pin
	digitalWriteFast(testPin, LOW);
	adjustDelay = (micros() - currentTimeMicros);
	adjustDelay = (adjustDelay >= 0) ? adjustDelay*stepperCount : 0;  //do in a second calculation for the count of steppers

}

uint_fast32_t CStepper_A4988::getAdjustDelay() {
	return adjustDelay;
}

*/

/*
long CStepper_A4988::getDelayFixValue() {
	unsigned long uS = micros();
	micros();
	return micros() - uS;
	//return delayFixValuePerLoop;
}


//Iwas geht hier nicht die ermittlelte delay ist anderst wie wenn man sie manuell setzt
unsigned long uS = 0;
void CStepper_A4988::autoSetDelayFixValue() {
	uS = micros();
	delayFixValuePerLoop = (micros() - uS)/2;
}
*/




//Old Stuff




/*alt
void CStepper_A4988::run(const uint_fast16_t stepPin_) {

unsigned long currentTimeMicros = micros(); // micros(); //save timeAcc here so that calcualtions timeAcc will not be included in our current micros and make wrong ticktimes etc... Remind that this call already need 2-4 uSec somehow on Arduino Uno.


//Acceleration
//if (finalStepDelay_uSec <= stepDelay_uSec && currentTimeMicros - timeLastAccTick >= tickTime2) {
if (finalStepDelay_uSec <= stepDelay_uSec) {
//Serial.println("time1: " + String(timeAcc * 100));
//Serial.println("stepDelay_uSec: " + String(stepDelay_uSec));
timeAcc += (currentTimeMicros - timeLastAccTick) / 1000000.0; //tickTimeToAdd;
stepDelay_uSec= 1000000/(acc*timeAcc);// V = a*t; d = 1000000/V; -> d = 1000000/(a*t)
timeLastAccTick = currentTimeMicros;

//Serial.println("stepDelay_uSec: " + String(stepDelay_uSec));
//Serial.println("time2: " + String(timeAcc*100));
//Serial.println("tickTimeToAdd: " + String(tickTimeToAdd*100));
}
if (finalStepDelay_uSec >= stepDelay_uSec && !LED13enabled)
digitalWriteFast(13, HIGH); //enable LED13 if acceleration is done
//Deacceleration
//Todo

//Do Steps
if (stepsToDo > 0 && currentTimeMicros - timeLastStep >= stepDelay_uSec) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {		//Serial.println("Run: " + String(stepDelay_uSec));
digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
stepsToDo--;						//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
timeLastStep = currentTimeMicros;	//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?
}
//Stop after all steps are done
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));
}
*/

/* Good one
//= 0;  //micros() takes some sec to call
void CStepper_A4988::run(const uint_fast16_t stepPin_) {

unsigned long currentTimeMicros = micros(); // micros(); //save timeAcc here so that calcualtions timeAcc will not be included in our current micros and make wrong ticktimes etc... Remind that this call already need 2-4 uSec somehow on Arduino Uno.
//Acceleration
if (finalStepDelay_uSec <= stepDelay_uSec && currentTimeMicros - timeLastAccTick >= accTickTime) {
stepDelay_uSec--;
timeLastAccTick = currentTimeMicros;
//Serial.println("stepDelay: " + String(stepDelay_uSec));
}
if(finalStepDelay_uSec >= stepDelay_uSec && !LED13enabled)
digitalWriteFast(13, HIGH); //enable LED13 if acceleration is done
//Deacceleration
//Todo

//Do Steps
if (stepsToDo > 0 && currentTimeMicros - timeLastStep >= stepDelay_uSec) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {		//Serial.println("Run: " + String(stepDelay_uSec));
digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
stepsToDo--;						//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
timeLastStep = currentTimeMicros;	//do this here to give a very small delay between the digitalWriteCalls. Sometimes there would be a DelayMicros() be used which we definitive dont want to use here!
digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?
}
//Stop after all steps are done
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));
}
*/
/* Für mikrokontroller.net
void CStepper_A4988::run(const uint_fast16_t stepPin_) {

unsigned long currentTimeMicros = micros();
//Acceleration (accelerate every accTickTime µS by subtract 1 µS from the current stepDelay_uSec)
if (finalStepDelay_uSec <= stepDelay_uSec && currentTimeMicros - timeLastAccTick >= accTickTime) {
stepDelay_uSec--;
timeLastAccTick = currentTimeMicros;
}
if (finalStepDelay_uSec >= stepDelay_uSec && !LED13enabled)
digitalWriteFast(13, HIGH); //enable LED13 if acceleration is done
//Deacceleration
//DO a Step every stepDelay_uSec
if (stepsToDo > 0 && currentTimeMicros - timeLastStep >= stepDelay_uSec) {
digitalWriteFast(stepPin_, HIGH);
stepsToDo--;
timeLastStep = currentTimeMicros;
digitalWriteFast(stepPin_, LOW);
}
//Stop after all steps are done
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));
}
*/



/*


unsigned long timeAcc = 1; //not 0!!! or it will end in 0/x
double accTime = 500000.0; //mS
unsigned long startDelay = accTime/4;  //=accTime/2
unsigned long accDelay = 0;
float frac = 0;
bool doAcc = true;

//= 0;  //micros() takes some sec to call
void CStepper_A4988::run(const uint_fast16_t stepPin_) {

unsigned long currentTimeMicros = micros(); // micros(); //this call somehow already need 2-4 uSec.
if (doAcc && currentTimeMicros - timeLastStep >= accDelay) {
timeAcc += accDelay;//%multi;//- (accMulti/multi);
frac = timeAcc / accTime;//(timeAcc%multi) / accTime;
accMulti = (3 - 2 * frac)*frac*frac; //f=(3 -2*frac)*frac*frac;
accDelay = startDelay * (1.0 - accMulti);
//Serial.println("Time: " + String(timeAcc));
//Serial.println("frac: " + String(frac));
//Serial.println("accMulti: " + String(accMulti));
//Serial.println("accDelay: " + String(accDelay));
if (timeAcc += accMulti > 65000) {//increase multi if 16Bit is near an overflow
multi = (multi * 100) ? < 100000) multi * 100 : 10000; //max 10.000 as multi (16 Bit int)
timeAcc /= 1000;
multi = 1000;
}
timeLastStep = currentTimeMicros;
if (timeAcc >= accTime) {
doAcc = false; //beter use bool for later beacus ebool compar is faster then double/long
Serial.println("DoAcc: " + String(doAcc));
}
}
else if (!doAcc && stepsToDo > 0 && currentTimeMicros - timeLastStep >= stepDelay_uSec) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {
//Serial.println("Run: " + String(stepDelay_uSec));
timeLastStep = currentTimeMicros;	//With += it autocorrects delay overtimes by adding always the same delaytime
digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
stepsToDo--;  //do it here to give a very small delay between the digitalWriteCalls
digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?
}
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));


}
*/


/* Not needed! Its totaly useless and error-prone because stepsPerRevolution depends on StepperResolution! User must know the anglePerSteps. With angles the StepsPerRevolution will be calculated!
void CStepper_A4988::setStepsPerRevolution(int_fast32_t steps) {
stepsPerRevolution = steps;
anglePerStep = 360.0f/ steps;  //needs to be changed to because normaly calcValue automatricly calculate the stepsPerRevolution
calcValues();
}
*/


/*
unsigned long micros1 = 0;
unsigned long micros2 = 0;
//stepDelay_uSec = (((stepsToDo / (double)speed_StepsPerSec)*1000000.0f) / stepsToDo)
long  a = (((100 / (double)28800)*1000000.0f) / 100); //28800 = 9 revolutions * 3200steps per revolution
//unsigned long a = 30000000;
long  d = (((100 / (double)288000)*1000000.0f) / 100); //1728000 = 60*1728000 acceleration steps/s²
long t = (28800 / (double)288000)*1000000.0f;
void loop()
{n: 292180992
micros1 = micros();
long D = (4 * a*a) + (d*(d - (4 * a) + (8 * t)));
//Serial.println("D: " + String(d));
//double sqrt_D = sqrt((4 * a*a) + (d*(d - (4 * a) + (8 * t))));
//double x = (-2 * a) - d;
double n = (((-2.0 * a) - d + sqrt(D)) / (2 * d));   //uint32_t n = (((-2 * a) - d + (uint32_t)sqrt(D))/(2*d));//int calculation would be faster .. but accuraccy is needed
*/


/*
unsigned long start_t = 0;
double d = 0;
long vt = 0;
unsigned long nexttick = 0;
unsigned long dummynextUpdate = 0;
long tx = 0;
int a = 3200;


//= 0;  //micros() takes some sec to call
void CStepper_A4988::run(const uint_fast16_t stepPin_){
unsigned long currentTimeMicros = micros(); // micros(); //this call somehow already need 2-4 uSec.




if (stepsToDo > 0 && currentTimeMicros >= timeLastStep) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {
//Serial.println("delay " + String(d));
//if (accelerationDelay_uSec > accelerationStepDelay_uSec) {
//timeLastStep += accelerationDelay_uSec;
//accelerationDelay_uSec -= accelerationStepDelay_uSec;
//Serial.println("accelerationDelay_uSec: " + String(accelerationDelay_uSec));
//	}
//else {
//Serial.println("Good: " + String(accelerationDelay_uSec));
if (d <= stepDelay_uSec) {
timeLastStep += stepDelay_uSec;  // do this as the very first thing before calling digitalWrite() so that the digitalWrite will not effect the timeLastStep delay. Also dont use "micros()+ stepDelay_uSec" because this will make wrong delays while some microcontrollers have a timerResolution of 4uSec (meanns if your stepDelay_uSec = 6 uSec you will loos 2 uSec because of a TimerResolution of 4 uSec)
//Serial.println("Good");
}
//}
digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?
stepsToDo--;
//Serial.println("Run");
}
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));


if ((d > stepDelay_uSec || d == 0) && nexttick <= currentTimeMicros) {
if (start_t == 0)
nexttick = start_t = currentTimeMicros;
vt = currentTimeMicros - start_t;
d = (1000000 / (a*(vt / 1000000.0)));
//Serial.println("delay " + String(nexttick));

dummynextUpdate = start_t + tx + d;
if (d <= (double)(vt - tx)) {
timeLastStep = dummynextUpdate;
//Serial.println("delay " + String(d) + " <= " + String((vt - tx)));
tx = vt;

}


nexttick += 240;
}

}
*/

/*
if (nextAccTick2 == 0) {
nextAccTick2 = currentTimeMicros;
//stepDelay_uSec = 1000;
}
if (currentTimeMicros >= nextAccTick2) {
if (stepDelay_uSec > 150) {  //2 = 220;8 = 80
stepDelay_uSec -= value; //*(currentTimeMicros+1 - nextAccTick)/accMulti;
}
else
stepDelay_uSec = 150;

if (stepsToDo < 300) {
stepDelay_uSec += 10;
}
nextAccTick2 += accMulti; //100 uSec
}
*/


/*


unsigned long currentTimeMicros = micros(); // micros(); //this call somehow already need 2-4 uSec.
if (stepsToDo > 0 && currentTimeMicros >= timeLastStep) { //if (isRunning && stepsToDo > 0 && currentTimeMicros >= timeLastStep) {
timeLastStep += stepDelay_uSec;
digitalWriteFast(stepPin_, HIGH); //diktial write is very slow using variables. fastest are using literals for pins!!!!!!!!!!!!!!!!!!!!!!!!!
digitalWriteFast(stepPin_, LOW); //ggf über Bitmask arbeiten da schneller?
stepsToDo--;
}
else if (stepsToDo == 0)   //after the last step is done do some final stuff and disable 64Bit compare (stepsToDo > 0) by setting isRunning = false
stop(); //calcs also the currentPos
//Serial.println("NextUpdate: " + String(timeLastStep));

if (counter == counterMax) {
if (stepDelay_uSec > 150) {  //2 = 220;8 = 80
stepDelay_uSec -= value;
}
else
stepDelay_uSec = 150;

if (stepsToDo < 300) {
stepDelay_uSec += 10;
}
counter = 0;
}
else
counter++;

*/

/*
unsigned long accelerationStartTime  = 0;
int32_t maxsteps = 3200*5;
int32_t startcounter = 3200;
int32_t startvalue = 10;


long vt = 0;

unsigned long diffstarttime = 0;
unsigned long dummynextUpdate = 0;
long tx = 0;
uint32_t a = 3200;
int32_t counterMax = 60;
int32_t counter = 0;

*/






/*uint32_t accDuration = (speed_StepsPerSec / acceleration) * 1000000; //Duration of Acceleration
uint32_t tickstotal = accDuration / accTickTime;
uint32_t ticksPerProfile = (Percent*(accDuration / 100))/ accTickTime;
uint32_t startDelay



double a = acceleration;  //Accel.
int32_t d = 1000000 / speed_StepsPerSec; //stepdelay
finalStepDelay_uSec = d;
uint32_t t = (speed_StepsPerSec / a)*1000000;  //timeAcc for Accel.
accTickTime = 1500*stepperResolution;//(1 / (a / 1000000.0)); ////there is a better solution need.. something dynamic currently the looptime is ~20 USec. means with this value ~25 can be controlled at the same timeAcc.. if more steppers are needed increase this timeAcc ....... Rubish: (10 * 20 > finalStepDelay_uSec * 2) ? 10 * 20 : finalStepDelay_uSec * 6;//10 * 20;//20 uSec = looptime per stepper; 10 = running steppers
uint32_t accTicks = t / accTickTime;

stepDelay_uSec = d + accTicks;
//accelerationStepDelay_uSec = d + accTicks;
//nextAccTick = start;
*/
/*
int32_t a = stepDelay_uSec = 1000;// 1000000 / speed_StepsPerSec;//(((stepsToDo / (double)speed_StepsPerSec)*1000000.0f) / stepsToDo);  ////every stepper takes ~2-3 uSec. (different on other arduinos) so fix the delay here! doublecast needed or data loss! Also its better do divide the 64 nuber before multiplicate it! This would work too without casting but can lead faster to 64bit overdlow -> (stepsToDo*1000000.0f / speed_StepsPerSec) 1.) Benötigte Zeit berechnen t = s/v  -> 2.) in uSec umrechnen   -> 3.) die stepdealy berechnen (dividieren mit stepsToDo)
double n = 0;
if (acceleration > 0) {
int32_t d = accelerationStepDelay_uSec = 1000000 / acceleration; //(((stepsToDo / (double)acceleration)*1000000.0f) / stepsToDo);  //calulateing in long (uSec) instead of shorter double because arduino uno can handle long faster then doubles
int64_t t = ((double)speed_StepsPerSec / acceleration)*1000000.0f;  //64 Bit value needed!!!!! we are calculating in uSec ..mabye better calc in Sec and later convert to uSec... Tested. In Sec seesm that t can get anyway a 64 Bit value
int64_t D = (4 * a*a) + (d*(d - (4 * a) + (8 * t)));  //dont calculat ehere with doubles.. all values have to be int/long
if (D >= 0) //Quadratic equation If Determinant < 0 then there is no result n. In this case let  n=0 which means that there will be no acceleration
n = (((-2.0 * a) - d + sqrt(D)) / (2 * d));   //uint32_t n = (((-2 * a) - d + (uint32_t)sqrt(D))/(2*d));//int calculation would be faster .. but accuraccy is needed
accelerationDelay_uSec = a + n*d;
Serial.println("n: " + String(n));
}
*/
//Beschleunigung
//accelerationDuration = ((double)speed_StepsPerSec / acceleration)*1000000.0f;
//	uint_fast32_t n = 																			  
//Serial.println("UnfixedDelay: " + String(stepDelay_uSec));
//lostMicrosEveryStep = (TIMER0_Resolution_uSec > 0)? stepDelay_uSec%(long)TIMER0_Resolution_uSec : 0;  //Micros() Resolution FIX: because the Micros() on Arduino UNO has a resolution of 4 uSec we will loose x microseconds on every step if the stepdelay ist not exaclty a multiple of the micros() resolution (4) ex. delay = 5; nextStep will be 5 (0+7); But the first micros()>5 is 8 (0,4,8,..) means we wait 3 uSec longer and this we every step because in next step 8+5=13 and  micros()>13=16 (0,4,8,12,16,..)
//stepDelay_uSec -= lostMicrosEveryStep; //fix the delay to a multiple of the Micros Resolution. We will add the lost micros later in the run()
//Serial.println("Timer0Resolution: " + String(TIMER0_Resolution_uSec));
//Serial.println("LostMicrosEveryStep: " + String(lostMicrosEveryStep));