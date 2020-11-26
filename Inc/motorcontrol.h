#ifndef MOTORCONTROL
#define MOTORCONTROL

// Переменные для запуска мотора
typedef struct {
	float i;				// 
	float id; 			//
	float iq;				//
	float didt;			// 
	float eps;			// 
	float w;				// 
	float wMax;			//
	float angle;		//
	float iMax;			//
	float u;				//
} TStartVar;
//application flags and state machine

enum MotorStates{
	STATE_STOPPED,//           0	//motor is stopped
	STATE_STOPPING,//          1	//event to tell the motor to stop
	STATE_STARTING,//          2	//event to tell the motor to start, and startup sequence
	STATE_RUNNING,//           3	//motor is running
	STATE_FAULT//             4	//motor fault
};

typedef struct {
	unsigned MeasReady 			: 1;
    unsigned CurrentState 		: 3;
    unsigned OverTemp			: 1;
	unsigned CurrentMotor		: 1;
	unsigned MotorDetection		: 1;
	unsigned LowCurrent			: 1;
	unsigned MRASEnable			: 1;
} TFlags;


/***** Motor Control *****/

void initVariables(void);

void reinitVariables(void);

void runFullyControlled(void);

void startMotor(void);

void stopMotor(void);


#endif
