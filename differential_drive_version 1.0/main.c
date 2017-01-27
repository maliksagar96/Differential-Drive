
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART_32.h"
#include "compass_sensor.h"
#include "movingArray.h"
#include <math.h>

// software var
#define PIDLoop_mainLoop_ratio		10
#define pi							3.1416
#define rpmMovArrayLength			10
#define	timeInterval				0.03264				// 1024 * 255 / F_CPU // in sec
#define leftPWM						OCR1A
#define rightPWM					OCR1B

// hardware var
#define ticksPerRotation			720
#define r							5.0
#define L							23.8
#define circumference				31.4				//2 * pi * r
#define vmax						200

struct position {float x; float y; int phi;};
struct unicycleState {float v; float w;};
struct differentialState {int leftRPM; int rightRPM;};

enum {left, right}; //wheel
enum {lRPM, rRPM, angularVel}; // movingArray, PID

volatile long ticks[2] = {0, 0};
volatile struct position curBotPosition;
volatile struct position desiredBotPosition;
volatile struct differentialState curDiffState;
volatile struct differentialState desiredDiffState;
int unitTimeCount = 0;
volatile float phi_ref = 0;
float kp[3] = {0.1, 0.1, 0.1}, ki[3] = {0, 0, 0}, kd[3] = {0.00, 0.00, 0}, E[3] = {0, 0, 0}, e_old[3] = {0, 0, 0};

float PID(float error,int x) {
	float pid = 0;
	pid = (kp[x]*error) + (ki[x]*E[x]) + (kd[x]*(error - e_old[x]));
	E[x]+=error;
	e_old[x] = error;
	return pid;
}

inline void Graph_Plot()
{
	USART_Transmitchar(0xAB);
	USART_Transmitchar(0xCD);
	USART_Transmitchar(0x08);
	USART_Transmitchar(0x00);
	USART_Transmitchar(curDiffState.leftRPM & 0x00FF);
	USART_Transmitchar(((curDiffState.leftRPM & 0xFF00) >> 8));
	USART_Transmitchar(curDiffState.rightRPM & 0x00FF);
	USART_Transmitchar(((curDiffState.rightRPM & 0xFF00) >> 8));
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
}

void print(int x) {
	if(x < 0) {
		x *= -1;
		USART_Transmitchar('-');
	}
	USART_TransmitNumber(x);
}

float degreeToRad(float degree) {
	return degree * pi / 180.0;
}

float radToDegree(float rad) {
	return rad * 180.0 / pi;
}

float normalizeAngle(float degree) {
	return radToDegree(atan2(sin(degreeToRad(degree)), cos(degreeToRad(degree))));
}

float sigmoid(int z) {
	return tanh(z/40);
}

struct unicycleState getDesiredUnicycleState(struct position curBotPosition, struct position desiredBotPosition) {
	struct unicycleState desiredState;
	
	int errDist = sqrt((desiredBotPosition.x - curBotPosition.x) * (desiredBotPosition.x -curBotPosition.x) + (desiredBotPosition.y - curBotPosition.y)*(desiredBotPosition.y - curBotPosition.y));
	int desiredPhi = 0;
	if((curBotPosition.x - desiredBotPosition.x) == 0) {
		if(desiredBotPosition.y > curBotPosition.y) {
			desiredPhi = 90;
			} else {
			desiredPhi = -90;
		}
		} else if((curBotPosition.y - desiredBotPosition.y) == 0) {
		if(desiredBotPosition.x > curBotPosition.x) {
			desiredPhi = 0;
			} else {
			desiredPhi = 180;
		}
		} else {
		desiredPhi = radToDegree(atan((desiredBotPosition.y - curBotPosition.y) / (desiredBotPosition.x - curBotPosition.x)));
	}
	
	desiredState.v = vmax * sigmoid(errDist);
	desiredState.w = PID(normalizeAngle(desiredPhi - curBotPosition.phi), angularVel);
	return desiredState;
}

struct differentialState transformUniToDiff(struct unicycleState uniState) {
	struct differentialState diffState;
	//using the kinematics equations
	float vleft = (2*uniState.v -L*uniState.w) / (2 * r);
	float vright = (2*uniState.v + L*uniState.w)/(2 * r);
	diffState.rightRPM = vright / circumference * 60;
	diffState.leftRPM = vleft / circumference * 60;
	return diffState;
}

void calculateDiffState() {
	int x;
	int sampledTicks[] = {ticks[left], ticks[right]};
	ticks[0] = 0;
	ticks[1] = 0;
	for(x = 0; x < 2; x++) {
		float rpm = sampledTicks[x] * 60 / ticksPerRotation / (timeInterval);
		addElement(rpm, x);
	}
	curDiffState.leftRPM = getAverage(lRPM);
	curDiffState.rightRPM = getAverage(rRPM);
}

void calculatePos() {
	curBotPosition.phi = normalizeAngle(phi_ref - getHeading());
	
	float leftDist = curDiffState.leftRPM * timeInterval / 60.0 * circumference;
	float rightDist = curDiffState.rightRPM * timeInterval / 60.0 * circumference;
	float dist = (leftDist + rightDist) / 2;
	
	curBotPosition.x += dist * cos(degreeToRad(curBotPosition.phi));
	curBotPosition.y += dist * sin(degreeToRad(curBotPosition.phi));
}

void changeWheelOutputs(struct differentialState curState, struct differentialState desiredState) {
	int leftPID = leftPWM;
	int rightPID = rightPWM;
	leftPID += PID(desiredState.leftRPM - curState.leftRPM, left);
	rightPID += PID(desiredState.rightRPM - curState.rightRPM, right);
	if(leftPID > 1023) {
		leftPID = 1023;
		} else if(leftPID < 0){
		leftPID = 0;
	}
	if(rightPID > 1023) {
		rightPID = 1023;
		} else if(rightPID < 0){
		rightPID = 0;
	}
	leftPWM = leftPID;
	rightPWM = rightPID;
}

void init() {
	_delay_ms(100); // time to let compass sensor load
	
	DDRD |= (1<<PD5) | (1<<PD4); // ocr pins
	
	//interrupt , any logical change
	MCUCR |= (1<<ISC00) | (1<<ISC10);
	GICR |= (1<<INT1) | (1<<INT0);
	
	//PWM_timer , Fast_PWM_mode, Top = 0x03FF
	TCCR1B |= (1<<CS10) | (1<<WGM12);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1 << WGM11) | (1<< WGM10);
	
	//Counter_timer, time for each loop = 255 * 1024(prescaler) = 261120.0;
	TCCR0 |= (1<<CS02) | (1<<CS00) ;
	TIMSK |= (1<<TOIE0);
	
	init_HMC5883L();
	USART_Init(12);
	sei();
	phi_ref = getHeading();					//initialising the first ever angle taken by the compass sensor as reference
	init_movingArray(rpmMovArrayLength, lRPM);
	init_movingArray(rpmMovArrayLength, rRPM);
	
	//taking initial point as origin
	curBotPosition.x = 0;
	curBotPosition.y = 0;
}

int main() {
	init();
	desiredBotPosition.x = 500;
	desiredBotPosition.y = 250;
	while(1) {

	}
}

ISR(TIMER0_OVF_vect) {
	unitTimeCount++;
	calculateDiffState();
	calculatePos();
	changeWheelOutputs(curDiffState, desiredDiffState);
	if(unitTimeCount == PIDLoop_mainLoop_ratio) {
		desiredDiffState = transformUniToDiff(getDesiredUnicycleState(curBotPosition, desiredBotPosition));
		unitTimeCount = 0;
	}
}

ISR(INT0_vect) {
	ticks[left]++;
}

ISR(INT1_vect) {
	ticks[right]++;
}

