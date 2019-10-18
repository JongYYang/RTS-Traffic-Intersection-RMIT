#ifndef STATES_H
#define STATES_H
enum e_STATE
{
	GATE_CLOSED_INIT,
	GATE_OPEN,
	GATE_CLOSED,
	RAIL_ERROR
};
enum e_SENSOR_STATE
{
	INACTIVE = 0,
	ACTIVE,
	PRESENT,
	NOT_PRESENT,
	UNDEFINED
};

enum sensormode_states {
	State0,
	State1,
	State2,
	State3,
	State4,
	State5,
	State6,
	State7,
	State8,
	State9,
	State10,
	State11,
	State12,
	State13,
	State14,
	State15

};

enum overridemode_states {
		I1NSG,
		I2NSG,
		EW,
		Rail,
		RailxNSG

};

enum timermode_states {
		StateA,
		StateB,
		StateC,
		StateD,
		StateE,
		StateF

};


enum faultmode_states {
	yellow
};

#endif
