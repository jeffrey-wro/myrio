#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <math.h>

#include "MyRio.h"
#include "I2C.h"
#include "Motor_Controller.h"
#include "Utils.h"

using namespace std;

extern NiFpga_Session myrio_session;

NiFpga_Status status;

int main(int argc, char **argv)
{
	status = MyRio_Open();
	if (MyRio_IsNotSuccess(status))
	{
		return status;
	}

	MyRio_I2c i2c;
	status = Utils::setupI2CB(&myrio_session, &i2c);

	Motor_Controller mc = Motor_Controller(&i2c);
	mc.controllerEnable(DC);
	mc.controllerEnable(SERVO);

	int volt = mc.readBatteryVoltage(1);
	printf("%d\n\n", volt);

	for(int i=0; i<5; i++){
		mc.setCRServoState(SERVO, CR_SERVO_1, 100);	
		Utils::waitFor(3);
		mc.setCRServoState(SERVO, CR_SERVO_1, -100);
		Utils::waitFor(3);
	}

	mc.controllerReset(DC);
	mc.controllerReset(SERVO);

	status = MyRio_Close();

	return status;

	return 0;
}
