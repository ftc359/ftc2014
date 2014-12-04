int joystickExponential(int deadzone, int maxSpeed, int joystickCurrent){
	if(joystickCurrent == 0){
		return 0;
	}
	int polarity = joystickCurrent/abs(joystickCurrent);
	long maxSpeed2 = maxSpeed*maxSpeed;
	long newInside = maxSpeed-sqrt(((abs(joystickCurrent)-127)*maxSpeed2)/(deadzone-127));
	return polarity*newInside;
}

int joystickLinear(int deadzone, int maxSpeed, int joystickCurrent){
	if(joystickCurrent == 0){
		return 0;
	}
	int polarity = joystickCurrent/abs(joystickCurrent);
	long newInside = maxSpeed-((abs(joystickCurrent)-127)*maxSpeed)/(deadzone-127);
	return polarity*newInside;
}
