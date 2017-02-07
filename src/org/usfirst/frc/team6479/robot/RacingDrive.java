package org.usfirst.frc.team6479.robot;

import edu.wpi.first.wpilibj.XboxController;

public class RacingDrive extends Teleop
{
	// controller object
	XboxController xbox;

	// get the drivetrain and the port for the controller
	public RacingDrive(Drivetrain drivetrain, int port)
	{
		super(drivetrain);
		// create controller
		xbox = new XboxController(port);
	}
	// racing drive
	public void drive()
	{
		drivetrain.arcade(rotate(), throttle(), true);
	}
	// how much speed
	public double throttle()
	{
		double left = xbox.getRawAxis(2);
		double right = xbox.getRawAxis(3);
		// each trigger has an axis range of 0 to 1
		// to make left trigger reverse, subtract axis value from right trigger
		return right - left;
	}
	// this method is called when the left joystick moves horizontally
	public double rotate()
	{
		double x = xbox.getRawAxis(0);
		// invert
		return(x * -1);
	}
}
