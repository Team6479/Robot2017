package org.usfirst.frc.team6479.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ArcadeDrive extends Teleop
{
	// controller object
	XboxController xbox;

	// enums for left and right joystick on
	public enum JoystickOn
	{
		LEFT, RIGHT
	}
	// which stick is on
	JoystickOn stickOn;

	// get the drivetrain and the port for the controller
	public ArcadeDrive(Drivetrain drivetrain, int port)
	{
		super(drivetrain);
		// create controller
		xbox = new XboxController(port);
		// starting mode is right
		stickOn = JoystickOn.RIGHT;
	}
	// arcade drive
	public void drive()
	{
		// if the right stick is on and the leftstick is clicked
		// switch which stick is on to left stick
		if(stickOn == JoystickOn.RIGHT && xbox.getStickButton(Hand.kLeft))
		{
			stickOn = JoystickOn.LEFT;
		}
		// if the left stick is on and the rightstick is clicked
		// switch which stick is on to right stick
		if(stickOn == JoystickOn.LEFT && xbox.getStickButton(Hand.kRight))
		{
			stickOn = JoystickOn.RIGHT;
		}
		// if the right stick is on, use full drive
		if(stickOn == JoystickOn.RIGHT)
		{
			fullDrive();
		}
		// if the left stick is on, use fine drive
		if(stickOn == JoystickOn.LEFT)
		{
			fineDrive();
		}
	}
	// for driving
	// this function is called when the right joystick on the xbox cotroller is
	// being used
	// gives full control, going from -1 to 1
	public void fullDrive()
	{

		// get x and y
		double x = xbox.getRawAxis(4);
		double y = xbox.getRawAxis(5);

		// invert
		drivetrain.arcade(x * -1, y * -1, true);
	}
	// this function is called when the left joystick on the xbox cotroller is
	// being used
	// gives fine control, going from -.5 to .5
	public void fineDrive()
	{

		// get x and y
		double x = xbox.getRawAxis(0);
		double y = xbox.getRawAxis(1);

		// scale the speed
		// eventually this will get scaling information from driver station or
		// controller, for now just scale to 1/10
		double scaleFactor = .5;

		// invert and scale
		drivetrain.arcade(x * scaleFactor * -1, y * scaleFactor * -1, false);
	}
}
