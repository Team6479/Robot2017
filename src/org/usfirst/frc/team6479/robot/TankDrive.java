package org.usfirst.frc.team6479.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class TankDrive extends Teleop
{
	// controller object
	XboxController xbox;

	// get the drivetrain and the port for the controller
	public TankDrive(Drivetrain drivetrain, int port)
	{
		super(drivetrain);
		// create controller
		xbox = new XboxController(port);
	}
	// tank drive
	public void drive()
	{
		//get the left speed
		double left = xbox.getY(Hand.kLeft);
		//get the right speed
		double right = xbox.getY(Hand.kRight);
		//drive
		drivetrain.drive(left, right);
	}
}
