package org.usfirst.frc.team6479.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;

public class Drivetrain
{
	// motors
	Spark leftDrive;
	Spark rightDrive;
	// RobotDrive object for arcade drive
	RobotDrive drive;
	// get the motor ports
	public Drivetrain(int left, int right)
	{
		// create motors
		leftDrive = new Spark(left);
		rightDrive = new Spark(right);
		// left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);

		// setup RobotDrive
		drive = new RobotDrive(leftDrive, rightDrive);
	}
	// drive robot at speed
	public void drive(double leftSpeed, double rightSpeed)
	{
		leftDrive.set(leftSpeed);
		rightDrive.set(rightSpeed);
	}
	//drive robot in arcade mode
	public void arcade(double move, double rotation, boolean squared)
	{
		drive.arcadeDrive(move, rotation, squared);
	}
	public void driveCurve(double outputMagnitude, double curve) {
	    final double leftOutput;
	    final double rightOutput;
	    double sensitivity = .5;
	    
	    if (curve < 0) {
	      double value = Math.log(-curve);
	      double ratio = (value - sensitivity) / (value + sensitivity);
	      if (ratio == 0) {
	        ratio = .0000000001;
	      }
	      leftOutput = outputMagnitude / ratio;
	      rightOutput = outputMagnitude;
	    } else if (curve > 0) {
	      double value = Math.log(curve);
	      double ratio = (value - sensitivity) / (value + sensitivity);
	      if (ratio == 0) {
	        ratio = .0000000001;
	      }
	      leftOutput = outputMagnitude;
	      rightOutput = outputMagnitude / ratio;
	    } else {
	      leftOutput = outputMagnitude;
	      rightOutput = outputMagnitude;
	    }
	    drive(-1 * leftOutput, rightOutput);
	  }
}
