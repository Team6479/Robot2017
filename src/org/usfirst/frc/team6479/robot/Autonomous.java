package org.usfirst.frc.team6479.robot;

public abstract class Autonomous
{
	Drivetrain drivetrain;
	//get the drivetrain
	public Autonomous(Drivetrain drivetrain)
	{
		this.drivetrain = drivetrain;
	}
	public abstract void go();
}
