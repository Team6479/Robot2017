package org.usfirst.frc.team6479.robot;

public abstract class Teleop
{
	Drivetrain drivetrain;
	//get the drivetrain
	public Teleop(Drivetrain drivetrain)
	{
		this.drivetrain = drivetrain;
	}
	public abstract void drive();
}
