package org.usfirst.frc.team6479.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class MultiSpeedController implements SpeedController {

	private SpeedController[] speedControllers;
	private double speed;
	private boolean inverted;

	public MultiSpeedController(SpeedController... speedControllers) {
		this.speedControllers = speedControllers;
		//defualt state
		inverted = false;
		for(SpeedController speedController: speedControllers) {
			speedController.setInverted(inverted);
		}
		set(0.0);
	}
	@Override
	public double get() {
		return speed;
	}
	@Override
	public void set(double speed) {
		this.speed = speed;

		for(SpeedController speedController: speedControllers) {
			speedController.set(speed);
		}
	}
	@Override
	public void pidWrite(double output) {
		set(output);
	}
	@Override
	public void setInverted(boolean isInverted) {
		inverted = isInverted;
		for(SpeedController speedController: this.speedControllers) {
			speedController.setInverted(isInverted);
		}
	}
	@Override
	public boolean getInverted() {
		return inverted;
	}
	@Override
	public void stopMotor() {
		for(SpeedController speedController: this.speedControllers) {
			speedController.set(0.0);
		}
	}
	@Override
	public void disable() {
		for(SpeedController speedController: this.speedControllers) {
			speedController.disable();
		}
	}
}
