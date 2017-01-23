package org.usfirst.frc.team6479.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* TODO
 * Camera Tracking
 * User inputed scaling for fineDrive()
 * Commands for testing robot testing
 * Test functions
 * Custom Dashboard
 * Misc controls for whatever robot needs to do (i.e. climb rope)
 */

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	//thread for camera
	Thread visionThread;
	//the joysticks
	//Joystick rightStick = new Joystick(1);
	//Joystick leftStick = new Joystick(0);
	
	//xbox cotroller
	XboxController xbox = new XboxController(0);
	
	//the motor controllers for the drive train
	Spark leftDrive = new Spark(0);
	Spark rightDrive = new Spark(1);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		//left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);
		
		//setup camera
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				Imgproc.rectangle(mat, new Point(20, 20), new Point(620, 460),
						new Scalar(255, 255, 255), 5);
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//get the y axis of both joysticks
		//double leftPower = leftStick.getY();
		//double rightPower = rightStick.getY();
		//set the drive power to the joysticks axis
		//left drive is inverted since both motors are built identical
		//leftDrive.set(leftPower * -1);
		//rightDrive.set(rightPower);
		
		//if both are being used, right joystick has priority over left joystick and is called fullDrive
		if(isJoystickMoving(GenericHID.Hand.kRight) && isJoystickMoving(GenericHID.Hand.kLeft)) {
			fullDrive();
		}
		//if the right joystick is being used, call fullDrive
		else if(isJoystickMoving(GenericHID.Hand.kRight)) {
			fullDrive();
		}
		//if the left joystick is being used, call fineDrive
		else if(isJoystickMoving(GenericHID.Hand.kLeft)) {
			fineDrive();
		}
	}
	
	//determine if joystick is being moved
	//param is which joystick on controller
	public boolean isJoystickMoving(GenericHID.Hand hand) {
		//get the x and invert it
		double x = xbox.getX(hand);
		//get the y
		double y = xbox.getY(hand);
		
		//is the value of x within .001 of zero
		boolean isXZero = Math.abs(x) <= .001;
		
		//is the value of y within .001 of zero
		boolean isYZero = Math.abs(y) <= .001;
		
		//if x and y are zero, joystick is not moving, return false
		if(isXZero && isYZero) {
			return false;
		}
		else {
			return true;
		}
	}
	
	//for driving
	//this function is called when the right joystick on the xbox cotroller is being used
	//gives full control, going from -1 to 1
	public void fullDrive() {
		//get the direction and power vector in form (x=right speed, y=left speed)
		Point powerDirection = powerAndDirection(GenericHID.Hand.kRight);
		//set the speed for both motors as is from vector
		rightDrive.set(powerDirection.x);
		leftDrive.set(powerDirection.y);
	}
	
	//for driving
	//this function is called when the left joystick on the xbox cotroller is being used
	//gives fine control, going from -.1 to .1
	public void fineDrive() {
		//get the direction and power vector in form (x=right speed, y=left speed)
		Point powerDirection = powerAndDirection(GenericHID.Hand.kLeft);
		
		//scale the speed
		//eventually this will get scaling information from driver station or controller, for now just scale to 1/10
		double scaleFactor = .1;
		double rightSpeed = powerDirection.x * scaleFactor;
		double leftSpeed = powerDirection.y * scaleFactor;
		
		//set the speed for both motors to scaled speed
		rightDrive.set(rightSpeed);
		leftDrive.set(leftSpeed);
	}
	
	//for driving
	//this function calculates what direction the joystick is pointing and how much power it outputs
	//uses tables and formula from http://home.kendra.com/mauser/Joystick.html
	//paramaters is which joystick to calculate from, right or left
	public Point powerAndDirection(GenericHID.Hand hand) {
		//get the x and invert it
		double x = xbox.getX(hand) * -1;
		//get the y
		double y = xbox.getY(hand);
		//calculate v and w, these are vars to calc left and right speed
		//if u want to know more, read the webpage
		double v = ((1 - Math.abs(x)) * y) + y;
		double w = ((1 - Math.abs(y)) * x) + x;
		//right speed
		double r = (v + w) / 2;
		//left speed
		double l = (v - w) / 2;
		
		//vector to hold right and left speed of drivetrain, values from -1 to 1
		Point powerDirection = new Point(r, l);
		return powerDirection;
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
}

