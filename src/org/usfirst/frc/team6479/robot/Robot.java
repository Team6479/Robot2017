package org.usfirst.frc.team6479.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
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
	
	//chooser for auto
	SendableChooser<String> autoChooser = new SendableChooser<>();
	//auto options
	final String autoDefault = "default";
	final String autoOne = "one";
	final String autoTwo = "two";
	//which auto is selected
	String autoSelected;
	
	//chooser for tele
	SendableChooser<String> teleChooser = new SendableChooser<>();
	//tele options
	final String teleDefault = "default";
	final String teleArcade = "arcade";
	final String teleRacing = "racing";
	//which tele is selected
	String teleSelected;
	
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
	
	RobotDrive driveTrain = new RobotDrive(leftDrive, rightDrive);
	
	//enums for left and right joystick on
	public enum JoystickOn {
		LEFT, RIGHT
	}
	//which stick is on
	JoystickOn stickOn;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		GripPipeline pipe = new GripPipeline();
		
		autoChooser.addDefault("Default Auto", autoDefault);
		autoChooser.addObject("First Auto", autoOne);
		autoChooser.addObject("Second Auto", autoTwo);
		SmartDashboard.putData("Auto Choices", autoChooser);
		
		teleChooser.addDefault("Default Tele", teleDefault);
		teleChooser.addObject("Arcade Drive", teleArcade);
		teleChooser.addObject("Racing Drive", teleRacing);
		SmartDashboard.putData("Tele Choices", teleChooser);
		
		
		//left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);
		
		//setup camera
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			int xRes = 320;
			int yRes = 240;
			camera.setResolution(xRes, yRes);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Cross Hairs", xRes, yRes);

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
				
				pipe.process(mat);
				mat = pipe.maskOutput();
				
				// Put cross hairs on the image
				int crossWidth = xRes / 20;
				System.out.println("xRex" + xRes);
				System.out.println("width of line" + crossWidth);
				int crossHeight = yRes / 20;
				Imgproc.line(mat, new Point((xRes / 2) - (crossWidth / 2), (yRes / 2)), new Point((xRes / 2) + (crossWidth / 2), (yRes / 2)), new Scalar(0, 255, 0), 5);
				Imgproc.line(mat, new Point((xRes / 2), (yRes / 2) + (crossHeight / 2)), new Point((xRes / 2), (yRes / 2) - (crossHeight / 2)), new Scalar(0, 255, 0), 5);
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
		//autoSelected = autoChooser.getSelected();
		//get the selected autonomous
		autoSelected = SmartDashboard.getString("Auto Choices", autoDefault);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case autoOne:
			// Put custom auto code here
			break;
		case autoTwo:
			// Put custom auto code here
			break;
		case autoDefault:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called at the start of operator control
	 */
	@Override
	public void teleopInit() {
		//get the teleop driving config
		teleSelected = SmartDashboard.getString("Tele Choices", teleDefault);
		//if its for arcade, set right stick on
		if(teleSelected.equals(teleArcade)) {
			stickOn = JoystickOn.RIGHT;
		}
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//choose which teleop is selected
		switch (teleSelected) {
		case teleArcade:
			//call the arcade function
			arcade();
			break;
		case teleRacing:
			//call function for racing drive
			racing();
			break;
		case teleDefault:
		default:
			//This will be tank drive
			break;
		}
		
	}
	
	public void racing(){
		driveTrain.arcadeDrive(rotate(), throttle());
	}
	//arcade drive
	public void arcade() {
		//if the right stick is on and the leftstick is clicked
		//switch which stick is on to left stick
		if(stickOn == JoystickOn.RIGHT && xbox.getStickButton(Hand.kLeft))
		{
			stickOn = JoystickOn.LEFT;
		}
		//if the left stick is on and the rightstick is clicked
		//switch which stick is on to right stick
		if(stickOn == JoystickOn.LEFT && xbox.getStickButton(Hand.kRight))
		{
			stickOn = JoystickOn.RIGHT;
		}
		
		//if the right stick is on, use full drive
		if(stickOn == JoystickOn.RIGHT)
		{
			fullDrive();
		}
		//if the left stick is on, use fine drive
		if(stickOn == JoystickOn.LEFT)
		{
			fineDrive();
		}
	}
	//determine if joystick is being moved
	//param is which joystick on controller
	public double throttle(){
		double left = xbox.getRawAxis(2);
		double right = xbox.getRawAxis(3);
		//each trigger has an axis range of 0 to 1
		//to make left trigger reverse, subtract axis value from right trigger
		return right - left;
	}
	//this method is called when the left joystick moves horizontally
	public double rotate(){
		double x = xbox.getRawAxis(0);
		//invert
		return (x * -1);
	}
	public boolean isJoystickMoving(Hand hand) {
		//get x and y
		double x = xbox.getX(hand);
		double y = xbox.getY(hand);
		
		//is the value of x within .001 of zero
		boolean isXZero = Math.abs(x) <= .01;
		
		//is the value of y within .001 of zero
		boolean isYZero = Math.abs(y) <= .01;
		
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
		
		//get x and y
		double x = xbox.getRawAxis(4);
		double y = xbox.getRawAxis(5);
		
		//invert
		driveTrain.arcadeDrive(x * -1, y * -1, true);
	}
	
	//for driving
	//this function is called when the left joystick on the xbox cotroller is being used
	//gives fine control, going from -.5 to .5
	public void fineDrive() {
		
		//get x and y
		double x = xbox.getRawAxis(0);
		double y = xbox.getRawAxis(1);
		
		//scale the speed
		//eventually this will get scaling information from driver station or controller, for now just scale to 1/10
		double scaleFactor = .5;
		
		//invert and scale
		driveTrain.arcadeDrive(x * scaleFactor * -1, y * scaleFactor * -1, false);
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
}