package org.usfirst.frc.team6479.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
	final String teleArcade = "arcade";
	final String teleRacing = "racing";
	final String teleTank = "tank";
	
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
	
	
	
	CustomDrive driveTrain = new CustomDrive(leftDrive, rightDrive);
	
	//enums for left and right joystick on
	public enum JoystickOn {
		LEFT, RIGHT
	}
	//which stick is on
	JoystickOn stickOn;
	
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;

	
	//RobotDrive and Gyro for autonomous
//	RobotDrive myDrive;
	ADXRS450_Gyro gyro;
	double Kp = 0.03;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		GripPipeline pipe = new GripPipeline();
		
		gyro = new ADXRS450_Gyro();
	//	gyro.startThread();
		//gyro.startThread();
		
		autoChooser.addDefault("Default Auto", autoDefault);
		autoChooser.addObject("First Auto", autoOne);
		autoChooser.addObject("Second Auto", autoTwo);
		SmartDashboard.putData("Auto Choices", autoChooser);
		
		teleChooser.addDefault("Arcade Drive", teleArcade);
		teleChooser.addObject("Racing Drive", teleRacing);
		teleChooser.addObject("Tank Drive", teleTank);
		SmartDashboard.putData("Tele Choices", teleChooser);
		
		
		SmartDashboard.putNumber("Angel to move", 90);
		
		
		//left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);
		
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
	//			System.out.println("xRex" + xRes);
	//			System.out.println("width of line" + crossWidth);
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
		autoSelected = autoChooser.getSelected();
		
		//init the encoders
		leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k4X);
		//set the time until when the robot is considered stopped, set in seconds
		leftDriveEncoder.setMaxPeriod(.05);
		rightDriveEncoder.setMaxPeriod(.05);
		
		//initialize RobotDrove and Gyro
	//	myDrive = new RobotDrive(1,2);
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	//	SmartDashboard.putNumber("Left Distance", leftDriveEncoder.getDistance());
	//	SmartDashboard.putNumber("Right Distance", rightDriveEncoder.getDistance());
		
	/*	while(isAutonomous()){
		driveTrain.tankDrive(-.8, .8);
		Timer.delay(3);
		}
	*/
		
		
		//gyro.calibrate();
		gyro.reset();
		//while(isAutonomous() && (leftDriveEncoder.getDistance()<2000) && (rightDriveEncoder.getDistance()<2000)){
		while(isAutonomous()&& (leftDriveEncoder.getDistance()<1000) && (rightDriveEncoder.getDistance()<1000)){
			double angle = gyro.getAngle();
			driveTrain.drive(0.25, -angle*Kp);

			//System.out.println("Angle: " + angle);
			//System.out.println("Turn: " + -1*angle*Kp);
			Timer.delay(0.004);
		}
		
		driveTrain.drive(0, 0);
		
		
		gyro.reset();
		SmartDashboard.putNumber("Angle before", gyro.getAngle());
		while(isAutonomous() &&Math.abs(gyro.getAngle()) <= 360){
			rightDrive.setSpeed(-0.30);
			leftDrive.setSpeed(-0.30);
		}
		leftDrive.setSpeed(0);
		rightDrive.setSpeed(0);
		Timer.delay(5);
		SmartDashboard.putNumber("Angle after", gyro.getAngle());
		/*switch (autoSelected) {
		case autoOne:
			driveTrain.tankDrive(-.8, .8);
			
		
			break;
		case autoTwo:
			// Put custom auto code here
			break;
		case autoDefault:
		default:
			// Put default auto code here
			
			//basic autonomous, drive forward, then turn left
			if(Math.abs(leftDriveEncoder.getDistance()) < 600 && Math.abs(rightDriveEncoder.getDistance()) < 600)
			{
				leftDrive.set(-.5);
				rightDrive.set(.5);
			}
			else {
				leftDrive.set(0);
				rightDrive.set(0);
			}
			break;
		}
		*/
	}

	/**
	 * This function is called at the start of operator control
	 */
	@Override
	public void teleopInit() {
		//get the teleop driving config
		teleSelected = teleChooser.getSelected();
		SmartDashboard.putString("Selected", teleSelected);
		//if its for arcade, set right stick on
		if(teleSelected.equals(teleArcade)) {
			stickOn = JoystickOn.RIGHT;
		}
		//init the encoders
		leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k4X);
		//set the time until when the robot is considered stopped, set in seconds
		leftDriveEncoder.setMaxPeriod(.05);
		rightDriveEncoder.setMaxPeriod(.05);
	
		angleToMove = SmartDashboard.getNumber("Angel to move", 90);
		gyro.reset();
	}
	double angleToMove;
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Left Distance", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("Right Distance", rightDriveEncoder.getDistance());
		
		if(xbox.getAButton()){
	
			gyro.reset();
			SmartDashboard.putNumber("Angle before", gyro.getAngle());
			while(Math.abs(gyro.getAngle()) <= angleToMove){
				rightDrive.setSpeed(-0.30);
				leftDrive.setSpeed(-0.30);
	
				//Timer.delay(0.004);
			}
			leftDrive.setSpeed(0);
			rightDrive.setSpeed(0);
			
			Timer.delay(5);
			
			SmartDashboard.putNumber("Angle after", gyro.getAngle());
			
		}
		
		//choose which teleop is selected
		switch (teleSelected) {
		case teleRacing:
			//call function for racing drive
			racing();
			break;
		case teleTank:
			//calculates left side, sets left drive speed to y axis of xbox left
			leftDrive.set(xbox.getY(Hand.kLeft) );
			//calculates right side, sets right drive speed to y axis of xbox right
				rightDrive.set(xbox.getY(Hand.kRight) );
			break;
		case teleArcade:
			//call the arcade function
		default:
			arcade();
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