package org.usfirst.frc.team6479.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
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
	
	PIDController pidTurnLeftDrive;
	PIDController pidTurnRightDrive;
	double pidTP;
	double pidTI;
	double pidTD;
	double pidTF;

	
	PIDController pidDriveLeftDrive;
	PIDController pidDriveRightDrive;
	double pidDP;
	double pidDI;
	double pidDD;
	double pidDF;
	
	CustomDrive driveTrain = new CustomDrive(leftDrive, rightDrive);
	
	Victor climber = new Victor(2);
	
	//enums for left and right joystick on
	public enum JoystickOn {
		LEFT, RIGHT
	}
	//which stick is on
	JoystickOn stickOn;
	
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;

	AnalogInput sonar;
	//RobotDrive and Gyro for autonomous
//	RobotDrive myDrive;
	ADXRS450Gyro gyro;
	double Kp = 0.03;
	
	
	
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	private boolean turn = false;
	
	//camera thread
	Thread thread;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		gyro = new ADXRS450Gyro();
		gyro.startThread();
		
		pidTP = .05;
		pidTI = 0;
		pidTD = .01;
		pidTF = 0;
		SmartDashboard.putNumber("PIDT P", pidTP);
		SmartDashboard.putNumber("PIDT I", pidTI);
		SmartDashboard.putNumber("PIDT D", pidTD);
		SmartDashboard.putNumber("PIDT F", pidTF);
		
		//enable PID's
		pidTurnLeftDrive =new PIDController(pidTP, pidTI, pidTD, pidTF, gyro, leftDrive);
		pidTurnRightDrive =new PIDController(pidTP, pidTI, pidTD, pidTF, gyro, rightDrive);
		pidTurnLeftDrive.setInputRange(-1200, 1200);
		pidTurnRightDrive.setInputRange(-1200, 1200);
		pidTurnLeftDrive.setOutputRange(-1, 1);
		pidTurnRightDrive.setOutputRange(-1, 1);
		pidTurnLeftDrive.setAbsoluteTolerance(1);
		pidTurnRightDrive.setAbsoluteTolerance(1);
		pidTurnLeftDrive.setContinuous(true);
		pidTurnRightDrive.setContinuous(true);
		
		
		
		
			//init the encoders
			leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
			rightDriveEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k4X);
			//set the time until when the robot is considered stopped, set in seconds
			leftDriveEncoder.setMaxPeriod(.05);
			rightDriveEncoder.setMaxPeriod(.05);
			//set distance per pulse to be 1 inch
			rightDriveEncoder.setDistancePerPulse((6 * Math.PI) / 360);
			leftDriveEncoder.setDistancePerPulse((6 * Math.PI) / 360);
			
			
			pidDP = .01;
			pidDI = 0;
			pidDD = 0;
			pidDF = 0;
			SmartDashboard.putNumber("PIDD P", pidDP);
			SmartDashboard.putNumber("PIDD I", pidDI);
			SmartDashboard.putNumber("PIDD D", pidDD);
			SmartDashboard.putNumber("PIDD F", pidDF);
			
			//enable PID's
			pidDriveLeftDrive =new PIDController(pidDP, pidDI, pidDD, pidDF, leftDriveEncoder, leftDrive);
			pidDriveRightDrive =new PIDController(pidDP, pidDI, pidDD, pidDF, rightDriveEncoder, rightDrive);
			pidDriveLeftDrive.setInputRange(-180, 180);
			pidDriveRightDrive.setInputRange(-180, 180);
			pidDriveLeftDrive.setOutputRange(-1, 1);
			pidDriveRightDrive.setOutputRange(-1, 1);
			pidDriveLeftDrive.setAbsoluteTolerance(1);
			pidDriveRightDrive.setAbsoluteTolerance(1);
			pidDriveLeftDrive.setContinuous(true);
			pidDriveRightDrive.setContinuous(true);
		
		
		autoChooser.addDefault("Default Auto", autoDefault);
		autoChooser.addObject("First Auto", autoOne);
		autoChooser.addObject("Second Auto", autoTwo);
		SmartDashboard.putData("Auto Choices", autoChooser);
		
		teleChooser.addDefault("Arcade Drive", teleArcade);
		teleChooser.addObject("Racing Drive", teleRacing);
		teleChooser.addObject("Tank Drive", teleTank);
		SmartDashboard.putData("Tele Choices", teleChooser);
		
		
		SmartDashboard.putNumber("Angle to move", 90);
		SmartDashboard.putNumber("Feet to move", 3);
		
		
		//left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);
		
		
		sonar = new AnalogInput(0);
		
		GripPipelineHSV grip = new GripPipelineHSV();
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(640, 480);
			
			thread = new Thread(() -> {
				

				// Get a CvSink. This will capture Mats from the camera
				CvSink cvSink = CameraServer.getInstance().getVideo();
				// Setup a CvSource. This will send images back to the Dashboard
				CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 320, 240);

				// Mats are very memory expensive. Lets reuse this Mat.
				Mat mat = new Mat();
			//	pipe.process(mat);
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
					//process image
					grip.process(mat);
				//	mat = grip.hsvThresholdOutput();

					ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
					Rect r = null;
					if (contours.size() > 0){
						r = Imgproc.boundingRect(contours.get(0));
					}
				    Rect r2 = null;
				    if (contours.size() > 1){
				    	r2 = Imgproc.boundingRect(contours.get(1));	
				    }
				    
					
			/*		Imgproc.rectangle(mat, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height),
							new Scalar(255, 255, 255), 1);
					Imgproc.rectangle(mat, new Point(r2.x, r2.y), new Point(r2.x+r2.width, r2.y+r2.height),
							new Scalar(255, 255, 255), 1);
			*/	
				    
				    Point center = null;
				    if (r != null && r2 != null){
				    	center = new Point((r.x+r.width+r2.x)/2, r.y + (r.height/2));	
				    }
				    
				 //   System.out.println("X: " + ((r.x+r.width+r2.x)/2) + "Y: " + (r.y + (r.height/2)));
					if (center != null){
						Imgproc.circle(mat, center, 5, new Scalar(255,255,255));
						centerX = center.x;
						turn = true;
					//	System.out.println("Thread");
					}
				
				//	System.out.println("X: " + centerX);
				//	Imgproc.line(mat, new Point(280, 240), new Point(360, 240), new Scalar(255,255,255));
					// Give the output stream a new image to display
					
				//	outputStream.putFrame(mat);
				//	outputStream.putFrame(grip.hslThresholdOutput());
				}
			});
			
			thread.setDaemon(true);
			thread.start();
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
		
		//reset the encoders
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		
		//initialize RobotDrove and Gyro
	//	myDrive = new RobotDrive(1,2);
		gyro.reset();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		rightDriveEncoder.reset();
		leftDriveEncoder.reset();
		while(Math.abs(rightDriveEncoder.getDistance()) <= (feetToMove * 12) &&
					Math.abs(leftDriveEncoder.getDistance()) <= (feetToMove * 12)){
			rightDrive.setSpeed(-0.30);
			leftDrive.setSpeed(0.30);
		}
		leftDrive.setSpeed(0);
		rightDrive.setSpeed(0);
		
		
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

		//reset the encoders
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
	
		
		pidTP = SmartDashboard.getNumber("PIDT P", pidTP);
		pidTI = SmartDashboard.getNumber("PIDT I", pidTI);
		pidTD = SmartDashboard.getNumber("PIDT D", pidTD);
		pidTF = SmartDashboard.getNumber("PIDT F", pidTF);
		pidTurnLeftDrive.setPID(pidTP, pidTI, pidTD, pidTF);
		pidTurnRightDrive.setPID(pidTP, pidTI, pidTD, pidTF);
		
		pidDP = SmartDashboard.getNumber("PIDD P", pidDP);
		pidDI = SmartDashboard.getNumber("PIDD I", pidDI);
		pidDD = SmartDashboard.getNumber("PIDD D", pidDD);
		pidDF = SmartDashboard.getNumber("PIDD F", pidDF);
		pidDriveLeftDrive.setPID(pidDP, pidDI, pidDD, pidDF);
		pidDriveRightDrive.setPID(pidDP, pidDI, pidDD, pidDF);
		
		
		angleToMove = SmartDashboard.getNumber("Angle to move", 90);
		feetToMove = SmartDashboard.getNumber("Feet to move", 3);
		gyro.reset();
	}
	double angleToMove;
	double feetToMove;
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Left Distance", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("Right Distance", rightDriveEncoder.getDistance());
		
		if(xbox.getAButton()){
			turn(angleToMove);
		}
		
		if(xbox.getYButton()){
			drive(feetToMove);
		}
		if (xbox.getBButton()){
			climber.set(0.5);
		}
		else
		{
			climber.set(0);
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
	public void drive(double feet) {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		//convert to inches
		double inchesToMove = feet * 12;
		pidDriveLeftDrive.setSetpoint(-inchesToMove);
		pidDriveRightDrive.setSetpoint(-inchesToMove);
		pidDriveLeftDrive.enable();
		pidDriveRightDrive.enable();
		loop: while(true) {
			if(Math.abs(rightDriveEncoder.getDistance()) >= Math.abs(inchesToMove) &&
					Math.abs(leftDriveEncoder.getDistance()) >= Math.abs(inchesToMove))
			{	
				pidDriveLeftDrive.disable();
				pidDriveRightDrive.disable();
				break loop;
			}
		}
	}
	public void turn(double degrees){
		gyro.reset();
		//half the degrees
		double degreesToMove = degrees / 2;
		pidTurnLeftDrive.setSetpoint(degreesToMove);
		pidTurnRightDrive.setSetpoint(-degreesToMove);
		pidTurnLeftDrive.enable();
		pidTurnRightDrive.enable();
		loop: while(true) {
			if(Math.abs(gyro.getAngle()) >= Math.abs(degreesToMove))
			{
				pidTurnLeftDrive.disable();
				pidTurnRightDrive.disable();
				break loop;
			}
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