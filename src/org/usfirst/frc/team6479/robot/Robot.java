package org.usfirst.frc.team6479.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	//SendableChooser<String> autoChooser;
	String autoSelected;
	//SendableChooser<String> teleChooser;
	//String teleSelected;
	XboxController xbox;
	
	//initializes proportional integral derivative controls
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

	// the motor controllers for the drive train
	Spark leftDrive;
	Spark rightDrive;
	CustomDrive driveTrain;
	Victor climber;
	
	//timer for uto
	Timer timer;

	// enums for left and right joystick on
	public enum JoystickOn
	{
		LEFT, RIGHT
	}
	// which stick is on
	JoystickOn stickOn;
	
	//init encoders
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;
	//init sonar
	RangeFinderAnalog sonar;
	
	//init gyro
	ADXRS450Gyro gyro;
	
	// camera thread
	Thread thread;
	private double centerX = 0.0;
	private boolean turn = false;
	private boolean inGeneralPosition;
	private boolean stopCamera;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	private int driverCounter;
	@Override
	public void robotInit()
	{
		timer = new Timer();
		stopCamera = false;
		
		gyro = new ADXRS450Gyro();
		gyro.startThread();

		sonar = new RangeFinderAnalog(0);
		
		leftDrive = new Spark(0);
		rightDrive = new Spark(1);
		
		// left drive is inverted since both motors are built identical
		leftDrive.setInverted(true);
		driveTrain = new CustomDrive(leftDrive, rightDrive);
		
		xbox = new XboxController(0);
		
		climber = new Victor(2);

		//pid tuning
		pidTP = .02;
		pidTI = 0;
		pidTD = .02;
		pidTF = .01;
		// enable PID's
		pidTurnLeftDrive = new PIDController(pidTP, pidTI, pidTD, pidTF, gyro, leftDrive);
		pidTurnRightDrive = new PIDController(pidTP, pidTI, pidTD, pidTF, gyro, rightDrive);
		pidTurnLeftDrive.setInputRange(-180, 180);
		pidTurnRightDrive.setInputRange(-180, 180);
		pidTurnLeftDrive.setOutputRange(-.5, .5);
		pidTurnRightDrive.setOutputRange(-.5, .5);
		pidTurnLeftDrive.setAbsoluteTolerance(1);
		pidTurnRightDrive.setAbsoluteTolerance(1);
		pidTurnLeftDrive.setContinuous(true);
		pidTurnRightDrive.setContinuous(true);

		// init the encoders
		leftDriveEncoder = new Encoder(1, 2, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k4X);
		// set the time until when the robot is considered stopped, set in seconds
		leftDriveEncoder.setMaxPeriod(.05);
		rightDriveEncoder.setMaxPeriod(.05);
		// set distance per pulse to be 1 inch
		rightDriveEncoder.setDistancePerPulse((6 * Math.PI) / 360);
		leftDriveEncoder.setDistancePerPulse((6 * Math.PI) / 360);

		pidDP = .01;
		pidDI = 0;
		pidDD = 0;
		pidDF = 0;
		// enable PID's
		pidDriveLeftDrive = new PIDController(pidDP, pidDI, pidDD, pidDF, leftDriveEncoder, leftDrive);
		pidDriveRightDrive = new PIDController(pidDP, pidDI, pidDD, pidDF, rightDriveEncoder, rightDrive);
		pidDriveLeftDrive.setInputRange(-1000, 1000);
		pidDriveRightDrive.setInputRange(-1000, 1000);
		pidDriveLeftDrive.setOutputRange(-.4, .4);
		pidDriveRightDrive.setOutputRange(-.4, .4);
		pidDriveLeftDrive.setAbsoluteTolerance(1);
		pidDriveRightDrive.setAbsoluteTolerance(1);
		pidDriveLeftDrive.setContinuous(true);
		pidDriveRightDrive.setContinuous(true);

		/*autoChooser = new SendableChooser<>();
		autoChooser.addDefault("Center Auto", "center");
		autoChooser.addObject("Left Auto", "left");
		autoChooser.addObject("Right Auto", "right");
		SmartDashboard.putData("Auto Choices", autoChooser);
		teleChooser = new SendableChooser<>();
		teleChooser.addDefault("Arcade Drive", "arcade");
		teleChooser.addObject("Racing Drive", "racing");
		teleChooser.addObject("Tank Drive", "tank");
		SmartDashboard.putData("Tele Choices", teleChooser);*/
		
		SmartDashboard.putString("DB/String 0", "center");
		driverInfo();
		driverCounter = 0;
		


		GripPipelineHSV grip = new GripPipelineHSV();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("FrontView", 0);
		camera.setResolution(320, 240);

		thread = new Thread(() ->
		{
			if(!stopCamera) {
			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 320, 240);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();
			
			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while(!Thread.interrupted())
			{
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if(cvSink.grabFrame(mat) == 0)
				{
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				
				// process image
				grip.process(mat);
				
				ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
				
				Rect r = null;
				if(contours.size() > 0)
				{
					r = Imgproc.boundingRect(contours.get(0));
				}
				
				Rect r2 = null;
				if(contours.size() > 1)
				{
					r2 = Imgproc.boundingRect(contours.get(1));
				}

				Point center = null;
				if(r != null && r2 != null)
				{
					center = new Point((r.x + r.width + r2.x) / 2, r.y + (r.height / 2));
				}
				if(center != null)
				{
			
					centerX = center.x;
					turn = true;
				}
				else if(r != null)
				{
		//			System.out.println("process");
					centerX = r.x + (r.width / 2);
					turn = true;
				}
			}
			}
		});

		thread.setDaemon(true);
		thread.start();
		//CameraServer.getInstance().startAutomaticCapture("BackView", 1);
	}
	
	public void driverInfo()
	{
		SmartDashboard.putString("DB/String 1", String.format("L Speed: %.3f", leftDrive.get()));
		SmartDashboard.putString("DB/String 2", String.format("R Speed: %.3f", rightDrive.get()));
		SmartDashboard.putString("DB/String 3", String.format("Distance: %.3f\"", sonar.getDistanceInInches()));
		SmartDashboard.putString("DB/String 4", String.format("Climber: %.3f", climber.get()));
		SmartDashboard.putString("DB/String 5", String.format("L Encoder: %.3f\"", leftDriveEncoder.getDistance()));
		SmartDashboard.putString("DB/String 6", String.format("R Encoder: %.3f\"", rightDriveEncoder.getDistance()));
		SmartDashboard.putString("DB/String 7", String.format("Gyro: %.3f", gyro.getAngle()));
		
		SmartDashboard.putBoolean("DB/LED 0", xbox.getBumper(Hand.kRight));
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
	public void autonomousInit()
	{
		// get the selected autonomous
		//autoSelected = autoChooser.getSelected();
		autoSelected = SmartDashboard.getString("DB/String 0", "center");

		// reset the encoders
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		gyro.reset();
		
		pidTurnLeftDrive.setPID(pidTP, pidTI, pidTD, pidTF);
		pidTurnRightDrive.setPID(pidTP, pidTI, pidTD, pidTF);

		pidDriveLeftDrive.setPID(pidDP, pidDI, pidDD, pidDF);
		pidDriveRightDrive.setPID(pidDP, pidDI, pidDD, pidDF);
	
		
		startDrive = true;
		startTurn = false;
		stopTurn = false;
		forward = false;
		atTarget = false;
		inGeneralPosition = false;
		
		driverInfo();
		driverCounter = 0;
		
		stopCamera = false;
	}
	/**
	 * This function is called periodically during autonomous
	 */
	
	private boolean forward = false;
	private boolean atTarget = false;
	
	@Override
	public void autonomousPeriodic()
	{
		// move into position using turn and drive methods
		
		switch(autoSelected)
		{
		case "forward":
			
			// Drive for 2 seconds
			if (timer.get() < 5.0) {
				driveTrain.drive(.3, 0.0); // drive forwards half speed
			} else {
				driveTrain.drive(0.0, 0.0); // stop robot
			}
		break;
		case "left":
			drive(111);
			//turn(60);
			turn(60, 1);
			break;
		case "right":
			drive(111);
			//turn(-60);
			turn(60, -1);
			break;
		case "center":
		default:
			//drive forward onto lift inches, lift is 111
			//drive(74.5);
			driveGyro(73);
			//drive(30);
			//turn(0);
			/*double distance =20;
			while(isAutonomous() && distance > 15){
				double angle = gyro.getAngle();
				
				driveTrain.drive(0.3, -angle*Kp);
				Timer.delay(0.05*(distance/15));
				driveTrain.drive(0, 0);
				Timer.delay(0.05);
				System.out.println(distance);
			}
			driveTrain.drive(0, 0);*/
		}
		
		//if robot has already moved into position
		if(inGeneralPosition)
		{
		//center robot on lift
		if(turn && !forward && !atTarget)
		{
			System.out.println("turn");
			
			if(centerX > 82)
			{
				leftDrive.set(-0.3);
				rightDrive.set(0.3);
				System.out.println("Turning right     X: " + centerX);
			}
			else if(centerX < 78)
			{
				leftDrive.set(0.3);
				rightDrive.set(-0.3);
				System.out.println("Turning left     X: " + centerX);
			}
			else
			{
				leftDrive.set(0);
				rightDrive.set(0);
				System.out.println("Stop       X: " + centerX);
				forward = true;
				gyro.reset();
			}
			turn = false;
		}
		Timer.delay(0.05);
		leftDrive.set(0);
		rightDrive.set(0);
		
		//find the distance to the target, then drive forward that amount
		if (forward){
		
			double distance = sonar.getDistanceInInches();
			while(isAutonomous() && distance > 15){
				double angle = gyro.getAngle();
				
				driveTrain.drive(0.3, -angle*Kp);
				Timer.delay(0.05*(distance/15));
				driveTrain.drive(0, 0);
				Timer.delay(0.05);
				distance = sonar.getDistanceInInches();
				System.out.println(distance);
			}
			driveTrain.drive(0, 0);
			System.out.println("STOP");
			forward = false;
			atTarget = true;
		}
		}
		driverCounter++;
		if (driverCounter == 3)
		{
			driverInfo();
			driverCounter = 0;
		}
	}

	private double Kp = 0.03;
	
	/**
	 * This function is called at the start of operator control
	 */
	@Override
	public void teleopInit()
	{
		//Kill the camera thread to improve performace
		stopCamera = true;
		/*try { thread.wait();}
		catch(InterruptedException e){e.printStackTrace();}*/
		
		// get the teleop driving config
		//teleSelected = teleChooser.getSelected();
		// if its for arcade, set right stick on
		/*if(teleSelected.equals("arcade"))
		{
			stickOn = JoystickOn.RIGHT;
		}*/

		// reset the encoders
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		gyro.reset();

		//climbSpeed = .5;
		driverInfo();
		driverCounter = 0;
	}
	//double angleToMove;
	//double inchesToMove;
	/**
	 * This function is called periodically during operator control
	 */
	//double climbSpeed;
	@Override
	public void teleopPeriodic()
	{
		if(xbox.getBumper(Hand.kRight))
		{
			climber.set(Math.abs(xbox.getY(Hand.kRight)));
		}
		else
		{
			climber.set(0);
		}
		racing();
		driverCounter++;
		if (driverCounter == 3)
		{
			driverInfo();
			driverCounter = 0;
		}
	}
	private boolean startDrive;
	public void drive(double inches)
	{
		if (startDrive)
		{
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		pidDriveLeftDrive.setSetpoint(-inches);
		pidDriveRightDrive.setSetpoint(-inches);
		pidDriveLeftDrive.enable();
		pidDriveRightDrive.enable();
		startDrive = false;
		}
		//loop: while(this.isAutonomous()) 
		//{
			if(Math.abs(rightDriveEncoder.getDistance()) >= Math.abs(inches) || 
					Math.abs(leftDriveEncoder.getDistance()) >= Math.abs(inches))
			{
				pidDriveLeftDrive.disable();
				pidDriveRightDrive.disable(); 
				//when this is done start turn
				startTurn = true;
				//break loop; 
			}
		//}
	}
	
	public void driveGyro(double inches)
	{
		if (startDrive)
		{
			while(isAutonomous() && (Math.abs(rightDriveEncoder.getDistance()) < Math.abs(inches) || 
					Math.abs(leftDriveEncoder.getDistance()) < Math.abs(inches))){
				double angle = gyro.getAngle();
				
				driveTrain.drive(0.4, -angle*0.03);
				Timer.delay(0.05);
				driveTrain.drive(0, 0);
				Timer.delay(0.05);
			//	distance = sonar.getDistanceInInches();
			//System.out.println(distance);
			}
			driveTrain.drive(0, 0);
				startTurn = true;
		}
	}
	private boolean startTurn;
	private boolean stopTurn;
	/*public void turn(double degrees)
	{
		// half the degrees
		double degreesToMove = degrees / 2;
		
		if (startTurn) {
			
		gyro.reset();
		pidTurnLeftDrive.setSetpoint(degreesToMove);
		pidTurnRightDrive.setSetpoint(-degreesToMove);
		pidTurnLeftDrive.enable();
		pidTurnRightDrive.enable();
		startTurn = false;
		stopTurn = true;
		}
		//loop: while(isAutonomous())
		//{
			if(Math.abs(gyro.getAngle()) >= Math.abs(degreesToMove) && stopTurn)
			{
				pidTurnLeftDrive.disable();
				pidTurnRightDrive.disable();
				//when done, prepare to go to vision targeting
				inGeneralPosition = true;
			//	break loop;
			}
		//}
	}*/
	public static boolean gyroUpdate = false;
	public void turn(double degrees, int direction){
		
		if(startTurn)
		{
		
		double angle = Math.abs(gyro.getAngle());
		System.out.println("Before Angle: " + angle);
	
		
		while (angle < degrees-4 && isAutonomous()){
			Timer.delay(0.004);
			if (gyroUpdate){
				angle = Math.abs(gyro.getAngle());
				
				System.out.println("Angle: " + angle);
				leftDrive.set(0.4*direction);
				rightDrive.set(0.4*direction);
			
				//comment this out if you want to try a different equation don't change this
			//	Timer.delay(0.5*(degrees-angle)/degrees + 0.001);
				if((0.5*(degrees-angle)/degrees + 0.001) <= 0){
					Timer.delay(0);
				}else{
					Timer.delay(0.3*(degrees-angle)/degrees + 0.001);
				}
				
				leftDrive.set(0);
				rightDrive.set(0);
				gyroUpdate = false;
				Timer.delay(0.03);
			
			}
		}
		leftDrive.set(0);
		rightDrive.set(0);
		System.out.println("Immediate Angle: " + gyro.getAngle());
		Timer.delay(5);
		System.out.println("After Angle: " + gyro.getAngle());
		}
	}
	
	public void racing()
	{
		double left = xbox.getRawAxis(2);
		double right = xbox.getRawAxis(3);
		// each trigger has an axis range of 0 to 1
		// to make left trigger reverse, subtract axis value from right trigger
		double throttle = right - left;
		double turn = xbox.getRawAxis(0) * -1;
		// invert
		/*if(throttle < 0){
			turn=turn*-1;
		}*/
		driveTrain.arcadeDrive(turn, throttle);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	public void arcade()
	{
		// if the right stick is on and the leftstick is clicked switch which stick is on to left stick
		if(stickOn == JoystickOn.RIGHT && xbox.getStickButton(Hand.kLeft))
			stickOn = JoystickOn.LEFT;
		// if the left stick is on and the rightstick is clicked switch which stick is on to right stick
		if(stickOn == JoystickOn.LEFT && xbox.getStickButton(Hand.kRight))
			stickOn = JoystickOn.RIGHT;
		// if the right stick is on, use full drive
		if(stickOn == JoystickOn.RIGHT)
			fullDrive();
		// if the left stick is on, use fine drive
		if(stickOn == JoystickOn.LEFT)
			fineDrive();
	}
	// this function is called when the right joystick on the xbox cotroller is being used gives full control, going from -1 to 1
	public void fullDrive()
	{
		// get x and y
		double x = xbox.getRawAxis(4);
		
		double y = xbox.getRawAxis(5);
		// invert
		driveTrain.arcadeDrive(x * -1, y * -1, true);
	}
	// this function is called when the left joystick on the xbox cotroller is being used gives fine control, going from -.5 to .5
	public void fineDrive()
	{
		// get x and y
		double x = xbox.getRawAxis(0);
		double y = xbox.getRawAxis(1);
		// scale the speed
		double scaleFactor = .5;
		// invert and scale
		driveTrain.arcadeDrive(x * scaleFactor * -1, y * scaleFactor * -1, false);
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override public void testPeriodic(){}
}