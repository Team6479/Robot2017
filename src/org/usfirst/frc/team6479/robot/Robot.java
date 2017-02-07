package org.usfirst.frc.team6479.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* TODO
 * Camera Tracking
 * Commands for testing robot testing
 * Test functions
 * Custom Dashboard
 * Misc controls for whatever robot needs to do (i.e. climb rope)
 * Make custom classes for autonomous
 * Make custom class for dashboard
 */

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{

	// the drivetrain
	Drivetrain drivetrain;
	XboxController xbox = new XboxController(0);

	// chooser for auto
	SendableChooser<String> autoChooser = new SendableChooser<>();
	// auto options
	final String autoDefault = "default";
	final String autoOne = "one";
	final String autoTwo = "two";
	// which auto is selected
	String autoSelected;

	// chooser for tele
	SendableChooser<String> teleChooser = new SendableChooser<>();
	// tele options
	final String teleArcade = "arcade";
	final String teleRacing = "racing";
	final String teleTank = "tank";

	// Teleop object, customized in teleopInit
	Teleop teleopDrive;

	// thread for camera
	Thread visionThread;

	// encoders
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;

	// gyro for autonoumous
	ADXRS450_Gyro gyro;
	double Kp = 0.03;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{

		// init drivetrain
		drivetrain = new Drivetrain(0, 1);

		autoChooser.addDefault("Default Auto", autoDefault);
		autoChooser.addObject("First Auto", autoOne);
		autoChooser.addObject("Second Auto", autoTwo);
		SmartDashboard.putData("Auto Choices", autoChooser);

		teleChooser.addObject("Arcade Drive", teleArcade);
		teleChooser.addObject("Racing Drive", teleRacing);
		teleChooser.addDefault("Tank Drive", teleTank);
		SmartDashboard.putData("Tele Choices", teleChooser);

		// init gyro
		gyro = new ADXRS450_Gyro();

		// init camera data reader
		GripPipeline pipe = new GripPipeline();

		// setup camera
		visionThread = new Thread(() ->
		{
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

				pipe.process(mat);
				mat = pipe.maskOutput();

				// Put cross hairs on the image
				int crossWidth = xRes / 20;
				int crossHeight = yRes / 20;
				Imgproc.line(mat, new Point((xRes / 2) - (crossWidth / 2), (yRes / 2)),
						new Point((xRes / 2) + (crossWidth / 2), (yRes / 2)), new Scalar(0, 255, 0), 5);
				Imgproc.line(mat, new Point((xRes / 2), (yRes / 2) + (crossHeight / 2)),
						new Point((xRes / 2), (yRes / 2) - (crossHeight / 2)), new Scalar(0, 255, 0), 5);
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
	public void autonomousInit()
	{
		// autoSelected = autoChooser.getSelected();
		// get the selected autonomous
		autoSelected = autoChooser.getSelected();

		// init the encoders
		leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k4X);
		// set the time until when the robot is considered stopped, set in
		// seconds
		leftDriveEncoder.setMaxPeriod(.05);
		rightDriveEncoder.setMaxPeriod(.05);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic()
	{
		// SmartDashboard.putNumber("Left Distance",
		// leftDriveEncoder.getDistance());
		// SmartDashboard.putNumber("Right Distance",
		// rightDriveEncoder.getDistance());

		/*	while(isAutonomous()){
		driveTrain.tankDrive(-.8, .8);
		Timer.delay(3);
		}
	*/

		// gyro.calibrate();
		gyro.reset();
		// while(isAutonomous() && (leftDriveEncoder.getDistance()<2000) &&
		// (rightDriveEncoder.getDistance()<2000)){
		while(isAutonomous() && (leftDriveEncoder.getDistance() < 1000) && (rightDriveEncoder.getDistance() < 1000))
		{
			double angle = gyro.getAngle();
			drivetrain.driveCurve(0.25, -angle * Kp);

			// System.out.println("Angle: " + angle);
			// System.out.println("Turn: " + -1*angle*Kp);
			Timer.delay(0.004);
		}

		drivetrain.driveCurve(0, 0);

		gyro.reset();
		System.out.println("Angel before:" + gyro.getAngle());
		while(isAutonomous() && Math.abs(gyro.getAngle()) <= 360)
		{
			drivetrain.drive(-.3, -.3);
		}
		drivetrain.drive(0, 0);
		Timer.delay(5);
		System.out.println("after" + gyro.getAngle());
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
	public void teleopInit()
	{
		// get the teleop driving config
		String teleSelected = teleChooser.getSelected();
		SmartDashboard.putString("Selected", teleSelected);

		// choose which teleop is selected
		switch(teleSelected)
		{
		case teleArcade:
			teleopDrive = new ArcadeDrive(drivetrain, 0);
			break;
		case teleRacing:
			teleopDrive = new RacingDrive(drivetrain, 0);
			break;
		case teleTank:
		default:
			teleopDrive = new TankDrive(drivetrain, 0);
			break;
		}

		// init the encoders
		leftDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k4X);
		// set the time until when the robot is considered stopped, set in
		// seconds
		leftDriveEncoder.setMaxPeriod(.05);
		rightDriveEncoder.setMaxPeriod(.05);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic()
	{

		SmartDashboard.putNumber("Left Distance", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("Right Distance", rightDriveEncoder.getDistance());
		SmartDashboard.putBoolean("Left Direction", leftDriveEncoder.getDirection());
		SmartDashboard.putBoolean("Right Direction", rightDriveEncoder.getDirection());
		
		if(xbox.getAButton())
		{
			gyro.reset();
			System.out.println("Angel before:" + gyro.getAngle());
			while(Math.abs(gyro.getAngle()) <= 360)
			{		
				drivetrain.drive(-.3, -.3);

				// Timer.delay(0.004);
			}
			drivetrain.drive(0, 0);
			System.out.println("After:" + gyro.getAngle());

		}
		
		teleopDrive.drive();

	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic()
	{

	}
}