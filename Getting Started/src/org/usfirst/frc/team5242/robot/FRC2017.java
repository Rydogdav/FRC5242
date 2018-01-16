package org.usfirst.frc.team5242.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.ADXL345_SPI;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("deprecation")
public class FRC2017 extends SampleRobot {
	/////////////////////////////////////////////////////////////////////////
	// Variable and object declarations 
	/////////////////////////////////////////////////////////////////////////
	//CONSTANTS*****************************************************																																								 
	static final double driveTrainMultiplierL = 0.55; //Multiplier to adjust motor speed .55
	static final double driveTrainMultiplierM = 0.65; //.65
	static final double driveTrainMultiplierH = 1.00;// Originally 0.75
	static final double leftDriveMultiplier = 1.00;//Made to make the drive equal
	static final double INVERSE_DRIVE = -1;
	static final int DRIVE_FORWARD = 1;
	static final int DRIVE_ROTATE = -1;
	//constants for drive mode
	static final int SPEED_CORRECTION = 1;
	static final int VISION_CORRECTION = 2;
	static final int MANUAL_CORRECTION = 3;
	static final double RobotWheelBase = 19;
	static final double WheelDiameter = 6.0; 
	static final double PI = 3.141;
	static final int pulsesPerRev = 2048;
	static final double inchesPerRev = WheelDiameter * PI;
	//Factor to control sensitivity of vision correction
	static final double visionGain = 5.0;
	//Auto mode distances and angles
	static final double AutoleftDistance = 0.0;
	static final double AutoRightDistance = 0.0;
	static final float AutoCenterDistance = 74;
	static final double AutoLeftTurnAngle = 30.0;
	static final double AutoRIghtTurnAngle = -30.0;
	static final double UltrasonicPegRange = 0.0;
	//Acceleration and decelleration factors
	static final double acceleration = 0.04;
	static final double deceleration = 1.0;
	//Distance fudge factor gain to account for decelleration distance adder
	static final double StopDistanceOffset = 24.0 * deceleration / 0.02;

	//Vision storage
	int TotalPixyBlockCount = 2;//Max objects we are interested in
	int PixyX[] = new int[TotalPixyBlockCount];
	int PixyY[] = new int[TotalPixyBlockCount];
	int PixyWidth[] = new int[TotalPixyBlockCount];
	int PixyHeight[] = new int[TotalPixyBlockCount];
	int PixySignatureNumber[] = new int[TotalPixyBlockCount];
	double CentroidX;
	double CentroidY;
	
	 //Drive Objects
    BuiltInAccelerometer accel;
	RobotDrive elv;
	RobotDrive climb = new RobotDrive(4, 5);
	//RobotDrive climb;
	//Joystick stick;
	//Joystick controller;
	Accelerometer accelerometer;
	//double eCount = 0;
	
	
	//Camera Objects
	CameraServer server;
	Double yawInitial = 0.5;			
	Double pitchInitial = 0.5;
	Servo yaw = new Servo(9);
	Servo pitch = new Servo(8);

	//Declare the robot drive type and motor count
	RobotDrive Drive = new RobotDrive(3, 2, 1, 0);
	//Declare the joysticks used for driving
	Joystick stick = new Joystick(0); // set to ID 1 in DriverStation
	//Joystick rightStick = new Joystick(1); // set to ID 2 in DriverStation
	//Smart dashboard seleciton options for starting position
	final String AutoLeft = "Left";
	final String AutoCenter = "Center";
	final String AutoRight = "Right";
	SendableChooser<String> AutoModeSelection = new SendableChooser<>();
	//Smart dashboard seleciton options for alliance (i.e. which side is the boiler on). Not really needed since could use .alliance setting but don't really trust it 
	final String AllianceRed = "Red";
	final String AllianceBlue = "Blue";
	SendableChooser<String> AutoAllianceSelection = new SendableChooser<>();
	//Declare distance measurement encoders
	Encoder leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
	Encoder rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
	//Declare ultra-sonic transducer
	Ultrasonic UltrasonicTransducer = new Ultrasonic(6, 7);
	//I2C object for Pixy communication. This too WAY to long to find out how to do !!!
	I2C PixyI2C = new I2C(I2C.Port.kOnboard, 0x54);
	

		Double acc = 0.0;
	    Double currentLeftSpeed = 0.0;
		Double currentRightSpeed = 0.0;
		Double targetLeftSpeed = 0.0;
		Double targetRightSpeed = 0.0;
		Double currentMultiplier = 0.65;
	
	/////////////////////////////////////////////////////////////////////////
	// Code start
	/////////////////////////////////////////////////////////////////////////
	public void Robot() 
	{
		Drive.setExpiration(0.1); //unneccessary?
	}

	public void initializeDashboard()
	{
		SmartDashboard.putString("Auto Selected","None");
		SmartDashboard.putNumber("left Enocder", 0);
		SmartDashboard.putNumber("right Enocder", 0);
		SmartDashboard.putNumber("Ultrasonic range", 0);
		SmartDashboard.putNumber("Left motor", 0);
		SmartDashboard.putNumber("Right motor", 0);

    	SmartDashboard.putNumber("Centroid X", 0);
    	SmartDashboard.putNumber("Centroid Y", 0);
    	SmartDashboard.putNumber("Vision error", 0);
    	SmartDashboard.putNumber("Vision error dial", 0);
		
		SmartDashboard.putNumber("SignatureNumber", 0);
		SmartDashboard.putNumber("X", 0);
		SmartDashboard.putNumber("Y", 0);
		SmartDashboard.putNumber("Width", 0);
		SmartDashboard.putNumber("Height", 0);
	
		SmartDashboard.putNumber("SignatureNumber2", 0);
		SmartDashboard.putNumber("X2", 0);
		SmartDashboard.putNumber("Y2", 0);
		SmartDashboard.putNumber("Width2", 0);
		SmartDashboard.putNumber("Height2", 0);
		
		SmartDashboard.putNumber("Left distance", 0);
		SmartDashboard.putNumber("Right distance", 0);
		SmartDashboard.putNumber("Speed Error", 0);
		SmartDashboard.putNumber("Distance", 0);
	}
	
	//Following function is called when the robot is initialized at game start before auto mode starts
	@Override
	public void robotInit() 
	{
		//Put starting position selection up on smart dashboard
		AutoModeSelection.addDefault("Left start", AutoLeft);
		AutoModeSelection.addObject("Center start", AutoCenter);
		AutoModeSelection.addObject("Right start", AutoRight);
		SmartDashboard.putData("Auto start position 1", AutoModeSelection);
		//Put alliance selection up on smart dashboard
		AutoAllianceSelection.addObject("Red Alliance", AllianceRed);
		AutoAllianceSelection.addObject("Blue  Alliance", AllianceBlue);
		SmartDashboard.putData("Current Alliance 1", AutoAllianceSelection);
		//Configure the encoders
    	rightEncoder.setDistancePerPulse(inchesPerRev / pulsesPerRev);
    	leftEncoder.setDistancePerPulse(inchesPerRev / pulsesPerRev);
    	//Configure the ultrasonic transducer
    	UltrasonicTransducer.setEnabled(true);
    	UltrasonicTransducer.setAutomaticMode(true);
    	//Initialize the smart dashboard display
    	initializeDashboard();
		CameraServer.getInstance().startAutomaticCapture();

    	
	}

	//Following function is called once autonomous mode is started
	@Override
	public void autonomous() 
	{
    	//Now get the auto mode selection
		int clockAntiClock;
		String autoSelected = AutoModeSelection.getSelected();
		SmartDashboard.putString("Auto Selected",autoSelected);

		//Disable safety checking for auton mode for climber
		climb.setSafetyEnabled(false);
		
		String Alliance = AutoAllianceSelection.getSelected();
		
		switch (Alliance)
		{
		case AllianceRed:
			clockAntiClock = 1;
			break;
		default:
			clockAntiClock = -1;
		
		}
		
		switch (autoSelected) 
		{
			case AutoLeft:
				//doDrive(24, (float) 0.5, DRIVE_FORWARD, 0, VISION_CORRECTION);
				//turnAngle(360 * clockAntiClock, (float) 0.4);
				autoMode1();
				break;
			case AutoRight:
				//doDrive(36, (float) 0.5, DRIVE_FORWARD, 0, VISION_CORRECTION);	
				autoMode2();
				//turnAngle(90 * clockAntiClock, (float) 0.4);
				break;
			case AutoCenter:
			default:
				//turnAngle(180 * clockAntiClock, (float) 0.4);
				//turnAngle(90, (float) 0.5);	
				//doDrive(48, (float) 0.5, DRIVE_FORWARD, 0, VISION_CORRECTION);	
				autoMode3();
				break;
		}
	}

	//Following function is called once tele-op mode is started 
	@Override
	public void operatorControl() 
	{/*
		//Make sure the motor safeties are on
		Drive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) 
		{
			DoTankDrive(ControlStick.getRawAxis(1),ControlStick.getRawAxis(5));
			Timer.delay(0.005); // wait for a motor update time
		}*/
		while(isOperatorControl() && isEnabled())
		//while(true)
		{
			SmartDashboard.putNumber("left Enocder", leftEncoder.getDistance());
			SmartDashboard.putNumber("right Enocder", rightEncoder.getDistance());

	    	currentLeftSpeed = stick.getRawAxis(1);     //Left analog joystick
			currentRightSpeed = stick.getRawAxis(5);    //Right analog joystick
	    	
			if (stick.getRawAxis(3) > 0.10) 
			{
				currentMultiplier = driveTrainMultiplierH;  
			}
			else if (stick.getRawAxis(2) > 0.10) 
			{
				currentMultiplier = driveTrainMultiplierL;
			}
			else 
			{
				currentMultiplier = driveTrainMultiplierM;		//Default speed is now in "medium" mode, as requested	
			}

			if (targetLeftSpeed > currentLeftSpeed) 
			{
				currentLeftSpeed = currentLeftSpeed + acc;
					
					if (currentLeftSpeed > targetLeftSpeed) 
					{
						currentLeftSpeed = targetLeftSpeed;
					}
								
			}	
			else if (targetLeftSpeed < currentLeftSpeed) 
			{
				currentLeftSpeed = currentLeftSpeed - acc;
				
				if (currentLeftSpeed < targetLeftSpeed)
				{
					currentLeftSpeed = targetLeftSpeed;
				}
				

			}
			
			if (targetRightSpeed > currentRightSpeed) 
			{
				currentRightSpeed = currentRightSpeed + acc;
					
					if (currentRightSpeed > targetRightSpeed)
					{
						currentRightSpeed = targetRightSpeed;
					}			
			}	
			else if (targetRightSpeed < currentRightSpeed) 
			{
				currentRightSpeed = currentRightSpeed - acc;
				
				if (targetRightSpeed > currentRightSpeed) 
				{
					currentRightSpeed = targetRightSpeed;
				}
				
			}
		
			Drive.tankDrive(INVERSE_DRIVE * currentLeftSpeed * currentMultiplier , 
					INVERSE_DRIVE * currentRightSpeed * currentMultiplier);

		
			/*
			//Camera code
			//Uses the D-Pad to control camera
			//If D-Pad up is pressed
	    	if (stick.getPOV() == 90) {
	    		yawInitial = yawInitial + 0.01;
	    		yaw.set(yawInitial);    		
	    	}
	    	//If D-Pad down is pressed
	    	else if (stick.getPOV() == 270) {
	    		yawInitial = yawInitial - 0.01;
	    		yaw.set(yawInitial);
	    	}
	    	
	    	//D-Pad right pressed
	    	if (stick.getPOV() == 0) {
	    		pitchInitial = pitchInitial - 0.01;
	    		pitch.set(pitchInitial);
	    	}
	    	
	    	//D-Pad left pressed
	    	else if (stick.getPOV() == 180) {
	    		pitchInitial = pitchInitial + 0.01;
	    		pitch.set(pitchInitial);
	    	} 	
	    	*/
			if (stick.getRawButton(5))
			{
				yaw.set(1);
				pitch.set(.75);
			}
			else 
			{
				yaw.set(0);
				pitch.set(.80);
				
				
				
			}			
	    	//climb motor code
	    	if (stick.getRawButton(6)) 
	    	{
	    		climb.tankDrive(1.0 * currentMultiplier, -1.0 * currentMultiplier);
	    	}
	    	else 
	    	{
	    		climb.tankDrive(0.0, 0.0);   		
	    		
	    	}
	    	
	    	if(stick.getRawButton(1))
	    	{
	    		SmartDashboard.putNumber("Vision error", getVisionError());
	    	}

	    	if (stick.getRawButton(3))
			{
				doDrive((float)10000.0, (float) 0.45 ,DRIVE_FORWARD,(float) 5.0, MANUAL_CORRECTION);
			}
	    	
	    	//test
	    	if (stick.getRawButton(2))
			{
				doDrive((float)10000.0, (float) 0.0, DRIVE_FORWARD,(float) 5.0, MANUAL_CORRECTION);
			}
		}
		
		
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() 
	{
	}
	
	
	
	
	public void UltrasonicTest()
	{
		float PingDistance;
		
		while(true)
		{
			PingDistance = (float) UltrasonicTransducer.getRangeInches();
			SmartDashboard.putNumber("Ulrtasonic range", PingDistance);
			Timer.delay(0.05);
		}
		
	}
	
	public void PlacePegFromStart(float Direction)
	{
		//Drive forward a bit
		doDrive((float) 74.5,(float) 0.5, DRIVE_FORWARD, 0, SPEED_CORRECTION);
		//Rotate clockwise about 30 degrees, should be pointing roughly towards the peg
		turnAngle(Direction * 75, (float) 0.5);
		//Drive forward a bit until we think we reached the peg. 3" away should be good
		//doDrive((float) 75.0,(float) 0.5, DRIVE_FORWARD, (float) 12.0, VISION_CORRECTION);		
		doDrive((float) 75.0,(float) 0.5, DRIVE_FORWARD, (float) 12.0, SPEED_CORRECTION);
		Drive.setSafetyEnabled(false);
		DoTankDrive(0.3, 0.4);
		//Wait until we think the gear has been removed
		Timer.delay(5);
		Drive.setSafetyEnabled(true);
		//Back off the peg
		//doDrive((float) -36.0,(float) 0.5, DRIVE_FORWARD, 0, SPEED_CORRECTION);
		//Rotate ready to head off down the playfield
		//turnAngle(-30 * Direction, (float) 0.5);
		//Move forward so maximized auto period
		//doDrive((float) 48.0,(float) 0.5, DRIVE_FORWARD, 0, SPEED_CORRECTION);
	}

	public void autoMode1()
	{
		//Drive.setSafetyEnabled(false);//If start using motors then should remove this and ensure motors are updated
		//leftEncoder.reset();
		//rightEncoder.reset();
		//Auto mode 1 is a left starting point
		PlacePegFromStart(1);
		/*while (true)
		{
			double currentLeftDistance = leftEncoder.getDistance();
			double currentRightDistance = rightEncoder.getDistance();
			SmartDashboard.putNumber("Left distance", currentLeftDistance);
        	SmartDashboard.putNumber("Right distance", currentRightDistance);
		}*/
	}
	
	public void DoTankDrive(double LeftSpeed, double RightSpeed)
	{
		//Wrapper function so that we can display the speed percentages
		Drive.tankDrive(LeftSpeed, RightSpeed);
		SmartDashboard.putNumber("Left motor", LeftSpeed * 100);
		SmartDashboard.putNumber("Right motor", RightSpeed * 100);
	}

	public void autoMode2()
	{
		
		//doDrive((float) 100.0,(float) 0.5, DRIVE_FORWARD, 0);		
		PlacePegFromStart(-1);
	}	
	
	public void autoMode3()
	{
		//Drive forward up to "AutoCenterDistance" inches at half power,forward, until 4 inches from wall, use vision for direction correction 
		//doDrive(AutoCenterDistance, (float) 0.5, DRIVE_FORWARD, 12, VISION_CORRECTION);
		doDrive(AutoCenterDistance, (float) 0.5, DRIVE_FORWARD, 12, SPEED_CORRECTION);
		Drive.setSafetyEnabled(false);
		DoTankDrive(0.3, 0.4);
		//Wait until we think the gear has been removed
		Timer.delay(5);
		Drive.setSafetyEnabled(true);

//		while(true)
//		{
//			getVisionError();
//			Timer.delay(0.05);
//		}
	}	
	
	public void turnAngle(float Angle, float Speed)
	{
		float AngularDistance;
		
		//Calculate the linear angular distance that the left/right wheels should move to give the requested angle of rotation
		AngularDistance = (float) (PI * RobotWheelBase * Angle/360) * (float) 0.7;
		doDrive(AngularDistance, Speed, DRIVE_ROTATE, 0, SPEED_CORRECTION);
	}
	
	//Read the ultrasonic transducer 4 times and take the average. This gives more stable readings
	public float GetAveragedUltrasonicDistance()
	{
		float Distance;
		
		Distance = (float) ((UltrasonicTransducer.getRangeInches() + UltrasonicTransducer.getRangeInches() + UltrasonicTransducer.getRangeInches() + UltrasonicTransducer.getRangeInches()) / 4);
		
		//Zero is a special case meaning no measurements available. Can't return zero since will cause distance comparisons to complete
		if (Distance == 0)
			return 10000;
		else
			return Distance;  
	}
	
	//Synchronize the Pixy read data. 
	//Note, if there are no objects found then Pixy is a PITA and returns zeros. Unfortunately there is no way to determine if this is really 
	//"nothing" or a zero coordinate, so we must manually check for SOMETHING and if all zeroes and no sync within 16 bytes then "nothing" found !!!
	//To add insult to injury, if you don't read the data soon enough then you can often miss objects since it looks like the Pixy
	//restarts a new block at the video frame and NOT when you have read the data. Absolute garbage !!!
	public int GetPixyObjects()
	{
		int CheckCount = 0;
		int Checksum;
		int ValidCount = 0;
		int Signature;
		int X;
		int Y;
		int Width;
		int Height;
		int Count = 0;
		int Nudge = 0;
		int[] Buffer = new int[16];
		
		byte[] A = new byte[1];
		byte[] B = new byte[1];
		byte[] C = new byte[1];
		byte[] D = new byte[1];
		
		//Default empty arrays so we can detect good/bad detection later
		PixyX[0] = 0;
		PixyY[0] = 0;
		PixyWidth[0] = 0;
		PixyHeight[0] = 0;
		PixyX[1] = 0;
		PixyY[1] = 0;
		PixyWidth[1] = 0;
		PixyHeight[1] = 0;
		
		//Sync pattern is 85 followed by -86. If we don't find the sync code then make sure we exit since we don't want to hang up here since "nothing" returns zeros, and there is no way to know if the data is really zero or "nothing" found!!!
		//Due to Pixy bizarre behavior we need to look for block start then read quickly before next frame start!!!
//		while ((D[0] != 85) && (C[0] != -86) && (B[0] != 85) && (A[0] != -86) && (CheckCount < 64))
		while ((B[0] != 85) && (A[0] != -86) && (CheckCount < 64))
		{
			//Slide the buffer data ready to read next data
			D[0] = C[0];			
			C[0] = B[0];			
			B[0] = A[0];
			//Read new data into Buffer[0]
			PixyI2C.readOnly(A, 1);
			//Increment our checking counter
			CheckCount++;
		}
		
		if (CheckCount == 64)
		{
			//Didn't find the double sync codes, so assume didn't read correct data or none available
			return 0;
		}
		else
		{
			//Capture the data as fast as possible into a buffer so that Pixy doesn't start a new block. Fingers crossed though
			for (Count = 0; Count < 16; Count++)
				Buffer[Count] = GetPixyWord();
			//Nasty way to do this but Pixy REALLY makes life difficult!!!!!
			//Now manually read object 1 data
			Checksum = Buffer[1 + Nudge];
//			//If the checksum is another sync word then, well, sigh !!!
//			if ((Checksum == 0xaa55) || (Checksum == 0xaa56))
//			{
//				//this is a double sync so ignore first value 
//				Nudge++;
//			}
			Checksum = Buffer[1 + Nudge];
			Signature = Buffer[2 + Nudge];
			X = Buffer[3 + Nudge];
			Y = Buffer[4 + Nudge];
			Width = Buffer[5 + Nudge];
			Height = Buffer[6 + Nudge];
			if (Signature == 1)//Signature should match item 1 for our simple, single object requirements
			{
				//Signature seems correct so store the data
				PixyX[0] = X;
				PixyY[0] = Y;
				PixyWidth[0] = Width;
				PixyHeight[0] = Height;
				ValidCount++;
			}
			//Now manually read object 2 data
			Checksum = Buffer[8 + Nudge];
//			//If the checksum is another sync word then, well, sigh !!!
//			if ((Checksum == 0xaa55) || (Checksum == 0xaa56))
//			{
//				//this is a double sync so ignore first value 
//				Nudge++;
//			}
//			Checksum = Buffer[8 + Nudge];
			Signature = Buffer[9 + Nudge];
			X = Buffer[10 + Nudge];
			Y = Buffer[11 + Nudge];
			Width = Buffer[12 + Nudge];
			Height = Buffer[13 + Nudge];
			if (Signature == 1)//Signature should match item 1 for our simple, single object requirements
			{
				//Signature seems correct so store the data
				PixyX[1] = X;
				PixyY[1] = Y;
				PixyWidth[1] = Width;
				PixyHeight[1] = Height;
				ValidCount++;
			}
			return ValidCount;
		}
	}
	
	public int GetPixyWord()
	{
		int Upper;
		int Lower;
		
		byte[] Buffer = new byte[2];
    	PixyI2C.readOnly(Buffer, 2);
    	//Following needed to effectively convert from signed byte to unsigned
    	Lower = Buffer[0] & 0xff;
    	Upper = Buffer[1] & 0xff;
    	return Lower  + (Upper * 256);
	}
	

	//Read data from the Pixy camera
	//This sets global variables for the centroid since I don't know how to pass by reference in Java
	public int  GetPixyCentroids()
	{
		
		if (GetPixyObjects() != 2)
		{
			//Values not valid
			return 0;
		}
		else
		{
			//Display the new object settings
	    	SmartDashboard.putNumber("SignatureNumber", PixySignatureNumber[0]);
	    	SmartDashboard.putNumber("X", PixyX[0]);
	    	SmartDashboard.putNumber("Y", PixyY[0]);
	    	SmartDashboard.putNumber("Width", PixyWidth[0]);
	    	SmartDashboard.putNumber("Height", PixyHeight[0]);
	
			//Display the new object settings
	    	SmartDashboard.putNumber("SignatureNumber2", PixySignatureNumber[1]);
	    	SmartDashboard.putNumber("X2", PixyX[1]);
	    	SmartDashboard.putNumber("Y2", PixyY[1]);
	    	SmartDashboard.putNumber("Width2", PixyWidth[1]);
	    	SmartDashboard.putNumber("Height2", PixyHeight[1]);
	    	
	    	return 2;
		}
    	
//    	//Need to wait some time to allow Pixy capture period
//    	Timer.delay(1.0/40);
    	
	}

	//Drive the left & right motors for a pre-determined distance at a specified motor speed (really motor power)
	//Since straight line and rotation are so similar we can use the same function with a minor modification.
	//'Mode' should be 1 for straight and -1 for rotation, and is used as a multiplier for the right side motor
	//Added 'StopAtDistance so that we can use he same function for ultrasonic range finding. 
	//StopAtDistance is the distance from wall preferred. In this case 'distance' becomes a max drive distance
	//Set 'SetAtDistance' to zero or something negative to disable
	public void doDrive(float distance, float speed, int Mode, float StopAtDistance, int correctionMode)
    {
    	double currentSpeed;
    	double currentLeftDistance = StopDistanceOffset * speed;//Offset to account for decelleration distance
    	double currentRightDistance = StopDistanceOffset * speed;
    	double SpeedError = 1.0;
    	double CorrectionFactor;
    	double CorrectionGain;
    	double driveDelay;
    	double LeftPower;
    	double RightPower;
    	double oldLeftDistance;
    	double oldRightDistance;
    	double direction;
    	
    	currentSpeed = 0.0;
    	currentLeftDistance = 0.0;
    	currentRightDistance = 0.0;
    	driveDelay = 0.005;
    	CorrectionFactor = 1.0;
    	CorrectionGain = 0.3;
    	if(distance < 0)
    		direction = -1;
    	else
    		direction = 1;
    	
    	
    	//Reset the encoder counters ready to measure the distance
    	rightEncoder.reset();
    	leftEncoder.reset();
    	
    	//Accelerate up to desired speed, then maintain speed
    	//Whilst the left wheels have not gone the requested distance...
    	while (((currentLeftDistance * direction) < (distance * direction)) && (GetAveragedUltrasonicDistance() > StopAtDistance) && (okToAutoDrive(correctionMode)))
    	{
    		//Are we up to speed yet?
    		if (currentSpeed < speed)
    		{
    			//Need to go faster
    			currentSpeed = currentSpeed + acceleration;
    		}
    		//Check how far each set of wheels have gone (really how much rotated, but should be OK for the moment)
    		currentLeftDistance = leftEncoder.getDistance();
    		currentRightDistance = Mode * rightEncoder.getDistance();
    		//Calculate a correction factor
    		if (correctionMode == SPEED_CORRECTION)
    		{
    			//Have left and right gone the same distance? If not then need to correct the power of one side to try and compensate
        		SpeedError = currentLeftDistance - currentRightDistance;
        		//'Mode' reverses the right side motor to allow same code to be used for rotation for the moment until gyroscopes are implemented
        		LeftPower = currentSpeed;
        		RightPower = Mode * currentSpeed * (CorrectionFactor + (SpeedError * CorrectionGain * direction));
    		}
    		else// if ((correctionMode == VISION_CORRECTION) || (correctionMode == MANUAL_CORRECTION))
    		{
    			//Rather than check the distance moved, check if going towards the targets and adjust based on that
        		SpeedError = getVisionError();
        		//'Mode' reverses the right side motor to allow same code to be used for rotation for the moment until gyroscopes are implemented
        		//For vision correction speed up one side and slow worn the other
        		LeftPower = currentSpeed * (CorrectionFactor + (SpeedError * CorrectionGain));
        		RightPower = Mode * currentSpeed * (CorrectionFactor + (SpeedError * CorrectionGain));
    		}
    		//Set the motors to the desired speed (note, really a power but OK for the moment)
    		DoTankDrive(LeftPower * direction, RightPower * direction);
    		//Display some stuff on the smart dashboard
        	SmartDashboard.putNumber("Left distance", currentLeftDistance);
        	SmartDashboard.putNumber("Right distance", currentRightDistance);
        	SmartDashboard.putNumber("Speed Error", SpeedError);
        	SmartDashboard.putNumber("Distance", distance);
        	//Wait a little time so not updating things too fast
    		Timer.delay(driveDelay);
    	}
    	//Done distance so decelerate to a stop
    	//Should really decelerate before gone full distance, but will do that at some later date
    /*	while (currentSpeed > 0)
    	{
    		currentSpeed = currentSpeed - deceleration;
    		if (currentSpeed < 0.0)
    			currentSpeed = 0.0;
    		DoTankDrive(currentSpeed, Mode * currentSpeed * (CorrectionFactor - (SpeedError * CorrectionGain)));
    		currentLeftDistance = leftEncoder.getDistance();
    		currentRightDistance = rightEncoder.getDistance();
        	SmartDashboard.putNumber("Left distance", currentLeftDistance);
        	SmartDashboard.putNumber("Right distance", currentRightDistance);
    		Timer.delay(driveDelay);
    	}*/
    	
    	double BreakingPower = -0.3;
    	double DistanceTravelled = 0;
    	
		currentLeftDistance = leftEncoder.getDistance();
		currentRightDistance = rightEncoder.getDistance();
    	do
    	{
        	oldLeftDistance = currentLeftDistance;
        	oldRightDistance = currentRightDistance;
    		DoTankDrive(BreakingPower * direction, BreakingPower * Mode * direction);
    		Timer.delay(driveDelay);
    		currentLeftDistance = leftEncoder.getDistance();
    		currentRightDistance = rightEncoder.getDistance();
    		DistanceTravelled = currentLeftDistance - oldLeftDistance; 
    	}while(DistanceTravelled > 0.0);
    	
		DoTankDrive(0.0, 0.0);
    	
    }
	
	public float getVisionError()
	{
		float error = 0;
		
		if (GetPixyCentroids() != 2)
		{
			error = 0;
	    	SmartDashboard.putNumber("Vision error", error);
	    	SmartDashboard.putNumber("Vision error dial", error);
		}
		else
		{
	    	CentroidX = (PixyX[0] + PixyX[1]) / 2;
	    	CentroidY = (PixyY[0] + PixyY[1]) / 2;
			error = -(float) (160 - CentroidX) * (float) visionGain;
	    	//Display the centroid location
	    	SmartDashboard.putNumber("Centroid X", CentroidX);
	    	SmartDashboard.putNumber("Centroid Y", CentroidY);
	    	SmartDashboard.putNumber("Vision error", error);
	    	SmartDashboard.putNumber("Vision error dial", error);
	    }
		
		//return error;
		return 0;
	}

		public boolean okToAutoDrive(int mode)
		{
			if (mode == MANUAL_CORRECTION)
			{
				if (stick.getRawButton(3) || stick.getRawButton(2))
					return true;
				else 	
					return false;
			}
			else return true;
		}
}
