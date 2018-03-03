package org.usfirst.frc.team1111.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends IterativeRobot {
	//Default values. These do not change when you re-deploy so when calibrating PID, use SmartDashboard, do not edit these variables.
	//NavX PID
	double nP = 0.04;
	double nI = 0.0;
	double nD = 0.0;
	AHRS navx = new AHRS(SPI.Port.kMXP);
	MiniPID navxPID = new MiniPID(nP, nI, nD);
	int desAngle = 90;
	
	//Encoder PID
	double eP = 0.00025;
	double eI = 0.0;
	double eD = 0.0;
	MiniPID encPID = new MiniPID(eP, eI, eD);
	final int TPI = 1871;
	double desDist = 12;
	
	//Motor Variables
	TalonSRX frontLeft;
	TalonSRX frontRight;
	TalonSRX backLeft;
	TalonSRX backRight;
	TalonSRX encoderLeft;
	TalonSRX encoderRight;
	
	//Misc Variables
	double speed;
	final double MAX_SPEED = .5;
	
	//TEST Chooser Variables
	SendableChooser<String> testChooser = new SendableChooser<>();
	final String TURN_CALIBRATION = "TC";
	final String ENC_CALIBRATION = "EC";
	final String TPI_CALIBRATION = "TPIC";
	String testSelected;
	
	//CHASSIS Chooser Variables
	SendableChooser<String> chassisChooser = new SendableChooser<>();
	final String PEREGRINE = "P";
	final String FAST_BIRD = "FB";
	String chassisSelected;
	
	Joystick joy;
	
	
	//=====BEGIN BASIC ROBOT CODE=====
	
		
	@Override
	public void robotInit() {
		SmartDashboard.putNumber("NavX-P:", nP); SmartDashboard.putNumber("NavX-I:", nI); SmartDashboard.putNumber("NavX-D:", nD);
		SmartDashboard.putNumber("Desired Angle:", desAngle);
		SmartDashboard.putNumber("Encoder-P:", eP); SmartDashboard.putNumber("Encoder-I:", eI); SmartDashboard.putNumber("Encoder-D:", eD);
		SmartDashboard.putNumber("Desired Distance:", desDist);
		
		navxPID.setOutputLimits(-MAX_SPEED, MAX_SPEED);
		encPID.setOutputLimits(-MAX_SPEED, MAX_SPEED);
		
		testChooser.addDefault("Turn Calibration", TURN_CALIBRATION);
		testChooser.addObject("Encoder Calibration", ENC_CALIBRATION);
		SmartDashboard.putData(testChooser);
		
		chassisChooser.addDefault("Peregrine", PEREGRINE);
		chassisChooser.addObject("Fast Bird", FAST_BIRD);
		SmartDashboard.putData(chassisChooser);
	}

	@Override
	public void autonomousInit() {
		//Gather data from the SmartDashboard
		testSelected = testChooser.getSelected();
		chassisSelected = chassisChooser.getSelected();
		desAngle = (int) SmartDashboard.getNumber("Desired Angle:", desAngle);
		desDist = SmartDashboard.getNumber("Desired Distance:", desDist);
		configureControl();
		
		//Analyze CHASSIS CHOOSER and instantiate appropriate motors for the chassis
		switch (chassisSelected) {
		case FAST_BIRD: //Instantiates motor variables for FAST BIRD
			frontLeft = new TalonSRX(37);
			frontRight = new TalonSRX(45);
			backLeft = new TalonSRX(51);
			backRight = new TalonSRX(57);
			encoderLeft = frontLeft;
			encoderRight = frontRight;
			break;
		default: //Instantiates motor variables for PEREGRINE
			frontLeft = new TalonSRX(46);
			frontRight = new TalonSRX(55);
			backLeft = new TalonSRX(47);
			backRight = new TalonSRX(39);
			encoderLeft = backLeft;
			encoderRight = frontRight;
			break;
		}
		
		//Reset and configure hardware for testing
		encoderLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
		encoderRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
		resetEncoder(encoderLeft);
		resetEncoder(encoderRight);
		navx.reset();
	}

	@Override
	public void autonomousPeriodic() {
		//Analyze TEST CHOOSER and run appropriate calibration tests
		switch (testSelected) {
		case ENC_CALIBRATION: //Drives a specified distance for ENCODER calibration
			driveDistance(desDist);
			break;
		case TPI_CALIBRATION: //Prints out the encoder ticks for TPI calibration
			SmartDashboard.putNumber("Ticks:", getTicks());
			break;
		default: //Turns to a specified angle for TURNING calibration
			turnTo(desAngle);
			break;
		}
	}
	
	public void testInit() {
		 joy = new Joystick(0);
	}
	
	public void testPeriodic() {
		drive(joy.getRawAxis(1), joy.getRawAxis(3)); //Uses joystick input to test alignment of left and right motors
	}
	
	
	//=====BEGIN TEST CODE=====
	
	
	/**
	 * Turns to a specified angle. Make negative to turn left.
	 * @param desAngle the desired angle (-180 to 180)
	 * @return if the robot has turned to the specified angle
	 */
	private boolean turnTo(int desAngle) {
		int deadzone = 3;
		int curAngle = Math.round(navx.getYaw()); //Rounds NavX data to nearest whole angle
		SmartDashboard.putNumber("Current Angle:", curAngle); // Debug
		navxPID.setSetpoint(desAngle);
		speed = navxPID.getOutput(curAngle);
		SmartDashboard.putNumber("Speed:", speed); //Debug

		if (isInRange(curAngle, desAngle, deadzone)) { //STOPS if in deadzone of angle
			drive(0); // STOPS motors
			SmartDashboard.putBoolean("Stopped Turning:", true); //Debug
			return true;
		} 
		else { //TURNS until facing a specified angle
			drive(-speed, speed); // Turn to the SHORTEST side. LEFT has to be negative as this is only way to turn shortest
			SmartDashboard.putBoolean("Stopped Turning:", false); //Debug
			return false;
		}
	}
	
	/**
	 * Drives a specified distance. Negative to go backwards.
	 * @param dist travel distance IN INCHES
	 * @return if the robot has traveled the specified distance
	 */
	public boolean driveDistance(double dist) {
		int deadzone = 1000;
		int desTicks = (int) (TPI * dist);
		encPID.setSetpoint(desTicks);
		int curTicks = getTicks();
		
		if (isInRange(curTicks, desTicks, deadzone)) { //STOPS if in deadzone of distance
			drive(0);
			SmartDashboard.putBoolean("Stopped Driving:", true); //Debug
			return true;
		}
		else { //DRIVES until traveled far enough to be in deadzone
			speed = encPID.getOutput(curTicks);
			drive(speed);
			SmartDashboard.putBoolean("Stopped Driving:", false); //Debug
			return false;
		}
	}
	
	
	//=====BEGIN UTILITY CODE=====
	
	
	/**
	 * Wrapper for driving at a specified speed. Drives all 4 DRIVETRAIN motors at the same speed.
	 * @param speed the speed (-1, 1) of the DRIVETRAIN motors
	 */
	private void drive(double speed) {
		speed *= 1; //Sets soft limit for speed
		
		frontLeft.set(ControlMode.PercentOutput, speed);
		frontRight.set(ControlMode.PercentOutput, speed);
		backLeft.set(ControlMode.PercentOutput, speed);
		backRight.set(ControlMode.PercentOutput, speed);
	}
	
	/**
	 * Wrapper for driving the LEFT and RIGHT drivetrain motors at a specified speed. Drives left and right independently
	 * @param left the speed for the left motors
	 * @param right the speed for the right motors
	 */
	private void drive(double left, double right) {
		left *= 1; right *= 1; //Sets soft limit for speed
		
		frontLeft.set(ControlMode.PercentOutput, left);
		frontRight.set(ControlMode.PercentOutput, right);
		backLeft.set(ControlMode.PercentOutput, left);
		backRight.set(ControlMode.PercentOutput, right);
	}
	
	/**
	 * Wrapper for getting PID variables from the SmartDashboard and updating the class variables
	 */
	private void updateValues() {
		nP = SmartDashboard.getNumber("NavX-P:", nP);
		nI = SmartDashboard.getNumber("NavX-I:", nI);
		nD = SmartDashboard.getNumber("NavX-D:", nD);
		eP = SmartDashboard.getNumber("Encoder-P:", eP);
		eI = SmartDashboard.getNumber("Encoder-I:", eI);
		eD = SmartDashboard.getNumber("Encoder-D:", eD);
	}
	
	/**
	 * Wrapper for updating the PID values and the PID algorithm with the new values
	 */
	private void configureControl() {
		updateValues();
		navxPID.setP(nP); navxPID.setI(nI); navxPID.setD(nD); //Sets NavX PID values
		encPID.setP(eP); encPID.setI(eI); encPID.setD(eD); //Sets encoder PID values
	}
	
	/**
	 * Wrapper for resetting the encoder of a motor
	 * @param motor the motor that the encoder is attached to
	 */
	private void resetEncoder(TalonSRX motor) {
		motor.setSelectedSensorPosition(0, 0, 200);
	}
	
	/**
	 * Wrapper for a try/catch block to get the encoder ticks
	 * @return the encoder ticks based on which one is working. Defaults to LEFT encoder.
	 */
	private int getTicks() {
		try {
			return encoderLeft.getSelectedSensorPosition(0);
		}
		catch (Exception e) {
			System.out.println(e);
			return encoderRight.getSelectedSensorPosition(0);
		}
	}
	
	/**
	 * Wrapper for testing if a value is within the deadzone of a target
	 * @param x the current value
	 * @param target the target value
	 * @param dz the deadzone around the target
	 * @return if the current value is within the deadzone of the target
	 */
	private boolean isInRange(long x, long target, long dz) {
		if (x > target-dz && x < target+dz) {
			return true;
		}
		else {
			return false;
		}
	}
}
