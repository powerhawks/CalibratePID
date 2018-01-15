package org.usfirst.frc.team1111.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	static final String chooseNavx = "NavX";
	static final String chooseTalonAbs = "Talon-Absolute";
	static final String chooseTalonRel = "Talon-Relative";
	String selected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	//Default values. These do not change when you re-deploy so when calibrating PID, use SmartDashboard, do not edit these variables.
	double pN = 0.01;
	double iN = 0;
	double dN = 0;
	double pT = 0.01;
	double iT = 0;
	double dT = 0;
	
	AHRS navx = new AHRS(SerialPort.Port.kMXP);
	MiniPID pid = new MiniPID(pN, iN, dN);
	double speed;

	TalonSRX motor = new TalonSRX(1111); //TODO: configure with proper motor port
	FeedbackDevice encoderAbsolute = FeedbackDevice.CTRE_MagEncoder_Absolute;
	FeedbackDevice encoderRelative = FeedbackDevice.CTRE_MagEncoder_Relative;
	int encoderPPR_ABS = 1024;
	int encoderDRP_ABS = calcDPR(encoderPPR_ABS);
	int encoderPPR_REL = 1; //TODO: Figure out
	int encoderDRP_REL = calcDPR(encoderPPR_ABS);
	int feedbackDelay = 200;
	
	TalonSRX frontLeft = new TalonSRX(1111); //TODO: configure with proper motor port
	TalonSRX frontRight = new TalonSRX(1111); //TODO: configure with proper motor port
	TalonSRX backLeft = new TalonSRX(1111); //TODO: configure with proper motor port
	TalonSRX backRight = new TalonSRX(1111); //TODO: configure with proper motor port

	
	@Override
	public void robotInit() {
		chooser.addObject("NavX", chooseNavx);
		chooser.addObject("Talon-Absolute",chooseTalonAbs);
		chooser.addObject("Talon-Relative",chooseTalonRel);
		SmartDashboard.putData(chooser);
		SmartDashboard.putNumber("NavX-P", pN); SmartDashboard.putNumber("NavX-I", iN); SmartDashboard.putNumber("NavX-D", dN);
		SmartDashboard.putNumber("Talon-P", pT); SmartDashboard.putNumber("Talon-I", iT); SmartDashboard.putNumber("Talon-D", dT);
		
		pid.setOutputLimits(-1, 1);		
	}

	@Override
	public void autonomousInit() {
		selected = chooser.getSelected();
		
		switch (selected) {
			case chooseTalonAbs:
				motor.configSelectedFeedbackSensor(encoderAbsolute, 0, feedbackDelay);
				break;
			case chooseTalonRel:
				motor.configSelectedFeedbackSensor(encoderRelative, 0, feedbackDelay);
				break;
		}
		
		configureControl();
	}

	@Override
	public void autonomousPeriodic() {
		switch (selected) {
			case chooseNavx:
				turnTo(90);
				break;
			case chooseTalonAbs:
				driveDist(5);
				break;
			case chooseTalonRel:
				driveDist(5);
				break;
		}
	}
	
	private void updateValues() {
		switch (selected) {
			case chooseNavx:
				pN = SmartDashboard.getNumber("NavX-P", pN);
				iN = SmartDashboard.getNumber("NavX-I", iN);
				dN = SmartDashboard.getNumber("NavX-D", dN);
				break;
			case chooseTalonAbs:
				pT = SmartDashboard.getNumber("Talon-P", pT);
				iT = SmartDashboard.getNumber("Talon-I", iT);
				dT = SmartDashboard.getNumber("Talon-D", dT);
				break;
		}
	}
	
	private void configureControl() {
		updateValues();
		pid.setP(pN); pid.setI(iN); pid.setD(dN); //Sets NavX PID values
		motor.config_kP(0, pT, feedbackDelay); motor.config_kI(0, iT, feedbackDelay); motor.config_kD(0, dT, feedbackDelay); //Sets Talon PID values
	}
	
	private void turnTo(double desAngle) {
		//TODO: Determine if deadzone needs to be set
		float curAngle = navx.getCompassHeading();
		
		pid.setSetpoint(desAngle);
		speed = pid.getOutput(curAngle);
		
		if (Math.abs(curAngle-desAngle) >= 180) { //Checks if robot needs to turn to the left
			drive(speed);
//			drive(-speed, speed); //Use this function for testing on actual robot
		}
		else {
			drive(-speed);
//			drive(speed, -speed); //Use this function for testing on actual robot
		}
	}
	
	private void drive(double value) {
		value *= 1; //Sets soft limit for speed
		
		motor.set(ControlMode.PercentOutput, value);
	}
	
	private void drive(double left, double right) {
		left *= 1; right *= 1; //Sets soft limit for speed
		
		frontLeft.set(ControlMode.PercentOutput, left);
		frontRight.set(ControlMode.PercentOutput, right);
		backLeft.set(ControlMode.PercentOutput, left);
		backRight.set(ControlMode.PercentOutput, right);
	}
	
	private void driveDist(double dist) {
		int ticks = calcTicks(dist);
		motor.set(ControlMode.Position, ticks);
		
		//Use this for testing on actual robot
//		frontLeft.set(ControlMode.Position, ticks);
//		frontRight.set(ControlMode.PercentOutput, frontLeft.getClosedLoopError(0));
//		backLeft.set(ControlMode.PercentOutput, frontLeft.getClosedLoopError(0));
//		backRight.set(ControlMode.PercentOutput, frontLeft.getClosedLoopError(0));
	}
	
	private int calcDPR(int ppr) {				
		return (int) (4*Math.PI / ppr); //Change magic number to wheel diameter (IN INCHES)
	}
	
	private int calcTicks(double dist) {
		int dpr = 0;
		
		switch (selected) {
			case (chooseTalonAbs):
				dpr = encoderDRP_ABS;
				break;
			case (chooseTalonRel):
				dpr = encoderDRP_REL;
				break;
		}
		
		return (int) (dist/dpr);
	}
}
