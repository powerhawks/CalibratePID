package org.usfirst.frc.team1111.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends IterativeRobot {
	//Default values. These do not change when you re-deploy so when calibrating PID, use SmartDashboard, do not edit these variables.
	double p = 0.0175;
	double i = 0.0000001;
	double d = 0.1125;
	AHRS navx = new AHRS(SPI.Port.kMXP);
	MiniPID pid = new MiniPID(p, i, d);
	double speed;
	double desAngle = 90;
	
	TalonSRX frontLeft = new TalonSRX(43);
	TalonSRX frontRight = new TalonSRX(48);
	TalonSRX backLeft = new TalonSRX(7);
	TalonSRX backRight = new TalonSRX(54);
	
	@Override
	public void robotInit() {
		SmartDashboard.putNumber("NavX-P", p); SmartDashboard.putNumber("NavX-I", i); SmartDashboard.putNumber("NavX-D", d);
		SmartDashboard.putNumber("Desired Angle:", desAngle);
		pid.setOutputLimits(-1, 1);		
	}

	@Override
	public void autonomousInit() {
		desAngle = SmartDashboard.getNumber("Desired Angle:", desAngle);
		navx.reset();
		configureControl();
	}

	@Override
	public void autonomousPeriodic() {
		turnTo(desAngle);
	}
	
	
	//=====BEGIN UTILITY CODE=====
	
	
	private void updateValues() {
		p = SmartDashboard.getNumber("NavX-P", p);
		i = SmartDashboard.getNumber("NavX-I", i);
		d = SmartDashboard.getNumber("NavX-D", d);
	}
	
	private void configureControl() {
		updateValues();
		pid.setP(p); pid.setI(i); pid.setD(d); //Sets NavX PID values
	}
	
	
	//=====BEGIN NAVX CODE=====
	
	
	private void turnTo(double desAngle) {
		double curAngle = navx.getAngle();
		SmartDashboard.putNumber("Current Angle:", curAngle); //Debug
		
		pid.setSetpoint(desAngle);
		speed = pid.getOutput(curAngle);
		SmartDashboard.putNumber("Speed:", speed);
		
		drive(speed, -speed);
	}
	
	
	//=====BEGIN TALON SRX CODE======
	
	
	private void drive(double speed) {
		speed *= 1; //Sets soft limit for speed
		
		frontLeft.set(ControlMode.PercentOutput, speed);
		frontRight.set(ControlMode.PercentOutput, -speed);
		backLeft.set(ControlMode.PercentOutput, speed);
		backRight.set(ControlMode.PercentOutput, -speed);
	}
	
	private void drive(double left, double right) {
		left *= 1; right *= -1; //Sets soft limit for speed
		
		frontLeft.set(ControlMode.PercentOutput, left);
		frontRight.set(ControlMode.PercentOutput, right);
		backLeft.set(ControlMode.PercentOutput, left);
		backRight.set(ControlMode.PercentOutput, right);
	}
}
