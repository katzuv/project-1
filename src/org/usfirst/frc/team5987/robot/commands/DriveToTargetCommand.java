package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.subsystems.DrivingSubsystem;

import auxiliary.MiniPID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Dan Katzuv
 *
 */
public class DriveToTargetCommand extends Command {
	DrivingSubsystem driveSubsystem;
	MiniPID leftPid;
	MiniPID rightPid;
	double kP;
	double kI;
	double kD;
	double rOutP;
	double lOutP;
	double rOutI = 0;
	double lOutI = 0;
	double rOutD;
	double lOutD;
	double prevRightError;
	double prevLeftError;
	double DELAY = 0.05;
	/**
	 * The distance the left motor has driven.
	 */
	double leftEncoderDelta = 0;
	/**
	 * The distance the right motor has driven.
	 */
	double rightEncoderDelta = 0;
	double initLeftEncoder, initRightEncoder, initDistanceFromTarget, rightError, leftError;
	double distance;
	
	public DriveToTargetCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
		driveSubsystem = Robot.driveSubsystem;
	}

	/**
	 * Initializes the encoders values and the distances from the target. Gets
	 * the initial encoder values from the driveSubsystem and gets the distance
	 * from the SmartDashboard in meters.
	 */
	private void init() {
		initDistanceFromTarget = SmartDashboard.getNumber("distance", 3);
		SmartDashboard.putNumber("distance", initDistanceFromTarget);
		initLeftEncoder = driveSubsystem.getLeftEncoder();
		initRightEncoder = driveSubsystem.getRightEncoder();
		SmartDashboard.putString("State", "START");
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		init();
		kP = SmartDashboard.getNumber("driveKp", 0.5);
		kI = SmartDashboard.getNumber("driveKi", 0);
		kD = SmartDashboard.getNumber("driveKd", 0);
		SmartDashboard.putNumber("driveKp", kP);
		SmartDashboard.putNumber("driveKi", kI);
		SmartDashboard.putNumber("driveKd", kD);
//		leftPid = new MiniPID(0.5, 0, 0);
//		rightPid = new MiniPID(0.5, 0, 0);
		updateErrors();
		prevLeftError = leftError;
		prevRightError = rightError;
	}
	public void updateErrors(){
		leftEncoderDelta = driveSubsystem.getLeftEncoder() - initLeftEncoder;
		rightEncoderDelta = driveSubsystem.getLeftEncoder() - initRightEncoder;
		leftError = initDistanceFromTarget - leftEncoderDelta;
		rightError = initDistanceFromTarget - rightEncoderDelta;
	}
	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		updateErrors();
		SmartDashboard.putNumber("leftError", leftError);
		lOutP = leftError * kP;
		lOutI = lOutI + (leftError * DELAY) * kI;
		lOutD = kD * (leftError - prevLeftError) / DELAY;
		double leftOutput = lOutP + lOutI + lOutD;
		SmartDashboard.putNumber("leftOutput", leftOutput);
		leftOutput = -leftOutput;
		
		SmartDashboard.putNumber("rightError", rightError);
		rOutP = rightError * kP;
		rOutI = rOutI + (rightError * DELAY) * kI;
		rOutD = kD * (rightError - prevRightError) / DELAY;
		double rightOutput = rOutP + rOutI + rOutD;
		SmartDashboard.putNumber("rightOutput", rightOutput);
		rightOutput = -rightOutput;



		driveSubsystem.drive(leftOutput, rightOutput);
		



		Timer.delay(DELAY);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(leftError) < 0.05 && Math.abs(rightError) < 0.05;
	}

	// Called once after isFinished returns true
	protected void end() {
		driveSubsystem.drive(0, 0);
		SmartDashboard.putString("State", "END");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}