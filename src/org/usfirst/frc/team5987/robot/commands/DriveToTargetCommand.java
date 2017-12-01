package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;
import org.usfirst.frc.team5987.robot.subsystems.DrivingSubsystem;

import auxiliary.MiniPID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Dan Katzuv
 *
 */
public class DriveToTargetCommand extends Command {
	DrivingSubsystem driveSubsystem;

	public DriveToTargetCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
		driveSubsystem = Robot.driveSubsystem;

	}

	double kP = SmartDashboard.getNumber("driveKp", 0);
	double kI = SmartDashboard.getNumber("driveKi", 0);
	double kD = SmartDashboard.getNumber("driveKd", 0);

	MiniPID leftPid = new MiniPID(kP, kI, kD);
	MiniPID rightPid = new MiniPID(kP, kI, kD);
	Timer time = new Timer();
	Timer speed = new Timer();
	double leftEncoderDelta = 0;
	double rightEncoderDelta = 0;
	boolean reachedTarget = false;
	double initLeftEncoder, initRightEncoder, initDistanceFromTarget;

	/**
	 * Initializes the encoders values and the distances from the target. Gets
	 * the initial encoder values from the driveSubsystem and gets the distance
	 * from the SmartDashboard in meters.
	 */
	private void init() {
		initLeftEncoder = driveSubsystem.getLeftEncoder();
		initRightEncoder = driveSubsystem.getRightEncoder();
		initDistanceFromTarget = SmartDashboard.getNumber("distance", 1);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		init();

		time.reset();
		time.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/**
		 * The distance the left motor has driven.
		 */
		leftEncoderDelta = driveSubsystem.getLeftEncoder() - initLeftEncoder;
		/**
		 * The distance the right motor has driven.
		 */
		rightEncoderDelta = driveSubsystem.getLeftEncoder() - initRightEncoder;
		/**
		 * The output from the PID controllers for the left motor.
		 */
		double leftOutput = leftPid.getOutput(leftEncoderDelta, 1);
		/**
		 * The output from the PID controllers for the right motor.
		 */
		double rightOutput = leftPid.getOutput(rightEncoderDelta, 1);

		if (initDistanceFromTarget != SmartDashboard.getNumber("distance", 1)) {
			init();
		}
		driveSubsystem.drive(leftOutput, rightOutput);
		time.delay(0.05);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if ((initDistanceFromTarget - leftEncoderDelta - 1) < 0.2 && (initDistanceFromTarget - rightEncoderDelta - 1) < 0.2) {
			return true;
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		driveSubsystem.drive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}