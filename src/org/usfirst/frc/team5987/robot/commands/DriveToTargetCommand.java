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

	double leftkP;
	double leftKi;
	double leftKd;
	double rightkP;
	double rightKi;
	double rightKd;
	MiniPID leftPid;
	MiniPID rightPid;

	double leftEncoderDelta;
	double rightEncoderDelta;
	double initLeftEncoder, initRightEncoder, initDistanceFromTarget, leftError, rightError;

	/**
	 * Initializes the encoders values and the distances from the target. Gets
	 * the initial encoder values from the driveSubsystem and gets the distance
	 * from the SmartDashboard in meters.
	 */
	private void init() {
		initLeftEncoder = driveSubsystem.getLeftEncoder();
		initRightEncoder = driveSubsystem.getRightEncoder();
		initDistanceFromTarget = SmartDashboard.getNumber("driveInitDistance", 3);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// init();
		SmartDashboard.putString("State", "START");
		initLeftEncoder = driveSubsystem.getLeftEncoder();
		initRightEncoder = driveSubsystem.getRightEncoder();
		initDistanceFromTarget = SmartDashboard.getNumber("driveInitDistance", 3);
		leftkP = SmartDashboard.getNumber("leftDriveKp", RobotMap.leftDriveKp);
		leftKi = SmartDashboard.getNumber("leftDriveKi", RobotMap.leftDriveKi);
		leftKd = SmartDashboard.getNumber("leftDriveKd", RobotMap.leftDriveKd);
		rightkP = SmartDashboard.getNumber("rightDriveKp", RobotMap.rightDriveKp);
		rightKi = SmartDashboard.getNumber("rightDriveKi", RobotMap.rightDriveKi);
		rightKd = SmartDashboard.getNumber("rightDriveKd", RobotMap.rightDriveKd);
		leftPid = new MiniPID(leftkP, leftKi, leftKd);
		rightPid = new MiniPID(rightkP, rightKi, rightKd);
		leftPid.setDirection(true);
		rightPid.setDirection(true);
		SmartDashboard.putNumber("leftEncoder", initLeftEncoder);
		SmartDashboard.putNumber("rightEncoder", initRightEncoder);
		SmartDashboard.putNumber("driveInitDistance", initDistanceFromTarget);
		SmartDashboard.putNumber("leftDriveKp", leftkP);
		SmartDashboard.putNumber("leftDriveKi", leftKi);
		SmartDashboard.putNumber("leftDriveKd", leftKd);
		SmartDashboard.putNumber("rightDriveKp", rightkP);
		SmartDashboard.putNumber("rightDriveKi", rightKi);
		SmartDashboard.putNumber("rightDriveKd", rightKd);
		SmartDashboard.putNumber("driveLeftError", leftError);
		SmartDashboard.putNumber("driveRightError", rightError);

		SmartDashboard.putString("State", "START");
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
		rightEncoderDelta = driveSubsystem.getRightEncoder() - initRightEncoder;
		/**
		 * The output from the PID controllers for the left motor.
		 */
		double leftOutput = leftPid.getOutput(leftEncoderDelta, initDistanceFromTarget);
		/**
		 * The output from the PID controllers for the right motor.
		 */
		double rightOutput = rightPid.getOutput(rightEncoderDelta, initDistanceFromTarget);

		leftError = initDistanceFromTarget - leftEncoderDelta;
		rightError = initDistanceFromTarget - rightEncoderDelta;
		
		driveSubsystem.drive(-leftOutput, -rightOutput);
		SmartDashboard.putNumber("driveLeftOutput", leftOutput);
		SmartDashboard.putNumber("driveRightOutput", rightOutput);
		SmartDashboard.putNumber("driveLeftError", leftError);
		SmartDashboard.putNumber("driveRightError", rightError);
		
		SmartDashboard.putNumber("driveLeftIOutput", leftPid.getI());
		SmartDashboard.putNumber("driveRightIOutput", rightPid.getI());
		Timer.delay(0.05);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return leftError < 0.05 && rightError < 0.05;
		return false;
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