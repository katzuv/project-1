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

	double leftkP, leftKi, leftKd;
	double rightkP, rightKi, rightKd;
	MiniPID leftPid, rightPid;
	double leftEncoderDelta, rightEncoderDelta;
	double initLeftEncoder, initRightEncoder;
	double initDistanceFromTarget;
	double leftError, rightError;
	double leftOutput, rightOutput;

	// Called just before this Command runs the first time
	protected void initialize() {
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
		leftOutput = leftPid.getOutput(leftEncoderDelta, initDistanceFromTarget);
		/**
		 * The output from the PID controllers for the right motor.
		 */
		rightOutput = rightPid.getOutput(rightEncoderDelta, initDistanceFromTarget);

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
		return leftError < 0.1 && rightError < 0.1 && leftOutput < 0.4 && rightOutput < 0.4;
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