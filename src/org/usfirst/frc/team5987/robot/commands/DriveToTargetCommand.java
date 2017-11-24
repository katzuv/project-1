package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;

import auxiliary.MiniPID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveToTargetCommand extends Command {

	public DriveToTargetCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
	}

	MiniPID pid = new MiniPID(RobotMap.driveKp, RobotMap.driveKi, RobotMap.driveKd);
	Timer time = new Timer();
	double encoderDelta = 0;
	boolean reachedTarget = false;
	double initLeftEncoder, initRightEncoder, initDistanceFromTarget;

	/**
	 * Initializes the encoders values and the distances from the target.
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
		 * The distance the encoders have driven.
		 */
		encoderDelta = ((driveSubsystem.getLeftEncoder() - initLeftEncoder)
				+ (driveSubsystem.getRightEncoder() - initRightEncoder)) / 2;
		/**
		 * The output from the PID controller.
		 */
		double output = pid.getOutput(initDistanceFromTarget - encoderDelta, 1);

		if (initDistanceFromTarget != SmartDashboard.getNumber("distance", 1)) {
			init();
		}
		driveSubsystem.drive(output, output);
		time.delay(0.05);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (Math.abs(SmartDashboard.getNumber("distance", 1) - 1) < 0.2) {
			if (!reachedTarget) {
				time.reset();
				time.start();
			}
			reachedTarget = true;

			if (time.get() > 2) {
				return true;
			}
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
