package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;
import org.usfirst.frc.team5987.robot.subsystems.DrivingSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnToTargetCommand extends Command {
	DrivingSubsystem driveSubsystem;
	// these values you get from the raspberry pi.
	double InitialTargetAngle = 0;
	double InitialTargetDistance = 0;

	// These are determined at the start of the command
	double ConstantP;
	double ConstantI;
	double ConstantD;
	double delay = 0.005;

	// the angle from the center of the robot to the target
	double robotTargetAngle;
	double error, prevError;
	double P, I, D;
	double Si = 0;

	public TurnToTargetCommand() {
		driveSubsystem = Robot.driveSubsystem;
		requires(driveSubsystem);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.resetNavX();
		ConstantP = SmartDashboard.getNumber("Pid: p - ", RobotMap.ConstantP);
		ConstantI = SmartDashboard.getNumber("Pid: i - ", RobotMap.ConstantI);
		ConstantD = SmartDashboard.getNumber("Pid: d - ", RobotMap.ConstantD);
		InitialTargetAngle = SmartDashboard.getNumber("Target Angle: ", 90);
		InitialTargetDistance = SmartDashboard.getNumber("Target Distance: ", 50);
		robotTargetAngle = Math.toDegrees(Math.atan((InitialTargetDistance * Math.sin(InitialTargetAngle))
				- (InitialTargetDistance * Math.cos(InitialTargetAngle) + RobotMap.distanceFromCenter)));

		prevError = InitialTargetAngle - Robot.driveSubsystem.getAngle();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		error = InitialTargetAngle - Robot.driveSubsystem.getAngle();
		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("Angle", Robot.driveSubsystem.getAngle());
		P = ConstantP * error;
		Si += delay * (prevError + error) / 2;
		I = ConstantI * Si;
		D = ConstantD * (error - prevError) / delay;

		SmartDashboard.putNumber("Val motors", (P + I + D));
		Robot.driveSubsystem.drive(P + I + D, -(P + I + D));
		prevError = error;
		Timer.delay(delay);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(error) < 1.4;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
