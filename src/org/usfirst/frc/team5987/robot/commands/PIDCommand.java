package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;
import org.usfirst.frc.team5987.robot.subsystems.DrivingSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDCommand extends Command {
	DrivingSubsystem driveSubsystem;
	double ConstantP;
	double ConstantI;
	double ConstantD;

	double P, I, D;
	double angle, error;
	public PIDCommand() {
		driveSubsystem = Robot.driveSubsystem;
		requires(driveSubsystem);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		ConstantP = SmartDashboard.getNumber("Pid: p - ", RobotMap.ConstantP);
		ConstantI = SmartDashboard.getNumber("Pid: i - ", RobotMap.ConstantI);
		ConstantD = SmartDashboard.getNumber("Pid: d - ", RobotMap.ConstantD);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
