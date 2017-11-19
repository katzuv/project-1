package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class OpenSolenoidCommand extends Command {

	SmartDashboard dashboard = new SmartDashboard();
    public OpenSolenoidCommand(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.pneumaticsSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pneumaticsSubsystem.startCompressor();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    			    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return SmartDashboard.getNumber("distance", 110) < 100;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.pneumaticsSubsystem.forward();
    	Robot.pneumaticsSubsystem.stopCompressor();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
