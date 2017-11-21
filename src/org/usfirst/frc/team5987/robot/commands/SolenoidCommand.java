package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SolenoidCommand extends Command {

	public boolean isForward;
	
	SmartDashboard dashboard = new SmartDashboard();
    public SolenoidCommand(boolean isForward) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.pneumaticsSubsystem);
    	this.isForward = isForward;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (isForward)
    	{
    		Robot.pneumaticsSubsystem.reverse();
    		return;
    	}
    	Robot.pneumaticsSubsystem.forward();
    			    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
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
