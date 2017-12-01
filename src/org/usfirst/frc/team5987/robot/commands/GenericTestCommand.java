package org.usfirst.frc.team5987.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GenericTestCommand extends PIDTurnCommand {

    public GenericTestCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
    protected double updateP(){
    	return SmartDashboard.getNumber("P whatever", 0);
    }
    protected double updateI(){
    	return SmartDashboard.getNumber("P whatever", 0);
    }
    protected double updateD(){
    	return SmartDashboard.getNumber("P whatever", 0);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
//    protected void execute() {
//    }

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
