package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.RobotMap;

import auxiliary.MiniPID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PIDTurnCommand extends Command {
	protected double constantP,constantI,constantD;
	protected MiniPID pid;
    public PIDTurnCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	pid = new MiniPID(constantP,constantI,constantD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	updateAngle();
    	updateSetpoint();
    	constantP = updateP();
    	constantI = updateI();
    	constantD = updateD();
    	
    	updatePID();
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
    protected double updateP(){
    	throw new Error("Must Extend updateP() method, and return a double!");
    }
    protected double updateI(){
    	throw new Error("Must Extend updateI() method, and return a double!");
    }
    protected double updateD(){
    	throw new Error("Must Extend updateD() method, and return a double!");
    }
    protected double updateAngle(){
    	throw new Error("Must Extend updateAngle() method, and return a double!");
    }
    protected double updateSetpoint(){
    	throw new Error("Must Extend updateSetpoint() method, and return a double!");
    }
    protected double updateMaxError(){
    	throw new Error("Must Extend updateMaxError() method, and return a double!");
    }
    protected double updateMinError(){
    	throw new Error("Must Extend updateMinError() method, and return a double!");
    }
    public double getP(){
    	return constantP;
    }
    public double getI(){
    	return constantI;
    }
    public double getD(){
    	return constantD;
    }
    /**
     * updates the MiniPID class.
     */
	public void updatePID(){
		pid.setP(constantP);
		pid.setI(constantI);
		pid.setD(constantD);
	}
	
}
