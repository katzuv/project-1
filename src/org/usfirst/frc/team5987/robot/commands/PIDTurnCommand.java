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
public abstract class PIDTurnCommand extends Command {
	protected double DELAY = 0.005;
	protected double startingAngle;
	protected MiniPID pid;
	Timer timer = new Timer();
    public PIDTurnCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	startingAngle = updateAngle();
		timer.reset();
		
		//Getting the constants from smartdashboard.
		pid = new MiniPID(getKP(),getKI(),getKD());
		pid.setDirection(true);
		updatePID();
		
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	updateAngle();
    	updateSetpoint();
    	updatePID();
    	pid.setSetpoint(updateSetpoint());
    	setMotors(getOutput());
    	Timer.delay(DELAY); 
    	printValues();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return checkFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    protected double getOutput(){
    	return pid.getOutput(updateAngle()-startingAngle, updateSetpoint());
    }
    /**
     * updates the MiniPID class.
     */
	protected void updatePID(){
		pid.setP(getKP());
		pid.setI(getKI());
		pid.setD(getKD());
	}
	/**
	 * Override this to return the constant P.
	 */
    protected abstract double getKP();
	/**
	 * Override this to return the constant I.
	 */
    protected abstract double getKI();
	/**
	 * Override this to return the constant D.
	 */
    protected abstract double getKD();
	/**
	 * Override this to return the current angle.
	 */
    protected abstract double updateAngle();
	/**
	 * Override this to return the goal.
	 */
    protected abstract double updateSetpoint();
	/**
	 * Override this to return the condition in which the program stops.
	 */
    protected abstract boolean checkFinished();
	/**
	 * Override this and apply the strengths of the motors using (double output).
	 */
    protected abstract void setMotors(double output);
    /**
     * Place in this class the values you want to print.
     */
    protected abstract void printValues();
    public double getError(){
    	return updateSetpoint() - updateAngle()-startingAngle;
    }
}
