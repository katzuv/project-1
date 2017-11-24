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
	Timer timer = new Timer();
	double lastTime;
	
	// these values you get from the raspberry pi.
	double prevCameraTargetAngle = 0;
	double cameraTargetAngle = 0;
	double cameraTargetDistance = 0;

	// These are determined at the start of the command
	double ConstantP;
	double ConstantI;
	double ConstantD;
	
	double DELAY = 0.005;

	// the angle from the center of the robot to the target
	double robotTargetAngle;
	
	//error: the angle the robot needs to turn
	double error, prevError;
	//pid changing values
	double P, I, D;
	double Si = 0;

	public TurnToTargetCommand() {
		driveSubsystem = Robot.driveSubsystem;
		requires(driveSubsystem);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.resetNavX();
		timer.reset();
		
		//Getting the constants from smartdashboard.
		ConstantP = SmartDashboard.getNumber("Pid: p - ", RobotMap.ConstantP);
		ConstantI = SmartDashboard.getNumber("Pid: i - ", RobotMap.ConstantI);
		ConstantD = SmartDashboard.getNumber("Pid: d - ", RobotMap.ConstantD);
		
		//getting the CAMERA VALUES from the raspberry pi
		prevCameraTargetAngle = SmartDashboard.getNumber("Target Angle: ", 30);
		cameraTargetAngle = SmartDashboard.getNumber("Target Angle: ", 30);
		cameraTargetDistance = SmartDashboard.getNumber("Target Distance: ", 50);
		
		//Angle from the robot base
		robotTargetAngle = cameraToCenterAngle(cameraTargetAngle, cameraTargetDistance);
		
		prevError = robotTargetAngle - Robot.driveSubsystem.getAngle();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		//Check if the camera Values changed, and fix the target.
		cameraTargetAngle = SmartDashboard.getNumber("Target Angle: ", 30);
		if(cameraTargetAngle != prevCameraTargetAngle){
			robotTargetAngle = cameraToCenterAngle(cameraTargetAngle, cameraTargetDistance)
					+ Robot.driveSubsystem.getAngle();
		}
		double timeDiff = timer.get() - lastTime;
		lastTime = timer.get();
		
		error = robotTargetAngle - Robot.driveSubsystem.getAngle();
		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("Angle", Robot.driveSubsystem.getAngle());
		P = ConstantP * error;
		Si += timeDiff * (prevError + error) / 2;
		I = ConstantI * Si;
		D = ConstantD * (error - prevError) / timeDiff;

		SmartDashboard.putNumber("Val motors", (P + I + D));
		Robot.driveSubsystem.drive(P + I + D, -(P + I + D));
		prevError = error;
		Timer.delay(DELAY);
		prevCameraTargetAngle = cameraTargetAngle;
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
	
	/**
	 * Turns the camera angle into the angle of the center of the robot.
	 * @param camAngle Camera angle in degrees
	 * @param camDistance Camera distance in degrees
	 * @return Robot center angle
	 */
	private double cameraToCenterAngle(double camAngle, double camDistance){
		camAngle = Math.toRadians(camAngle);
		return Math.toDegrees(
				Math.atan(
				(camDistance * Math.sin(camAngle))
				/ (camDistance * Math.cos(camAngle) + RobotMap.distanceFromCenter))
			);
	}
}

//Find more accurate delay
//Document
