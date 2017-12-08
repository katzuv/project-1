package org.usfirst.frc.team5987.robot.commands;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;

import auxiliary.MiniPID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GenericTestCommand extends PIDTurnCommand {
	double startingCamAngle, startingCamDistance;
    public GenericTestCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
        
    }
    @Override
    protected void initialize() {
    	startingCamAngle = SmartDashboard.getNumber("targetAngle", 90);
    	startingCamDistance = SmartDashboard.getNumber("targetDistance", 50);
    	super.initialize();
    	
    }
    
    @Override
    protected void execute() {
    	// TODO Auto-generated method stub
    	super.execute();
    	SmartDashboard.putNumber("error", getError());
    	SmartDashboard.putNumber("errorRotation", getError());
    }
	@Override
	protected double getKP() {
		return SmartDashboard.getNumber("kpRotation", RobotMap.ConstantP);
	}

	@Override
	protected double getKI() {
		// TODO Auto-generated method stub
		return SmartDashboard.getNumber("kiRotation", RobotMap.ConstantI);
	}

	@Override
	protected double getKD() {
		// TODO Auto-generated method stub
		return SmartDashboard.getNumber("kdRotation", RobotMap.ConstantD);
	}

	@Override
	protected double updateAngle() {
		// TODO Auto-generated method stub
		return Robot.driveSubsystem.getAngle();
	}

	@Override
	protected double updateSetpoint() {
		// TODO Auto-generated method stub
		return cameraToCenterAngle(startingCamAngle, startingCamDistance);
	}

	@Override
	protected boolean checkFinished() {
//		return Math.abs(getError())<0.9;
		return false;
	}

	@Override
	protected void setMotors(double output) {
		Robot.driveSubsystem.drive(output, -output);
		
	}
	
	private double cameraToCenterAngle(double camAngle, double camDistance){
		camAngle = Math.toRadians(camAngle);
		return Math.toDegrees(
				Math.atan(
				(camDistance * Math.sin(camAngle))
				/ (camDistance * Math.cos(camAngle) + RobotMap.distanceFromCenter))
			);
	}
}