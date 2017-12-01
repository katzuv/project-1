package org.usfirst.frc.team5987.robot.subsystems;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;
import org.usfirst.frc.team5987.robot.commands.JoystickDriveCommand;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivingSubsystem extends Subsystem {
	static RobotDrive robotDrive;
	
	private static Encoder leftEncoder;
	private static Encoder rightEncoder;
	
	public DrivingSubsystem(){
    	// set ports for the victors using the preassigned values of the RobotMap
    	robotDrive = new RobotDrive(RobotMap.leftFrontMotor,RobotMap.leftRearMotor,RobotMap.rightFrontMotor,RobotMap.rightRearMotor);
    	
    	leftEncoder = new Encoder(RobotMap.leftDriveChanelA, RobotMap.leftDriveChanelB);
    	rightEncoder = new Encoder(RobotMap.rightDriveChanelA, RobotMap.rightDriveChanelB);
    	
    	leftEncoder.setDistancePerPulse(RobotMap.distancePerPulse);
    	rightEncoder.setDistancePerPulse(RobotMap.distancePerPulse);
	}
    public void initDefaultCommand() {

    	setDefaultCommand(new JoystickDriveCommand());
    }
    /**
     * Normal TankDrive
     * @param leftValue leftMotor speed -1 <= speed <= 1
     * @param rightValue rightMotor speed -1 <= speed <= 1
     */
    public void drive(double leftValue, double rightValue){
    	leftValue = limit(-1,1,leftValue);
    	rightValue = limit(-1,1,rightValue);
    	robotDrive.tankDrive(-leftValue, -rightValue);
    	getLeftEncoder();
    	getRightEncoder();
    }
    public double getLeftEncoder() {
    	double distance = -leftEncoder.getDistance();
    	SmartDashboard.putNumber("leftEncoder", distance);
    	return distance;
    }
    
    public double getRightEncoder() {
    	double distance = rightEncoder.getDistance();
    	SmartDashboard.putNumber("rightEncoder", distance);
    	return distance;
    }
    
    public double getAngle() {
		return Robot.ahrs.getAngle();

	}
	public double limit(double minLimit, double maxLimit, double val) {
		if (val < minLimit)
			return minLimit;
		if (val > maxLimit)
			return maxLimit;
		return val;
	}

	public void resetNavX() {
		Robot.ahrs.reset();
	}
}

