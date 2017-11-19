package org.usfirst.frc.team5987.robot.subsystems;

import org.usfirst.frc.team5987.robot.Robot;
import org.usfirst.frc.team5987.robot.RobotMap;
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
	AHRS ahrs = Robot.ahrs;
	static RobotDrive robotDrive;
	
	private static Encoder leftEncoder;
	private static Encoder rightEncoder;

    public void initDefaultCommand() {
    	
    	// set ports for the victors using the preassigned values of the RobotMap
    	robotDrive = new RobotDrive(RobotMap.leftFrontMotor,RobotMap.leftRearMotor,RobotMap.rightFrontMotor,RobotMap.rightRearMotor);
    	
    	leftEncoder = new Encoder(RobotMap.leftDriveChanelA, RobotMap.leftDriveChanelB);
    	rightEncoder = new Encoder(RobotMap.rightDriveChanelA, RobotMap.rightDriveChanelB);
    	
    	leftEncoder.setDistancePerPulse(RobotMap.distancePerPulse);
    	rightEncoder.setDistancePerPulse(RobotMap.distancePerPulse);
    }
    /**
     * Normal TankDrive
     * @param leftValue leftMotor speed -1 <= speed <= 1
     * @param rightValue rightMotor speed -1 <= speed <= 1
     */
    public void drive(double leftValue, double rightValue){
    	robotDrive.tankDrive(leftValue, rightValue);
    	SmartDashboard.putNumber("leftEncoder", -leftEncoder.getDistance());
    	SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());
    }
    public double getLeftEncoder() {	
    	SmartDashboard.putNumber("leftEncoder", -leftEncoder.getDistance());
    	return -leftEncoder.getDistance();
    }
    
    public double getRightEncoder() {
    	SmartDashboard.putNumber("rightEncoder", rightEncoder.getDistance());
    	return rightEncoder.getDistance();
    }
    
    public double getAngle() {
		return ahrs.getAngle();

	}
	public double limit(double minLimit, double maxLimit, double val) {
		if (val < minLimit)
			return minLimit;
		if (val > maxLimit)
			return maxLimit;
		return val;
	}

	public void resetNavX() {
		ahrs.reset();
	}
}

