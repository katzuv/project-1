package org.usfirst.frc.team5987.robot.subsystems;

import org.usfirst.frc.team5987.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PneumaticsSubsystem extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.solenoidForward, RobotMap.solenoidReverse);
	Compressor compressor = new Compressor(RobotMap.compressor);
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	/**
	   * Set the value of a solenoid to forward.
	   *
	   */
	public void forward()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
	}
	/**
	   * Set the value of a solenoid to off.
	   *
	   */
	public void off()
	{
		solenoid.set(DoubleSolenoid.Value.kOff);
	}
	/**
	   * Set the value of a solenoid to reverse.
	   *
	   */
	public void reverse()
	{
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}
	public void startCompressor()
	{
		compressor.start();
	}
	public void stopCompressor()
	{
		compressor.stop();
	}
	/**
	   * Read the current value of the solenoid.
	   *
	   * @return The current value of the solenoid.
	   */
	public Value getValue()
	{
		Value value = solenoid.get();
		boolean forward = false;
		boolean reverse = false;
		switch (value) {
	      case kOff:
	        forward = false;
	        reverse = false;
	        break;
	      case kForward:
	        forward = true;
	        reverse = false;
	        break;
	      case kReverse:
	        forward = false;
	        reverse = true;
	        break;
		}
		SmartDashboard.putBoolean("forward", forward);
		SmartDashboard.putBoolean("reverse", reverse);
	    return value;
	}
}
