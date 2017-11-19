package org.usfirst.frc.team5987.robot.subsystems;

import org.usfirst.frc.team5987.robot.RobotMap;
import org.usfirst.frc.team5987.robot.commands.OpenSolenoidCommand;

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
	double distance = SmartDashboard.getNumber("distance", 110);
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new OpenSolenoidCommand(distance));
	}
	
	/**
	   * Set the value of a solenoid to forward.
	   *
	   */
	public void forward()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
	}
	public void off()
	{
		solenoid.set(DoubleSolenoid.Value.kOff);
	}
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
		return solenoid.get();
	}
}
