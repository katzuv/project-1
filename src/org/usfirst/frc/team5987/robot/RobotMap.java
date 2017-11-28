package org.usfirst.frc.team5987.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
	public final static double distancePerPulse = 0.00133;
	
    public static int leftFrontMotor = 1;
    public static int leftRearMotor = 0;
    public static int rightFrontMotor = 3;
    public static int rightRearMotor = 2;
    
	public static int leftDriveChanelA = 0;
	public static int leftDriveChanelB = 1;
	
	public static int rightDriveChanelA =2;
	public static int rightDriveChanelB = 3;

	public static double ConstantP = 0.1;
	public static double ConstantI = 0;
	public static double ConstantD = 0;

    public static int solenoidForward = 5;
    public static int solenoidReverse = 4;
    public static int compressor = 6;
    
    public static double distanceFromCenter = 30;
    public static double driveKp = 0;
    public static double driveKi = 0;
    public static double driveKd = 0;
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}
