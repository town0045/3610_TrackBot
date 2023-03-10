//This package is necessary to tell the computer this code belones to our robot.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN 
    public static final int drive_spark_0 = 0; //Drivetrain Right
    public static final int drive_spark_1 = 1; //Drivetrain Left
    public static final int drive_spark_2 = 2; //Drivetrain Right
    public static final int drive_spark_3 = 3; //Drivetrain Left
    public static final int intake_neo = 5;     //Indexer motor
    public static final int shooter_falcon0 = 6;  //Shooter Inverted
    public static final int shooter_falcon1 = 7;  //Shooter
    public static final int climber_falcon = 8;   //Climber motor
    public static final int intake_snowblower = 9; //Controlls intake angle
    public static final int intake_775 = 10;       //Spins the intake
    public static final int shooter_snowblower = 11; //Was to control hood angle, never used.
    public static final int climber_snowblower = 12; //Controls climber arm position.

    //Input
    public static int shooter_lowerswitchchannel = 5;  //limit switch input not used.
    public static int shooter_upperswitchchannel = 2;  //Limit switch input, not used.
    public static int intakeArmEncoderChannel = 0;     //Intake encoder input
    public static int climber_encoder = 1;             //Climber encoder, not used.

    public static int drive_PDP = 1;           //Power Distribution Panal address
    public static double gearRatio = 10.71;    //Gear ratio, needed for characterization of drivetrain
    public static double wheelRadius = 3;      //Wheel size, needed for characterization of drivetrain.

    //PID
    public static  double SpinKp = 0.17;                      //Intake angle Kp value
    public static  double SpinKi = 0.0; 
    public static  double SpinKd = 0.0075;
   
    public static  double SpinKp_Turn = 0.017;                      //Intake angle Kp value
    public static  double SpinKi_Turn = -0.1; 
    public static  double SpinKd_Turn = 0.0;

    public static double kTurnToleranceDeg = 0.05;  
    public static double kTurnToleranceDegPerS = 0.0005;  
    public static double kTurnToleranceDeg_Turn = 2;  
    public static double kTurnToleranceDegPerS_Turn = 0.05;  
    //Auto Constants
    public static double AutoDriveSpeed = 0.7;



    public static double IntakePID_kI = 1;                      //Intake angle Ki value - not used
    public static double IntakePID_kD = 1;                      //Intake angle Kd value - not used
    public static double IntakePID_kS = 1;                      //Intake angle Ks value - not used
    public static double IntakePID_kV = 1;                      //Intake angle Kv value - not used
    public static double IntakePID_UpSetpoint = 0.554;
    public static double IntakePID_IntakeSetpoint = 0.13;
    public static double IntakePID_UptakeSetpoint = 0.165;
    //
    //public static double targetUpperShooterSpeed = 0.3; //CHANGE THIS
    public static double Auto_Shooter_Speed = 0.5;

    //Values for characterization of drivetrain
    public static double ksVolts = 0.3;
    public static double ksVoltsSecondsPerMeter = 5.0;
    public static double kVoltsSecondsSquaredPerMeter = 1.0;

    public static final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Units.inchesToMeters(26)); //Wheel base taken from CAD drawing.
    //Sets the maximium velocities and accelerations
    public static double maxVelocityMetersPerSecond = 3;
    public static double maxAccelerationMetersPerSecondSq = 3;

    public static double ramsete_b = 2;
    public static double ramsete_z = 0.7;
    public static double kPDrive = 5; // Needs to be validated   
    
    public static double Shooter_Kp = 1;
    public static double Shooter_Ki = 0;

    //Lime light related constants - useful for range finding.
    public static double limelightMountAngle = 45; // in degrees
    public static double limelightHeight = 30; // in inches
    public static double GoalHeight = 104; // in inches

    public static double ShooterSpeedSlope = 50; //Need to determine by trial and error
    public static double ShooterSpeedIntercept = 1000; //Need to determine by trial and error
}
