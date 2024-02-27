package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ConversionConstants{
    public final static double kWheelDiameter = Units.inchesToMeters(1.5);           // wheel diameter//      ayarlanacak
    public final static double kDriveMotorGearRatio = 1/6.75 ;                 //DriveMotor GearRatio//              ayarlanacak
    public final static double kAngleMotorGearRatio = 1/12.8;                  //AngleMotor GearRatio//              ayarlanacak
    public final static double kDriveEncoderRotation2meter =kDriveMotorGearRatio * Math.PI * kWheelDiameter;
    public final static double kAngleEncoderRotation2rad = kAngleMotorGearRatio * Math.PI * 2;
    public final static double kDriveEncoderRPMperSecond=  kDriveEncoderRotation2meter / 60;
    public final static double kAngleEncoderRPMperSecond = kAngleEncoderRotation2rad /60;
    public final static double kPAngle = 0.5;

  }


    
    
  


  public static class FrontLeftModuleConstants{
  public final static int driveMotorId = 1;
  public final static int angleMotorId = 2;
  public final static boolean  isDriveMotorReversed = false;
  public final static boolean isAngleMotorReversed = false;
  public final static int cancoderId = 3;     
  public final static double cancoderOffSetRad =0;     // radian
  public final static  boolean isCancoderReversed = false;

  }
  
  public static class FrontRightModuleConstants{
  public final static int driveMotorId = 4;
  public final static int angleMotorId = 5;
  public final static boolean  isDriveMotorReversed = false;
  public final static boolean isAngleMotorReversed = false;
  public final static int cancoderId = 6;     
  public final static double cancoderOffSetRad =0;     // radian
  public final static  boolean isCancoderReversed = false;

  }

  public static class RearLeftModuleConstants{
  public final static int driveMotorId = 7;
  public final static int angleMotorId = 8;
  public final static boolean  isDriveMotorReversed = false;
  public final static boolean isAngleMotorReversed = false;
  public final static int cancoderId = 9;     
  public final static double cancoderOffSetRad =0;     // radian
  public final static  boolean isCancoderReversed = false;

  }


  public static class RearRightModuleConstants{
  public final static int driveMotorId = 10;
  public final static int angleMotorId = 11; 
  public final static boolean  isDriveMotorReversed = false;
  public final static boolean isAngleMotorReversed = false;
  public final static int cancoderId = 12;     
  public final static double cancoderOffSetRad =0;     // radian
  public final static  boolean isCancoderReversed = false;


  }

  public static class GyroConstants{
    public final static int gyroDeviceId = 13;
  }


  public static class SpeedConstants{
    public final static double maxSpeedPerSecond = 5.0;
    public final static double maxAngularSpeedPerSecond = 5.0;
    public final static double maxIndividSpeedPerSec = 5.0;
    public final static double accelerationTeleopXY = 3;
    public final static double accelerationAngular = 3;
    public final static double maxConvenientSpeed = maxSpeedPerSecond / 4;
    public final static double maxConvenientAngularSpeed = maxAngularSpeedPerSecond / 4;

  }


  public static class JoystickConstants{
    public final static double kDeadBand = 0.01;

  }

  public static class PhysicalProperties{
    public static final double kTrackWidth = 0.54;                // doldur
        // Distance between right and left wheels
        public static final double kWheelBase = 0.54;                     ///              doldur  şaseden değil tekerleklerden ölç 
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

  }

  }


