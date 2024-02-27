package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.FrontLeftModuleConstants;
import frc.robot.Constants.FrontRightModuleConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.RearLeftModuleConstants;
import frc.robot.Constants.RearRightModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ConversionConstants;
import  com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SpeedConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
public class SwerveSystemBase extends SubsystemBase {

  private Pigeon2 gyro= new Pigeon2(GyroConstants.gyroDeviceId);


  public SwerveSystemBase(){
  new Thread(()->{
    try  {
    Thread.sleep(1000);
    
    } catch (Exception e) {
      
      System.out.println("??))((()))"+ e.getMessage());
    }
  }).start();

};
    
// frontLeftModule 
 static CANcoder frontLeftCancoder = new CANcoder(FrontLeftModuleConstants.cancoderId);
  static CANSparkMax frontLeftDriveMotor = new CANSparkMax(FrontLeftModuleConstants.driveMotorId,MotorType.kBrushless);
 static  CANSparkMax frontLeftAngleMotor = new CANSparkMax(FrontLeftModuleConstants.angleMotorId,MotorType.kBrushless);


 //frontRightModule
 static CANcoder frontRightCancoder = new CANcoder(FrontRightModuleConstants.cancoderId);
  static CANSparkMax frontRightDriveMotor = new CANSparkMax(FrontRightModuleConstants.driveMotorId,MotorType.kBrushless);
 static  CANSparkMax frontRightAngleMotor = new CANSparkMax(FrontRightModuleConstants.angleMotorId,MotorType.kBrushless);


  //rearLeftModule
static CANcoder rearLeftCancoder = new CANcoder(RearLeftModuleConstants.cancoderId);
static CANSparkMax rearLeftDriveMotor = new CANSparkMax(RearLeftModuleConstants.driveMotorId,MotorType.kBrushless);
static CANSparkMax rearLeftAngleMotor = new CANSparkMax(RearLeftModuleConstants.angleMotorId,MotorType.kBrushless);

  //rearLeftModule
static CANcoder rearRightCancoder = new CANcoder(RearRightModuleConstants.cancoderId);
static CANSparkMax rearRightDriveMotor = new CANSparkMax(RearRightModuleConstants.driveMotorId,MotorType.kBrushless);
static CANSparkMax rearRightAngleMotor = new CANSparkMax(RearRightModuleConstants.angleMotorId,MotorType.kBrushless);


////////////////////////////////////////////////// constructs modules

  final SwerveModule frontLeftModule = new SwerveModule(frontLeftDriveMotor,
frontLeftAngleMotor,FrontLeftModuleConstants.isDriveMotorReversed,FrontLeftModuleConstants.isAngleMotorReversed,
frontLeftCancoder,FrontLeftModuleConstants.cancoderOffSetRad,FrontLeftModuleConstants.isCancoderReversed
);

private final SwerveModule frontRightModule = new SwerveModule(
  frontRightDriveMotor,frontRightAngleMotor,
  FrontRightModuleConstants.isDriveMotorReversed,FrontRightModuleConstants.isAngleMotorReversed,
  frontRightCancoder,FrontRightModuleConstants.cancoderOffSetRad,FrontRightModuleConstants.isCancoderReversed);

private final SwerveModule rearLeftModule = new SwerveModule(rearLeftDriveMotor,rearLeftAngleMotor,
RearLeftModuleConstants.isDriveMotorReversed, RearLeftModuleConstants.isAngleMotorReversed, 
rearLeftCancoder,RearLeftModuleConstants.cancoderOffSetRad,RearLeftModuleConstants.isCancoderReversed


);


private final SwerveModule rearRightModule = new SwerveModule(rearRightDriveMotor,rearRightAngleMotor,
RearRightModuleConstants.isDriveMotorReversed,RearRightModuleConstants.isAngleMotorReversed,
rearRightCancoder,RearRightModuleConstants.cancoderOffSetRad,RearRightModuleConstants.isCancoderReversed

);

public void resetGyro(){
 gyro.reset();


}
@Override
public void periodic() {
double loggings[]={
frontLeftModule.getAnglePosition(),frontLeftModule.getDriveVelocity(),
frontRightModule.getAnglePosition(),frontRightModule.getDriveVelocity(),
rearLeftModule.getAnglePosition(),rearLeftModule.getDriveVelocity(),
rearRightModule.getAnglePosition(),rearRightModule.getDriveVelocity()
};

  SmartDashboard.putNumberArray("SwerveModuleStates",loggings);
  SmartDashboard.putNumber("robot Heading",getHeadingDouble());
  
}

@Override
public void simulationPeriodic() {

  
    
}
public void stopMotors(){
  frontLeftModule.stopMotors();
  frontRightModule.stopMotors();
  rearLeftModule.stopMotors();
  rearRightModule.stopMotors();
}


public void setDesiredStates(SwerveModuleState[] swerveModuleStates_){
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates_,SpeedConstants.maxIndividSpeedPerSec);
  frontLeftModule.setDesiredState(swerveModuleStates_[0]);
  frontRightModule.setDesiredState(swerveModuleStates_[1]);
  rearLeftModule.setDesiredState(swerveModuleStates_[2]);
  rearRightModule.setDesiredState(swerveModuleStates_[3]);

}



public double getHeadingDouble(){
 return  gyro.getAngle();

}

public Rotation2d getHeadingRotation2d(){
    return gyro.getRotation2d();

}


    }

