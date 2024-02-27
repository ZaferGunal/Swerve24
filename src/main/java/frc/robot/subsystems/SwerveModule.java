package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ConversionConstants;
import  com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SpeedConstants;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax angleMotor;
 RelativeEncoder driveEncoder;
     RelativeEncoder angleEncoder;

     PIDController anglePIDController;

       CANcoder cancoder;
     final double cancoderOffSet;
      final boolean cancoderReversed;


public SwerveModule(CANSparkMax driveMotorId_, CANSparkMax angleMotorId_, boolean driveMotorReversed_, boolean angleMotorReversed_,
CANcoder cancoderId_, double cancoderOffSet_, boolean cancoderReversed_)

{
    driveMotor= driveMotorId_;
    angleMotor =angleMotorId_;
     driveMotor.setInverted(driveMotorReversed_);
     angleMotor.setInverted(angleMotorReversed_);
     

     driveEncoder = driveMotor.getEncoder();          
     angleEncoder =angleMotor.getEncoder();
     
     cancoder = cancoderId_;
      cancoderOffSet = cancoderOffSet_;
    cancoderReversed =cancoderReversed_;

    anglePIDController = new PIDController(ConversionConstants.kPAngle,0,0);

 
    driveEncoder.setPositionConversionFactor(ConversionConstants.kDriveEncoderRotation2meter);
    driveEncoder.setVelocityConversionFactor(ConversionConstants.kDriveEncoderRPMperSecond);

    angleEncoder.setPositionConversionFactor(ConversionConstants.kAngleEncoderRotation2rad);
    angleEncoder.setVelocityConversionFactor(ConversionConstants.kAngleEncoderRPMperSecond);

    anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();



}

    public double getDrivePosition(){
        return driveEncoder.getPosition();

    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    
    public double getAnglePosition(){
        return angleEncoder.getPosition();

    }

    public double getAngleVelocity(){
        return angleEncoder.getVelocity();
    }

    public double getCancoderRad(){
        double angle  =cancoder.getPosition().getValue() * 2  * Math.PI;
        angle -= cancoderOffSet;
        return angle *(cancoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        angleEncoder.setPosition(getCancoderRad());
        driveEncoder.setPosition(0.0);
    }


    public SwerveModuleState getStateOfModule(){
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getAnglePosition()));
    }


    public void stopMotors(){
        angleMotor.set(0.0);
        driveMotor.set(0.0);
    }


    public void setDesiredState(SwerveModuleState moduleState){
         if(moduleState.speedMetersPerSecond < 0.001){
            stopMotors();
            return;
         }
        else{
        moduleState.optimize(moduleState,getStateOfModule().angle);
        driveMotor.set(moduleState.speedMetersPerSecond/SpeedConstants.maxSpeedPerSecond);
        angleMotor.set(anglePIDController.calculate(getAnglePosition(),moduleState.angle.getRadians()));
        }
                
    }


    


}











    











