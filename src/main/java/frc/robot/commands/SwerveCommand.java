package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSystemBase;
import java.util.function.Supplier;
import frc.robot.Constants.JoystickConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.PhysicalProperties;
import edu.wpi.first.math.kinematics.SwerveModuleState;
public class SwerveCommand extends Command{
    SlewRateLimiter yLimiter,xLimiter,turningLimiter;
     final  SwerveSystemBase swerveSystemBase ;
    Supplier<Double> xSpeedFunc,ySpeedFunc,turningSpeedFunc;
    
   public  SwerveCommand( SwerveSystemBase swerveSystem_ ,Supplier<Double> xSpeedF_, Supplier<Double> ySpeedF_, Supplier<Double> turningSpeedF_){
        swerveSystemBase= swerveSystem_;
        xSpeedFunc = xSpeedF_;
        ySpeedFunc = ySpeedF_;
        turningSpeedFunc = turningSpeedF_;
        xLimiter = new SlewRateLimiter(SpeedConstants.accelerationTeleopXY);
        yLimiter = new SlewRateLimiter(SpeedConstants.accelerationTeleopXY);
        turningLimiter = new SlewRateLimiter(SpeedConstants.accelerationAngular);
        addRequirements(swerveSystemBase);
    }


  @Override
  public void initialize() {}

 // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double turningSpeed = turningSpeedFunc.get();


    xSpeed = Math.abs(xSpeed) > JoystickConstants.kDeadBand ? xSpeed : 0.0;
     ySpeed = Math.abs(ySpeed) > JoystickConstants.kDeadBand ? ySpeed : 0.0;
     turningSpeed = Math.abs(turningSpeed) > JoystickConstants.kDeadBand ? turningSpeed : 0.0;

     /*xSpeed = xLimiter.calculate(xSpeed);
     ySpeed = yLimiter.calculate(ySpeed);
     turningSpeed = turningLimiter.calculate(turningSpeed);        for smooth drive 
*/
       
    xSpeed *= SpeedConstants.maxConvenientSpeed;
    ySpeed *= SpeedConstants.maxConvenientSpeed;
    turningSpeed *= SpeedConstants.maxConvenientAngularSpeed;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed,ySpeed,turningSpeed,swerveSystemBase.getHeadingRotation2d());

    SwerveModuleState[] swerveModuleStates_i = PhysicalProperties.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

     swerveSystemBase.setDesiredStates(swerveModuleStates_i);
  }


  @Override
  public void end(boolean interrupted) {
    swerveSystemBase.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }



}
