package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax azimuthMotor;
    private final CANCoder absoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder azimuthEncoder;
    private final double offset;
    private final boolean encoderTop;
    private final SparkMaxPIDController drivePIDController;
    private final SparkMaxPIDController azimuthPIDController;
    private double referenceAngleRadians;
    
    public SwerveModule(int driveID, int azimuthID, int absoluteID, double inOffset, boolean isEncoderTop) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(azimuthID, MotorType.kBrushless);
        absoluteEncoder = new CANCoder(absoluteID);
        driveEncoder = driveMotor.getEncoder();
        azimuthEncoder = azimuthMotor.getEncoder();
        driveMotor.restoreFactoryDefaults();
        azimuthMotor.restoreFactoryDefaults();
        offset = inOffset;
        encoderTop = isEncoderTop;
        drivePIDController = driveMotor.getPIDController();
        azimuthPIDController = azimuthMotor.getPIDController();
        driveMotor.setSmartCurrentLimit(60);
        azimuthMotor.setSmartCurrentLimit(20);
        driveMotor.setInverted(true);
        azimuthMotor.setInverted(true);
        driveMotor.enableVoltageCompensation(12.6);
        azimuthMotor.enableVoltageCompensation(12.6);
        driveEncoder.setVelocityConversionFactor(1/(ModuleConstants.mk4iL1DriveGearRatio)/60 * .0762 * Math.PI);
        driveEncoder.setPositionConversionFactor(1/(ModuleConstants.mk4iL1DriveGearRatio) * .0762 * Math.PI);
        azimuthEncoder.setVelocityConversionFactor(1/(ModuleConstants.mk4iL1TurnGearRatio)*2*Math.PI/60);
        azimuthEncoder.setPositionConversionFactor(1/(ModuleConstants.mk4iL1TurnGearRatio)*2*Math.PI);
        drivePIDController.setP(0.0001);
        drivePIDController.setFF(1/ DriveConstants.kMaxSpeedMetersPerSecond);
        azimuthPIDController.setP(3.0);
        azimuthEncoder.setPosition(getAbsoluteEncoder());
        driveMotor.burnFlash();
        azimuthMotor.burnFlash();

        referenceAngleRadians = 0.0;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getStateAngle()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getStateAngle()));
    }

    public double getAbsoluteEncoder() {
        double pose = (absoluteEncoder.getAbsolutePosition())*Math.PI/180.0 + (encoderTop ?  offset*-1.0 : offset) ;
        if (encoderTop) {
            return -pose;
        }
        else {
            return pose;
        }
    }

    public double getStateAngle() {
        double motorAngleRadians = azimuthEncoder.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0*Math.PI;
        }
        return motorAngleRadians;
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = azimuthEncoder.getPosition();
  
        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }
  
        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }
  
        this.referenceAngleRadians = referenceAngleRadians;
  
        //SmartDashboard.putNumber("ADJRot "+moduleID, adjustedReferenceAngleRadians);
  
        azimuthPIDController.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
    }
    /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));
   
    drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

      setReferenceAngle(state.angle.getRadians());
 
  }
}