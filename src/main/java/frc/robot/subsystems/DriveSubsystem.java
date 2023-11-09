// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.utils.ShuffleboardContent;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics m_kinematics = DriveConstants.m_kinematics;

  // public final SwerveModuleSparkMax m_frontLeft = new SwerveModuleSparkMax(
  //     IDConstants.FRONT_LEFT_LOCATION,
  //     CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
  //     CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
  //     CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
  //     DriveConstants.kFrontLeftDriveMotorReversed,
  //     DriveConstants.kFrontLeftTurningMotorReversed,
  //     DriveConstants.kFrontLeftCancoderDirection,
  //     PDPConstants.FRONT_LEFT_DRIVE_CHANNEL,
  //     PDPConstants.FRONT_LEFT_TURN_CHANNEL,

  //     CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

  // public final SwerveModuleSparkMax m_frontRight = new SwerveModuleSparkMax(
  //     IDConstants.FRONT_RIGHT_LOCATION,
  //     CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
  //     CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
  //     CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
  //     DriveConstants.kFrontRightDriveMotorReversed,
  //     DriveConstants.kFrontRightTurningMotorReversed,
  //     DriveConstants.kFrontRightCancoderDirection,
  //     PDPConstants.FRONT_RIGHT_DRIVE_CHANNEL,
  //     PDPConstants.FRONT_RIGHT_TURN_CHANNEL,
  //     CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

  // public final SwerveModuleSparkMax m_backLeft = new SwerveModuleSparkMax(
  //     IDConstants.REAR_LEFT_LOCATION,
  //     CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
  //     CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
  //     CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
  //     DriveConstants.kBackLeftDriveMotorReversed,
  //     DriveConstants.kBackLeftTurningMotorReversed,
  //     DriveConstants.kBackLeftCancoderDirection,
  //     PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
  //     PDPConstants.BACK_LEFT_TURN_CHANNEL,
  //     CanConstants.BACK_LEFT_MODULE_STEER_OFFSET);

  // public final SwerveModuleSparkMax m_backRight = new SwerveModuleSparkMax(
  //     IDConstants.REAR_RIGHT_LOCATION,
  //     CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
  //     CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
  //     CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
  //     DriveConstants.kBackRightDriveMotorReversed,
  //     DriveConstants.kBackRightTurningMotorReversed,
  //     DriveConstants.kBackRightCancoderDirection,
  //     PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
  //     PDPConstants.BACK_LEFT_TURN_CHANNEL,
  //     CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

    public final SwerveModule m_frontLeft = new SwerveModule(
    CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
    CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
    CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
    CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET, false);

    public final SwerveModule m_frontRight = new SwerveModule(
      CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
      CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, false);

    public final SwerveModule m_backLeft = new SwerveModule(
      CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
      CanConstants.BACK_LEFT_MODULE_STEER_OFFSET, true);

    public final SwerveModule m_backRight = new SwerveModule(
      CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
      CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET, false);

  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  private PIDController m_xController = new PIDController(DriveConstants.kP_X, 0, DriveConstants.kD_X);
  private PIDController m_yController = new PIDController(DriveConstants.kP_Y, 0, DriveConstants.kD_Y);
  private ProfiledPIDController m_turnController = new ProfiledPIDController(
      DriveConstants.kP_Theta, 0,
      DriveConstants.kD_Theta,
      Constants.TrapezoidConstants.kThetaControllerConstraints);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private boolean showOnShuffleboard = true;

  private SimDouble m_simAngle;// navx sim

  public double throttleValue;

  public double targetAngle;

  public boolean m_fieldOriented = false;

  public boolean useVisionOdometry = true;

  private double startTime;

  private double positionStart;

  double positionChange;

  public boolean isOpenLoop = true;

  // private SwerveModuleDisplay m_smd = new SwerveModuleDisplay(this);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // SmartDashboard.putData("SM", m_smd);

    m_gyro.reset();

    //resetModuleEncoders();

    setIdleMode(true);

    m_fieldOriented = false;

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    if (showOnShuffleboard)

      ShuffleboardContent.initMisc(this);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  /*
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * 
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * 
   * @param rot Angular rate of the robot.
   * 
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        this.m_fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    updateOdometry();
    SmartDashboard.putNumber("Xpos", getX());
    SmartDashboard.putNumber("Ypos", getY());
    SmartDashboard.putNumber("Angle", getHeadingDegrees());


    if (startTime == 0) {

      startTime = Timer.getFPGATimestamp();

      positionStart = getEstimatedPose().getX();

    }

    if (Timer.getFPGATimestamp() > startTime + 5) {

      positionChange = getEstimatedPose().getX() - positionStart;

      startTime = 0;

      positionStart = getEstimatedPose().getX();

    }
    SmartDashboard.putNumber("fl encoder angle", m_frontLeft.getAbsoluteEncoder());
    SmartDashboard.putNumber("fr encoder angle", m_frontRight.getAbsoluteEncoder());
    SmartDashboard.putNumber("rl encoder angle", m_backLeft.getAbsoluteEncoder());
    SmartDashboard.putNumber("rr encoder angle", m_backRight.getAbsoluteEncoder());
  }

  public void updateOdometry() {

    SmartDashboard.putString(("EstPosn"), getEstimatedPose().toString());
    SmartDashboard.putString(("FLModPos"), m_frontLeft.getPosition().toString());
    //SmartDashboard.putNumber(("FLDrPos"), m_frontLeft.getDrivePosition());

    if (true) {

      /** Updates the field relative position of the robot. */

      m_poseEstimator.update(
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });

      // Also apply vision measurements. We use 0.3 seconds in the past as an example
      // -- on
      // a real robot, this must be calculated based either on latency or timestamps.
      // m_poseEstimator.addVisionMeasurement(
      // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
      // m_poseEstimator.getEstimatedPosition()),
      // Timer.getFPGATimestamp() - 0.3);

    }
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeadingRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition() },
        pose);
    m_gyro.reset();

  }

  public void setAngleAdjustment(double adjustment) {
    m_gyro.setAngleAdjustment(adjustment);
  }

  public double getAngleAdjustment() {
    return m_gyro.getAngleAdjustment();
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  // public boolean checkCANOK() {
  //   return RobotBase.isSimulation() ||
  //       m_frontLeft.checkCAN()
  //           && m_frontRight.checkCAN()
  //           && m_backLeft.checkCAN()
  //           && m_backLeft.checkCAN();

  // }

  // public void resetModuleEncoders() {
  //   m_frontLeft.resetAngleToAbsolute();
  //   m_frontRight.resetAngleToAbsolute();
  //   m_backLeft.resetAngleToAbsolute();
  //   m_backRight.resetAngleToAbsolute();
  // }

  /** Zeroes the heading of the robot. */
  public void resetGyro() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);

  }

  public Translation2d getTranslation() {
    return getEstimatedPose().getTranslation();
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public ProfiledPIDController getThetaPidController() {
    return m_turnController;
  }

  public double getX() {
    return getTranslation().getX();
  }

  public double getY() {
    return getTranslation().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void setIdleMode(boolean brake) {

    // m_frontLeft.setDriveBrakeMode(brake);
    // m_frontLeft.setTurnBrakeMode(brake);
    // m_frontRight.setDriveBrakeMode(brake);
    // m_frontRight.setTurnBrakeMode(brake);
    // m_backLeft.setDriveBrakeMode(brake);
    // m_backLeft.setTurnBrakeMode(brake);
    // m_backRight.setDriveBrakeMode(brake);
    // m_backRight.setTurnBrakeMode(brake);

  }

  @Override
  public void simulationPeriodic() {

    ChassisSpeeds chassisSpeedSim = m_kinematics.toChassisSpeeds(

        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;
    SmartDashboard.putNumber("CHSSM", chassisSpeedSim.omegaRadiansPerSecond);
    temp += m_simAngle.get();

    m_simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void jogTurnModule(SwerveModuleSparkMax i, double speed) {
    i.turnMotorMove(speed);
  }

  public void positionTurnModule(SwerveModuleSparkMax i, double angle) {
    i.positionTurn(angle);
  }

  public void driveModule(SwerveModuleSparkMax i, double speed) {
    i.driveMotorMove(speed);
  }

  public boolean getTurnInPosition(SwerveModuleSparkMax i, double targetAngle) {
    return i.turnInPosition(targetAngle);
  }

  public double getAnglefromThrottle() {

    return 180 * throttleValue;
  }

}
