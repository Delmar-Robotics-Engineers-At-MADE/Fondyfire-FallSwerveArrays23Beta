// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;
import frc.robot.utils.ShuffleboardContent;

public class SwerveModuleSparkMax extends SubsystemBase {
  public final int m_locationIndex;
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  public final RelativeEncoder m_driveEncoder;

  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_driveVelSMController;

  private final PIDController m_driveVelController = new PIDController(.001, 0, 0);

  private SparkMaxPIDController m_turnSMController = null;

  private PIDController m_turnController = null;

  public final CTRECanCoder m_turnCANcoder;

  SwerveModuleState state;

  public int m_moduleNumber;

  public String[] modAbrev = { "FL ", "FR ", "RL ", "RR " };
  String driveLayout;

  String turnLayout;

  String canCoderLayout;

  Pose2d m_pose;

  double testAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      ModuleConstants.ksVolts,
      ModuleConstants.kvVoltSecondsPerMeter,
      ModuleConstants.kaVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  public double angle;

  public double m_turningEncoderOffset;

  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;
  private final int SIM_SLOT = 2;

  private int tuneOn;
  public double actualAngleDegrees;

  private double angleDifference;
  private double angleIncrementPer20ms;
  private double tolDegPerSec = .05;
  private double toleranceDeg = .25;
  public boolean driveMotorConnected;
  public boolean turnMotorConnected;
  public boolean turnCoderConnected;
  private boolean useRRPid = true;
  private double turnDeadband = .5;
  private boolean showOnShuffleboard = true;
  private boolean driveBrakeMode;
  private boolean turnBrakeMode;
  public SendableBuilder m_builder;

  private int testCall;
  private double simDriveEncoderPosition;
  private double simDriveVelocity;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset
   */
  public SwerveModuleSparkMax(
      int locationIndex,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      boolean cancoderDirection,
      int pdpCDrivehannel,
      int pdpTurnChannel,
      double turningEncoderOffset) {

    m_locationIndex = locationIndex;

    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);

    m_turningMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);

    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.restoreFactoryDefaults();

    m_turningMotor.setSmartCurrentLimit(20);

    m_driveMotor.setSmartCurrentLimit(20);

    m_driveMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);

    m_turningMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);

    // absolute encoder used to establish known wheel position on start position
    m_turnCANcoder = new CTRECanCoder(cancoderCanChannel);
    m_turnCANcoder.configFactoryDefault();
    CANCoderConfiguration cancoderSettings = AngleUtils.generateCanCoderConfig();
    cancoderSettings.sensorDirection = cancoderDirection;
    m_turnCANcoder.configAllSettings(cancoderSettings);

    m_turningEncoderOffset = turningEncoderOffset;

    m_driveMotor.setInverted(driveMotorReversed);

    m_turningMotor.setInverted(turningMotorReversed);

    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
    // Set neutral mode to brake
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    SmartDashboard.putNumber("DriveMETPERENCREv", ModuleConstants.kDriveMetersPerEncRev);

    // m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);

    // m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerEncRev
    // / 60);

    m_driveVelSMController = m_driveMotor.getPIDController();

    if (RobotBase.isReal()) {

      m_driveVelSMController.setP(.03, VEL_SLOT);

      m_driveVelSMController.setD(0, VEL_SLOT);

      m_driveVelSMController.setI(0, VEL_SLOT);

      m_driveVelSMController.setIZone(1, VEL_SLOT);

    }

    else {

      m_driveVelSMController.setP(2, SIM_SLOT);

    }

    m_turningEncoder = m_turningMotor.getEncoder();

    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);

    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

    if (!useRRPid)

      m_turnSMController = m_turningMotor.getPIDController();

    else

      m_turnController = new PIDController(ModuleConstants.kPModuleTurnController, 0, 0);

    if (useRRPid)

      tunePosGains();

    else

      tuneSMPosGains();

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));

    }

    checkCAN();

    if (RobotBase.isReal())

      resetAngleToAbsolute();

    if (showOnShuffleboard) {

      ShuffleboardContent.initDriveShuffleboard(this);
      ShuffleboardContent.initTurnShuffleboard(this);
      ShuffleboardContent.initCANCoderShuffleboard(this);
      ShuffleboardContent.initBooleanShuffleboard(this);
      ShuffleboardContent.initCoderBooleanShuffleboard(this);
    }

  }

  @Override
  public void periodic() {

    if (Pref.getPref("SwerveTune") == 1 && tuneOn == 0) {

      tuneOn = 1;

      if (useRRPid)

        tunePosGains();

      else

        tuneSMPosGains();

    }

    if (tuneOn == 1) {

      tuneOn = (int) Pref.getPref("SwerveTune");
    }

    if (m_turnCANcoder.getFaulted()) {
      // SmartDashboard.putStringArray("CanCoderFault"
      // + m_modulePosition.toString(), m_turnCANcoder.getFaults());
      SmartDashboard.putStringArray("CanCoderStickyFault"
          + String.valueOf(m_locationIndex), m_turnCANcoder.getStickyFaults());

      if (DriverStation.isDisabled()) {
        if (turnBrakeMode)
          setTurnBrakeMode(false);

        if (driveBrakeMode)
          setDriveBrakeMode(false);
      }

    }
  }

  public void tunePosGains() {
    m_turnController.setP(0.01);
    m_turnController.setI(Pref.getPref("SwerveTurnPoskI"));
    m_turnController.setD(0.00001);
    // m_turnController.setIZone(Pref.getPref("SwerveTurnPoskIz"));
  }

  public void tuneSMPosGains() {
    m_turnSMController.setP(Pref.getPref("SwerveTurnSMPoskP"), POS_SLOT);
    m_turnSMController.setI(Pref.getPref("SwerveTurnSMPoskI"), POS_SLOT);
    m_turnSMController.setD(Pref.getPref("SwerveTurnSMPoskD"), POS_SLOT);
    m_turnSMController.setIZone(Pref.getPref("SwerveTurnSMPoskIz"), POS_SLOT);
  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(simDriveVelocity, Rotation2d.fromDegrees((angle)));
  }

  public SwerveModulePosition getPosition() {

    double tempPos = getDrivePosition();
    Rotation2d tempAngle = getHeadingRotation2d();
    return new SwerveModulePosition(tempPos, tempAngle);

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

    SmartDashboard.putNumber("StateSpeed", state.speedMetersPerSecond);
    SmartDashboard.putNumber("StateAngle", state.angle.getDegrees());

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

        ? m_lastAngle

        : state.angle.getDegrees();

    m_lastAngle = angle;

    if (RobotBase.isReal()) {

      // turn axis

      actualAngleDegrees = m_turningEncoder.getPosition();

      if (useRRPid) {

        positionTurn(angle);
      }

      else {

        positionSMTurn(angle);
      }

      // drive axis

      if (isOpenLoop)

        m_driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);

      else {

        m_driveVelSMController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, VEL_SLOT);

      }
    }

    if (RobotBase.isSimulation()) {

      // rev position sim not working - do own position sim
      simDriveEncoderPosition();

      // no simulation for angle - angle command is returned directly to drive
      // subsystem as actual angle in 2 places - getState() and getHeading

      simTurnPosition(angle);
    }
  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    simDriveEncoderPosition = 0;

  }

  public double getHeadingDegrees() {

    if (RobotBase.isReal())

      return m_turningEncoder.getPosition();

    else

      return actualAngleDegrees;

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public void setDriveBrakeMode(boolean on) {
    driveBrakeMode = on;
    if (on)
      m_driveMotor.setIdleMode(IdleMode.kBrake);
    else
      m_driveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean on) {
    turnBrakeMode = on;
    if (on) {
      m_turningMotor.setIdleMode(IdleMode.kBrake);

    } else
      m_turningMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetAngleToAbsolute() {
    double angle = m_turnCANcoder.getAbsolutePosition() - m_turningEncoderOffset;
    m_turningEncoder.setPosition(angle);
  }

  public double getTurnAngle() {
    return m_turningEncoder.getPosition();
  }

  public void turnMotorMove(double speed) {
    m_turningMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void positionSMTurn(double angle) {

    m_turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT);
  }

  public void positionTurn(double angle) {

    double turnAngleError = Math.abs(angle - m_turningEncoder.getPosition());

    double pidOut = m_turnController.calculate(m_turningEncoder.getPosition(), angle);
    // if robot is not moving, stop the turn motor oscillating
    if (turnAngleError < turnDeadband

        && Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

      pidOut = 0;

    m_turningMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());

  }

  private void simTurnPosition(double angle) {

    actualAngleDegrees = angle;

    // if (angle != actualAngleDegrees && angleIncrementPer20ms == 0) {

    // angleDifference = angle - actualAngleDegrees;

    // angleIncrementPer20ms = angleDifference / 5;// 5*20ms = .1 sec move time
    // }

    // if (angleIncrementPer20ms != 0) {

    // actualAngleDegrees += angleIncrementPer20ms;

    // if (actualAngleDegrees > 180)

    // actualAngleDegrees -= 180;

    // if (actualAngleDegrees < -180)

    // actualAngleDegrees += 180;

    // if ((Math.abs(angle - actualAngleDegrees)) < .2) {

    // actualAngleDegrees = angle;

    // angleIncrementPer20ms = 0;
    // }
    // }
  }

  private double simDriveEncoderPosition() {

    double vel = state.speedMetersPerSecond;
    simDriveVelocity = vel;
    double distancePer20Ms = vel / 50;

    simDriveEncoderPosition += distancePer20Ms;
    SmartDashboard.putNumber("SIMDPOS", simDriveEncoderPosition);
    return simDriveEncoderPosition;
  }

  public void driveMotorMove(double speed) {
    m_driveMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public double getDriveVelocity() {
    if (RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return simDriveVelocity;

  }

  public double getDrivePosition() {
    SmartDashboard.putNumber("TCPP", testCall++);
    SmartDashboard.putNumber("GEGPCF", m_driveEncoder.getPositionConversionFactor());
    SmartDashboard.putNumber("GEGPREV", m_driveEncoder.getCountsPerRevolution());

    if (RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
      return simDriveEncoderPosition;/// ModuleConstants.kDriveMetersPerEncRev;
  }

  public double getDriveCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  public double getTurnVelocity() {
    return m_turningEncoder.getVelocity();
  }

  public double getTurnPosition() {
    return m_turningEncoder.getPosition();
  }

  public double getTurnCurrent() {
    return m_turningMotor.getOutputCurrent();
  }

  public boolean turnInPosition(double targetAngle) {
    return Math.abs(targetAngle - getTurnAngle()) < toleranceDeg;
  }

  public boolean turnIsStopped() {

    return Math.abs(m_turningEncoder.getVelocity()) < tolDegPerSec;
  }

  public boolean checkCAN() {

    driveMotorConnected = m_driveMotor.getFirmwareVersion() != 0;
    turnMotorConnected = m_turningMotor.getFirmwareVersion() != 0;
    turnCoderConnected = m_turnCANcoder.getFirmwareVersion() > 0 || RobotBase.isSimulation();

    return driveMotorConnected && turnMotorConnected && turnCoderConnected;

  }

}