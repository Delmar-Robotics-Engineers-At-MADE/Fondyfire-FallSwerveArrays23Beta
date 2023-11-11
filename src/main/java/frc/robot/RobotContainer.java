// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PusherOpenLoop;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopPusher;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.Vision.SetDriverMode;
import frc.robot.commands.Vision.StrafeToGamePiece;
// import frc.robot.commands.swerve.JogDriveModule;
// import frc.robot.commands.swerve.JogTurnModule;
import frc.robot.commands.swerve.SetSwerveDrive;
//import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Optical;
import frc.robot.subsystems.Pusher;
import frc.robot.subsystems.VisionPoseEstimator;
import frc.robot.utils.AutoSelect;
import frc.robot.utils.FFDisplay;
import frc.robot.utils.LEDControllerI2C;

public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_drive = new DriveSubsystem();

  public VisionPoseEstimator m_vpe = new VisionPoseEstimator(m_drive);

  public FFDisplay ff1 = new FFDisplay("test");

  public AutoSelect m_autoSelect;

  public Intake intake = new Intake();

  public Pusher pusher = new Pusher();

  final Limelight limelight = new Limelight();
  final Optical optical = new Optical();

  LEDControllerI2C lcI2;

  //public final FieldSim m_fieldSim = new FieldSim(m_drive);

   // The driver's controller

  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  final PowerDistribution m_pdp = new PowerDistribution();

  final GamepadButtons codriver = new GamepadButtons(m_coDriverController, true);

  // temp controller for testing -matt
  // private PS4Controller m_ps4controller = new PS4Controller(1);
  // public PoseTelemetry pt = new PoseTelemetry();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
    Pref.deleteUnused();

    Pref.addMissing();

    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    LiveWindow.disableAllTelemetry();
    // Configure the button bindings

   // m_fieldSim.initSim();

    m_autoSelect = new AutoSelect(m_drive);

    // m_ls = new LightStrip(9, 60);

    // lc = LEDController.getInstance();
    lcI2 = LEDControllerI2C.getInstance();

    // PortForwarder.add(5800, "10.21.94.11", 5800);
    // PortForwarder.add(1181, "10.21.94.11", 1181);
    // PortForwarder.add(1182, "10.21.94.11", 1182);
    // PortForwarder.add(1183, "10.21.94,11", 1183);
    // PortForwarder.add(1184, "10.21.94.11", 1184);

    // () -> -m_coDriverController.getRawAxis(1),
    // () -> -m_coDriverController.getRawAxis(0),
    // () -> -m_coDriverController.getRawAxis(4)));
    // m_drive.setDefaultCommand(
    // new SetSwerveDrive(
    // m_drive,
    // () -> m_ps4controller.getRawAxis(1),
    // () -> m_ps4controller.getRawAxis(0),
    // () -> m_ps4controller.getRawAxis(2)));

    SmartDashboard.putData("SetDriverMode1", new SetDriverMode(m_vpe.m_cam1, true));
    SmartDashboard.putData("ResetDriverMode1", new SetDriverMode(m_vpe.m_cam1, false));

    SmartDashboard.putData("SetDriverMode2", new SetDriverMode(m_vpe.m_cam2, true));
    SmartDashboard.putData("ResetDriverMode2", new SetDriverMode(m_vpe.m_cam2, false));

    m_drive.setDefaultCommand(
        new SetSwerveDrive(
            m_drive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> leftJoystick.getRawAxis(4)));

    // codriver.leftTrigger.whileTrue(new JogTurnModule(
    //     m_drive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3)));

    // // individual modules
    // codriver.leftBumper.whileTrue(new JogDriveModule(
    //     m_drive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3),
    //     true));

    // // all modules
    // codriver.rightBumper.whileTrue(new JogDriveModule(
    //     m_drive,
    //     () -> -m_coDriverController.getRawAxis(1),
    //     () -> m_coDriverController.getRawAxis(0),
    //     () -> m_coDriverController.getRawAxis(2),
    //     () -> m_coDriverController.getRawAxis(3),
    //     false));

    JoystickButton zero = new JoystickButton(leftJoystick, 3);
    JoystickButton intakeIn= new JoystickButton(leftJoystick, 6);
    JoystickButton intakeOut = new JoystickButton(leftJoystick, 5);
    POVButton pusherDeploy = new POVButton(leftJoystick, 180);
    POVButton pusherRetract = new POVButton(leftJoystick, 0);
    JoystickButton runDemo = new JoystickButton(leftJoystick, 2);


    //subsystem commands
    zero.whileTrue(new ToggleFieldOriented(m_drive));

    intakeIn.whileTrue(new RunIntake(false, intake));
    intakeOut.whileTrue(new RunIntake(true, intake));
    intake.setDefaultCommand(new StopIntake(intake));

    pusher.setDefaultCommand(new StopPusher(pusher));
    pusherDeploy.whileTrue(new PusherOpenLoop(true, pusher));
    pusherRetract.whileTrue(new PusherOpenLoop(false, pusher));

    runDemo.whileTrue(futbol);
    // position turn modules individually
    // driver.X_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.FRONT_LEFT));
    // driver.A_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.FRONT_RIGHT));
    // driver.B_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.BACK_LEFT));
    // driver.Y_button.whenPressed(new PositionTurnModule(m_drive,
    // ModulePosition.BACK_RIGHT));



  }
  private final ParallelCommandGroup collect = new ParallelCommandGroup(
    new StrafeToGamePiece(DriveConstants.kDemoSpeedMetersPerSecond, limelight, m_drive, intake, optical),
    new RunIntake(false, intake)
  );

  private final SequentialCommandGroup futbol = new SequentialCommandGroup(
    collect,
    new StopIntake(intake),
    new RunIntake(true, intake)
  );

  // public void simulationPeriodic() {

  //   m_fieldSim.periodic();
  // }

  // public void periodic() {
  //   m_fieldSim.periodic();
  // }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  // public Command getAutonomousCommand() {
  // return autoBuilder.fullAuto(pathGroup);
  // }
}