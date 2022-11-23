// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.swerve.JogDriveModule;
import frc.robot.commands.swerve.JogTurnModule;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LEDControllerI2C;

public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_drive = new DriveSubsystem();

  LEDControllerI2C lcI2;

  public final FieldSim m_fieldSim = new FieldSim(m_drive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // The driver's controller

  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  final GamepadButtons codriver = new GamepadButtons(m_coDriverController, true);

  //temp controller for testing -matt
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

    m_fieldSim.initSim();

    // m_ls = new LightStrip(9, 60);

    // lc = LEDController.getInstance();
    lcI2 = LEDControllerI2C.getInstance();

    initializeAutoChooser();

    // PortForwarder.add(5800, "10.21.94.11", 5800);
    // PortForwarder.add(1181, "10.21.94.11", 1181);
    // PortForwarder.add(1182, "10.21.94.11", 1182);
    // PortForwarder.add(1183, "10.21.94,11", 1183);
    // PortForwarder.add(1184, "10.21.94.11", 1184);

    // () -> -m_coDriverController.getRawAxis(1),
    // () -> -m_coDriverController.getRawAxis(0),
    // () -> -m_coDriverController.getRawAxis(4)));
    // m_drive.setDefaultCommand(
    //     new SetSwerveDrive(
    //         m_drive,
    //         () -> m_ps4controller.getRawAxis(1),
    //         () -> m_ps4controller.getRawAxis(0),
    //         () -> m_ps4controller.getRawAxis(2)));
    m_drive.setDefaultCommand(
        new SetSwerveDrive(
            m_drive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> leftJoystick.getRawAxis(2)));

    codriver.leftTrigger.whileTrue(new JogTurnModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3)));

    // individual modules
    codriver.leftBumper.whileTrue(new JogDriveModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3),
        true));

    // all modules
    codriver.rightBumper.whileTrue(new JogDriveModule(
        m_drive,
        () -> -m_coDriverController.getRawAxis(1),
        () -> m_coDriverController.getRawAxis(0),
        () -> m_coDriverController.getRawAxis(2),
        () -> m_coDriverController.getRawAxis(3),
        false));

    JoystickButton button_8 = new JoystickButton(leftJoystick, 8);
    JoystickButton button_7 = new JoystickButton(leftJoystick, 7);

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

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
   

    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void simulationPeriodic() {
 
    m_fieldSim.periodic();
  }

  public void periodic() {
 m_fieldSim.periodic();
  }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
