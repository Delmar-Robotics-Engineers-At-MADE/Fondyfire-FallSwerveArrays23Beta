package frc.robot.commands.Vision;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Optical;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class StrafeToGamePiece extends ProfiledPIDCommand {
  
  private Intake intake;
  private Optical optical;
  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kYawP, 0.0, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public StrafeToGamePiece(double strafeSpeed,
                  Limelight limelight, DriveSubsystem drive, Intake intake, Optical sensor) {
    super(
        m_PID,
        // Close loop on heading
        limelight::getBestGamepieceYaw,
        // Set reference to target
        CameraConstants.kGamepieceCenterPos,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(0, strafeSpeed, 0),
        // Require the drive
        drive);

    intake = intake;
    optical = sensor;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kMaxYawRateDegPerS, DriveConstants.kMaxYawAccelerationDegPerSSquared);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Gamepiece PID 2", m_PID);
        turnTab.addDouble("Gamepiece error 2", () -> m_PID.getPositionError());
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new turn to limelight command created");
  
      addRequirements(limelight, sensor);
  }


  @Override
  public boolean isFinished() {
    if(optical.getState()){
        return true;
    }
    else {
        return false;
    }
  }
}

