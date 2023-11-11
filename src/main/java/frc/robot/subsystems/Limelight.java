// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Limelight extends SubsystemBase {

  private double m_bestLimelightYaw = 0.0;
  private double  m_bestLimelightDistance = 0.0;

  private static NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private boolean m_limelightOn;
  
  /** Creates a new DriveSubsystem. */
  public Limelight() {

    ShuffleboardTab cameraTab = Shuffleboard.getTab("Cameras");
    cameraTab.addDouble("Limelight Yaw", () -> m_bestLimelightYaw);
    cameraTab.addDouble("Limelight Distance", () -> m_bestLimelightDistance);
    cameraTab.addBoolean("Limelight On", () -> m_limelightOn);

  }

  // for summer lightsaber fighting
//   public double getInstantaneousYaw() {
//     return m_limelightTable.getEntry("tx").getDouble(0.0);
//   }

//   public double getInstantaneousArea() {
//     return m_limelightTable.getEntry("ta").getDouble(0.0);
//   }


  // call this from PID command to turn robot to best target
  public double getBestLimelightYaw() {
    updateBestLimelight(0.0, true);
    return m_bestLimelightYaw;
  }

  public double getBestGamepieceYaw() {
    updateBestLimelight(CameraConstants.kGamepieceCenterPos, false);
    return m_bestLimelightYaw;
  }

  public double getBestLimelightDistance() {
    updateBestLimelight(0.0, true);
    return m_bestLimelightDistance;
  }

  public void updateBestLimelight(double noTargetFallbackYaw, boolean needLight) {
    if (needLight) {turnLightOnOrOff(true);}
    double targetsSeen = m_limelightTable.getEntry("tv").getDouble(0.0);
    if (targetsSeen > 0) {
      m_bestLimelightYaw = m_limelightTable.getEntry("tx").getDouble(0.0);
      m_bestLimelightDistance = 0.0; // try calculating this using AprilTag util
    } else {
      m_bestLimelightYaw = noTargetFallbackYaw;
      m_bestLimelightDistance = 0.0;
    }
  }

  public void turnLightOnOrOff (boolean turnOn) {
    boolean turnOff = !turnOn;
    boolean lightIsOff = !m_limelightOn;
    if (m_limelightOn && turnOff) {
      System.out.println("sending command to turn OFF light");
      m_limelightTable.getEntry("ledMode").setNumber(1.0); // LED off
      m_limelightOn = false;
    } else if (lightIsOff && turnOn) {
      System.out.println("sending command to turn ON light");
      m_limelightTable.getEntry("ledMode").setNumber(3.0); // LED on bright
      m_limelightOn = true;
    }
  }
    
}






