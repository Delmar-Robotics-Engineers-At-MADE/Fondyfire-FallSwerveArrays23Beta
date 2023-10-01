// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSparkMax;

public class FieldSim {
  private final DriveSubsystem m_swerveDrive;

  private SwerveModuleSparkMax[] modules = new SwerveModuleSparkMax[4];
  
 

  private final Field2d m_field2d = new Field2d();

  private final Pose2d[] m_swerveModulePoses = new Pose2d[4];

  public FieldSim(DriveSubsystem swerveDrive) {

    m_swerveDrive = swerveDrive;

    modules[0] = m_swerveDrive.m_frontLeft;
    // modules[1] = m_swerveDrive.m_frontRight;
    // modules[2] = m_swerveDrive.m_backLeft;
    // modules[3] = m_swerveDrive.m_backRight;
  }

  private int tst;

  private double starttime;

  private double endtime;

  public void initSim() {
  }

  public Field2d getField2d() {

    return m_field2d;
  }

  private void updateRobotPoses() {

    Pose2d testing = m_swerveDrive.getEstimatedPose();

    m_field2d.setRobotPose(m_swerveDrive.getEstimatedPose());

    for (int i = 0; i < 4; i++) {

      Translation2d updatedPositions = DriveConstants.kModuleTranslations[i]

          .rotateBy(m_swerveDrive.getEstimatedPose().getRotation())

          .plus(m_swerveDrive.getEstimatedPose().getTranslation());

      m_swerveModulePoses[i] = new Pose2d(
          updatedPositions,
          modules[i]
              .getHeadingRotation2d()
              .plus(m_swerveDrive.getHeadingRotation2d()));

    }

    m_field2d.getObject("Swerve Modules")

        .setPoses(m_swerveModulePoses);

  }

  public void periodic() {

    updateRobotPoses();

    if (RobotBase.isSimulation())

      simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {
  }
}
