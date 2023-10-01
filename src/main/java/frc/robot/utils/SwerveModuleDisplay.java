// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class SwerveModuleDisplay extends SubsystemBase {

    private final DriveSubsystem m_drive;

    private static FFDisplay m_disp = new FFDisplay("a");

    public SwerveModuleDisplay(DriveSubsystem drive) {

        m_drive = drive;



        m_disp.putBoolean("S", m_drive.m_frontLeft.checkCAN());
        m_disp.putNumber("LLL", Robot.lpctra);

    }

    @Override
    public void initSendable(SendableBuilder builder) {

        // front

        builder.addBooleanProperty("m_fl_can_ok", m_drive.m_frontLeft::checkCAN, null);
        builder.addBooleanProperty("m_fl_turn_stopped", m_drive.m_frontLeft::turnIsStopped, null);

        builder.addDoubleProperty("m_fl_turn_velocity", m_drive.m_frontLeft::getTurnVelocity, null);
        builder.addDoubleProperty("m_fl_turn_angle", m_drive.m_frontLeft::getTurnPosition, null);

        builder.addDoubleProperty("m_fl_drive_velocity", m_drive.m_frontLeft::getDriveVelocity, null);
        builder.addDoubleProperty("m_fl_drive_position", m_drive.m_frontLeft::getDrivePosition, null);

        // builder.addBooleanProperty("m_fr_can_ok", m_drive.m_frontRight::checkCAN, null);
        // builder.addBooleanProperty("m_fr_turn_stopped", m_drive.m_frontRight::turnIsStopped, null);

        // builder.addDoubleProperty("m_fr_turn_velocity", m_drive.m_frontRight::getTurnVelocity, null);
        // builder.addDoubleProperty("m_fr_turn_angle", m_drive.m_frontRight::getTurnPosition, null);

        // builder.addDoubleProperty("m_fr_drive_velocity", m_drive.m_frontRight::getDriveVelocity, null);
        // builder.addDoubleProperty("m_fr_drive_position", m_drive.m_frontRight::getDrivePosition, null);

        // back

        // builder.addBooleanProperty("m_bl_can_ok", m_drive.m_backLeft::checkCAN, null);
        // builder.addBooleanProperty("m_bl_turn_stopped", m_drive.m_backLeft::turnIsStopped, null);

        // builder.addDoubleProperty("m_bl_turn_velocity", m_drive.m_backLeft::getTurnVelocity, null);
        // builder.addDoubleProperty("m_bl_turn_angle", m_drive.m_backLeft::getTurnPosition, null);

        // builder.addDoubleProperty("m_bl_drive_velocity", m_drive.m_backLeft::getDriveVelocity, null);
        // builder.addDoubleProperty("m_bl_drive_position", m_drive.m_backLeft::getDrivePosition, null);

        // builder.addBooleanProperty("m_br_can_ok", m_drive.m_backRight::checkCAN, null);
        // builder.addBooleanProperty("m_br_turn_stopped", m_drive.m_backRight::turnIsStopped, null);

        // builder.addDoubleProperty("m_br_turn_velocity", m_drive.m_backRight::getTurnVelocity, null);
        // builder.addDoubleProperty("m_br_turn_angle", m_drive.m_backRight::getTurnPosition, null);

        // builder.addDoubleProperty("m_br_drive_velocity", m_drive.m_backRight::getDriveVelocity, null);
        // builder.addDoubleProperty("m_br_drive_position", m_drive.m_backRight::getDrivePosition, null);
    }
}
