// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pusher;

public class PusherOpenLoop extends CommandBase {
  Pusher pusher;
  Boolean reversed;
  /** Creates a new PusherOpenLoop. */
  public PusherOpenLoop(Boolean reversed, Pusher pusher) {
    this.pusher = pusher;
    this.reversed= reversed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pusher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pusher.runClimberManual(reversed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
