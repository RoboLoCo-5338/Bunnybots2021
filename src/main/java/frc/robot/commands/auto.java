// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSystem;

public class auto extends CommandBase {
  /** Creates a new auto. */
  double target;
  double targetTime = 6500;
  public auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.driveSystem.autoDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - target > targetTime;
  }
}
