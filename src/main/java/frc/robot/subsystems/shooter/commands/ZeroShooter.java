// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.Shooter;

public class ZeroShooter extends Command {
  private Rotation2d angle;
  private Shooter shooter;


  /** Creates a new ZeroIntake. */
  public ZeroShooter(Rotation2d angle, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.shooter = shooter;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.zeroAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isTeleopEnabled();
  }
}
