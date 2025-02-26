// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class ZeroIntake extends Command {
  private Rotation2d angle;
  private Intake intake;


  /** Creates a new ZeroIntake. */
  public ZeroIntake(Rotation2d angle, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.intake = intake;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.zeroAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isTeleopEnabled();
  }
}
