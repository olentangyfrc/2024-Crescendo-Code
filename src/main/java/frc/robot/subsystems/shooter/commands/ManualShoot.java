// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class ManualShoot extends Command {
  private Shooter shooter;
  private static final double shooterVelocity = 76;
  private double shooterPivotDegrees;

  /** Creates a new ManualShoot. */
  public ManualShoot(Shooter shooter, ShootPosition position) {
    this.shooter = shooter;
    shooterPivotDegrees = position.getDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTargetShooterAngle(shooterPivotDegrees);
    shooter.setTargetShooterVelocity(shooterVelocity);
    shooter.setState(ShooterState.MANUAL_SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getState() == ShooterState.IDLE;
  }

  public enum ShootPosition {
    PODIUM(37.2),
    SUBWOOFER(54),
    AMP(32);

    private Rotation2d angle;

    private ShootPosition(double angleDegrees) {
      angle = Rotation2d.fromDegrees(angleDegrees);
    }

    public double getDegrees() {
      return angle.getDegrees();
    }
  }
}
