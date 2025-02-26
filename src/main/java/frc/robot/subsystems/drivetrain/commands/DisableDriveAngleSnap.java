package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Stop snapping to an angle
 */
public class DisableDriveAngleSnap extends InstantCommand {
  private Drivetrain drivetrain;

  public DisableDriveAngleSnap(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.removeSnapAngle();
  }
}
