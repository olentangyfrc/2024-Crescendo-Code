package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Reset the estimated position of the drivetrain
 */
public class ResetLocation extends InstantCommand {
  private Drivetrain drivetrain;
  private Pose2d location;

  public ResetLocation(Drivetrain drivetrain, Pose2d location) {
    this.drivetrain = drivetrain;
    this.location = location;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetLocation(location);
  }
}
