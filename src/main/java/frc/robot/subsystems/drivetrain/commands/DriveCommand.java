package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DriveSignal;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

/**
 * This command is run during teleop. It uses inputs from the main IO controller to drive the robot.
 */
public class DriveCommand extends Command {
  private Drivetrain drivetrain;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IO io = IO.getInstance();

    double xVel = io.getController().getLeftY() * Drivetrain.MAX_LINEAR_SPEED * ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue || !drivetrain.isFieldOriented())? 1 : -1);
    double yVel = -io.getController().getLeftX() * Drivetrain.MAX_LINEAR_SPEED * ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue || !drivetrain.isFieldOriented())? 1 : -1);
    double omegaVel = -io.getController().getRightX() * Drivetrain.MAX_ANGULAR_SPEED;

    if(SubsystemManager.getInstance().getShooter().getState() == ShooterState.SPEAKER_MOVING_SHOOTING) {
      double angle = Math.atan2(yVel, xVel);
      double mag = Math.hypot(xVel, yVel);
      mag = Math.min(1.3, mag);

      yVel = mag * Math.sin(angle);
      xVel = mag * Math.cos(angle);
    }

    ChassisSpeeds speeds = new ChassisSpeeds(
      xVel,
      yVel,
      omegaVel
    );

    drivetrain.setSignal(new DriveSignal(speeds, drivetrain.isFieldOriented()));
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
