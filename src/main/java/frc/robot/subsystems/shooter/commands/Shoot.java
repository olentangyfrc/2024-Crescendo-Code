// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Shoot extends Command {
  private Shooter shooter;
  private boolean rotateDrive;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Drivetrain drivetrain, boolean rotateDrive) {
    this.shooter = shooter;
    this.rotateDrive = rotateDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SubsystemManager.getInstance().getIntake().setIntakeRollersDisabled(true);
    // SubsystemManager.getInstance().getIntake().setState(IntakeState.DEPLOYING);
    shooter.setState(ShooterState.SPEAKER_MOVING_SHOOTING);
    
    if(!rotateDrive) {
      shooter.plsDisableAngleSnap();
    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubsystemManager.getInstance().getIntake().setIntakeRollersDisabled(false);

    shooter.plsEnableAngleSnap();
    shooter.setShouldPreAlignNext(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getState() == ShooterState.IDLE;
  }
}
