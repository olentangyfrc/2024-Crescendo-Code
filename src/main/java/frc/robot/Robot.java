// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.AutoManager;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Logger logger = Logger.getLogger(this.getClass().getName());

  private Command autoCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    SubsystemManager.getInstance().init();

    // ShootingParams params = ShooterUtils.getShootingWhileMovingParams(ShooterUtils.getSpeakerLoc().plus(new Translation2d(3, 0)), new Vector2d(3, 0));

    // // System.out.println(ShooterUtils.speakerVelToFieldVel(new Vector2d(), new Translation2d()));

    // System.out.println("Angle: " + params.shooterPivotDegrees);
    // System.out.println("Flywheel Velocity: " + params.shooterVelociy);
    // System.out.println("Snap Angle: " + params.driveSnapDegrees);
    // System.out.println("Drive Velocity X: " + params.fieldRelativeRobotVelocity.getX());
    // System.out.println("Drive Velocity Y: " + params.fieldRelativeRobotVelocity.getY());
    // System.out.println("Drive Velocity mag: " + params.fieldRelativeRobotVelocity.magnitude());
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(SubsystemManager.getInstance().getLeds() != null) {
      SubsystemManager.getInstance().getLeds().update();
    } 
  }
  
  @Override
  public void disabledExit() {
    if(SubsystemManager.getInstance().getIntake() != null) {
      SubsystemManager.getInstance().getIntake().resetPivotPid();
    }
    
    if(SubsystemManager.getInstance().getShooter() != null) {
      SubsystemManager.getInstance().getShooter().resetAnglePid();
    }

    if(SubsystemManager.getInstance().getClimber() != null) {
      SubsystemManager.getInstance().getClimber().resetPid();
    }
      
    if(SubsystemManager.getInstance().getDeflector() != null) {
      SubsystemManager.getInstance().getDeflector().resetPID();
    }
  }
  
  @Override
  public void autonomousInit() {
    if(autoCommand != null) {
      autoCommand.cancel();
    }

    if(SubsystemManager.getInstance().getDrivetrain() != null) {
      SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
      SubsystemManager.getInstance().getDrivetrain().disablePassiveSnap();
    }
    
    AutoManager autoManager = SubsystemManager.getInstance().getAutoManager();
    if(autoManager != null) {
      autoCommand = autoManager.getSelectedAutoCommand();
      autoCommand.schedule();
    } else {
      logger.warning("Attempted to run auto when no AutoManager has been initialized!!!");
    }
  }
  
  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopInit() {
    if(autoCommand != null) {
      autoCommand.cancel();
    }

    if(SubsystemManager.getInstance().getDrivetrain() != null) {
      SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
      SubsystemManager.getInstance().getDrivetrain().disablePassiveSnap();
    }
    

    Intake intake = SubsystemManager.getInstance().getIntake();
    if(intake != null) {
      intake.setCenteringNote(false);
    }
  }

  @Override
  public void teleopPeriodic() {}
  
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void testInit() {
    if(autoCommand != null) {
      autoCommand.cancel();
    }

    if(SubsystemManager.getInstance().getIntake() != null) {
      SubsystemManager.getInstance().getIntake().zeroAngle(Rotation2d.fromRadians(IntakePosition.ZERO_POSITION.getRadians()));
    }
    if(SubsystemManager.getInstance().getDeflector() != null) {
      SubsystemManager.getInstance().getDeflector().zeroMotor();
    }
    // SubsystemManager.getInstance().getShooter().zeroAngle(Rotation2d.fromDegrees(-19));
  }
  
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
