// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Climber extends SubsystemBase {
  
  private static final double WINCH_REVS_PER_SHAFT_REV = 1.0 / (3 * 3 * 3);
  private static final double METERS_PER_WINCH_REV = 0.081118;

  private static final double RETRACTED_METERS = 0.52;

  private static final double TOLERANCE_METERS = 0.02;

  private final TalonFX motor;

  private final ProfiledPIDController climbingPidController = new ProfiledPIDController(600, 0, 0, new Constraints(99999999, 3));
  private final ElevatorFeedforward climbingFeedForward = new ElevatorFeedforward(0, 0, 0);
  private final int motorId = 0;

  private DoublePublisher heightFromGround;
  private DoublePublisher heightFromRetracted;
  private DoublePublisher appliedVoltage;
  public Climber() {
    motor = new TalonFX(motorId, "*");
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motor.getConfigurator().apply(motorConfig);

    climbingPidController.setTolerance(TOLERANCE_METERS);

    zeroMotor(0);
    climbingPidController.setGoal(getHeightFromGround());
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("climber");
    heightFromGround = table.getDoubleTopic("heightFromGround").publish();
    heightFromRetracted = table.getDoubleTopic("heightFromRetracted").publish();
    appliedVoltage = table.getDoubleTopic("appliedVoltage").publish();


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command positionControl() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          motor.setVoltage(
          climbingFeedForward.calculate(climbingPidController.getSetpoint().velocity) +
          climbingPidController.calculate(getHeightFromGround())
        );
        });
  }
  public Command manualVoltage(double manualVoltage) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        motor.setVoltage(manualVoltage);
        });
  }
  public Command disable(double manualVoltage) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        motor.stopMotor();
        });
  }
  public void zeroMotor(double heightFromRetracted) {
    motor.setPosition((heightFromRetracted) / METERS_PER_WINCH_REV / WINCH_REVS_PER_SHAFT_REV);
  }
  public double getHeightFromGround() {
    return getHeightFromRetracted() + RETRACTED_METERS;
  }
  public double getHeightFromRetracted() {
    return motor.getPosition().getValueAsDouble() * WINCH_REVS_PER_SHAFT_REV * METERS_PER_WINCH_REV;
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    heightFromGround.set(getHeightFromGround());
    heightFromRetracted.set(getHeightFromRetracted());
    appliedVoltage.set(motor.getMotorVoltage().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
