// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSbTab;

public class Climber extends SubsystemBase {
  private static final ShuffleboardTab tab = Shuffleboard.getTab("Climber");

  private static final double WINCH_REVS_PER_SHAFT_REV = 1.0 / (3 * 3 * 3);
  private static final double METERS_PER_WINCH_REV = 0.081118;

  private static final double RETRACTED_METERS = 0.52;

  private static final double TOLERANCE_METERS = 0.02;

  private final TalonFX motor;

  private final ProfiledPIDController climbingPidController = new ProfiledPIDController(600, 0, 0, new Constraints(99999999, 3));
  private final ElevatorFeedforward climbingFeedForward = new ElevatorFeedforward(0, 0, 0);

  private ClimberState state = ClimberState.POSITION_CONTROL;
  private ClimberState previousState = ClimberState.POSITION_CONTROL;

  private double manualVoltage = 0;

  /** Creates a new Climber. */
  public Climber(int motorId) {
    motor = new TalonFX(motorId, "*");
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    motor.getConfigurator().apply(motorConfig);

    climbingPidController.setTolerance(TOLERANCE_METERS);

    zeroMotor(0);
    climbingPidController.setGoal(getHeightFromGround());

    tab.addNumber("Height From Ground", this::getHeightFromGround);
    tab.addNumber("Height From Retracted", this::getHeightFromRetracted);
    tab.addNumber("Applied Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void periodic() {

    if(!DriveSbTab.getInstance().isClimberEnabled()) {
      motor.stopMotor();
      return;
    }

    switch(state) {
      case POSITION_CONTROL -> {
        motor.setVoltage(
          climbingFeedForward.calculate(climbingPidController.getSetpoint().velocity) +
          climbingPidController.calculate(getHeightFromGround())
        );

      }

      case MANUAL -> {
        motor.setVoltage(manualVoltage);
      }

      case DISABLED -> {
        motor.stopMotor();
      }
    }
  }

  /**
   * Begin locking to a position while under load.
   * 
   * @param position
   */
  public void enablePositionControl(ClimberPosition position) {
    resetPid();
    climbingPidController.setGoal(MathUtil.clamp(position.height, ClimberPosition.FULLY_RETRACTED.height, ClimberPosition.MAX_HEIGHT.height));
    state = ClimberState.POSITION_CONTROL;
  }

  public void setManual(double manualVoltage) {
    if(state != ClimberState.MANUAL) {
      previousState = state;
    }
    this.manualVoltage = manualVoltage;
    state = ClimberState.MANUAL;
  }

  public void exitManual() {
    climbingPidController.setGoal(getHeightFromGround());
    resetPid();
    state = previousState;
  }

  public void resetPid() {
    climbingPidController.reset(getHeightFromGround());
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

  public ClimberState getState() {
    return state;
  }

  public enum ClimberState {
    POSITION_CONTROL,
    MANUAL,
    DISABLED
  }

  public enum ClimberPosition {
    FULLY_RETRACTED(RETRACTED_METERS),
    STOW(RETRACTED_METERS),
    EXTENDED(1.16),
    MAX_HEIGHT(1.2);

    // Height from ground (meters)
    public double height;

    /**
     * Construct a new ClimberPosition enum value
     * 
     * @param height Height from ground (meters)
     */
    private ClimberPosition(double height) {
      this.height = height;
    }
  }
}
