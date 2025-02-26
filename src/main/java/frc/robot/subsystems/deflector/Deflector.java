package frc.robot.subsystems.deflector;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSbTab;

public class Deflector extends SubsystemBase {
    
    private static final double DEFLECTOR_GEAR_RATIO = 80.0/10.0 * 10;
    private static final double ZERO_POSITION = -55.0;
    private static final double MIN_POSITION = -55.0;
    private static final double MAX_POSITION = 90.0;

    private static final double MAX_VELOCITY = 99999.0;
    private static final double MAX_ACCELERATION = 300.0;

    private static final double ANGLE_TOLERANCE_DEGREES = 3;

    private ShuffleboardTab tab = Shuffleboard.getTab("Deflector");

    private Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    private ProfiledPIDController deflectorPID = new ProfiledPIDController(0.5, 0.0, 0.0, constraints);

    private TalonFX deflectorMotor;

    private Boolean manualMode = false;
    
    /**
     * Initializes the deflector class
     * @param deflectorCAN
     */
    public Deflector(int deflectorCAN) {
        tab.addNumber("Deflector Angle",() -> getPosition());

        deflectorMotor = new TalonFX(deflectorCAN, "*");

        
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.CurrentLimits.SupplyCurrentLimit = 2;
        deflectorMotor.getConfigurator().apply(configuration);
        
        zeroMotor();

        deflectorPID.setTolerance(ANGLE_TOLERANCE_DEGREES);
        setTargetPosition(DeflectorPosition.STOWED);
    }

    /**
     * Sets voltage based off of target position
     */
    public void periodic() {
        if(!DriveSbTab.getInstance().isDeflectorEnabled()) {
            deflectorMotor.stopMotor();
            return;
        }

        if (DriverStation.isTest()) {
            deflectorMotor.setVoltage(0.0);
        } else if (!manualMode) {
            deflectorMotor.setVoltage(deflectorPID.calculate(getPosition()));
        }
        
    }

    /**
     * Position values
     */
    private static final Map<DeflectorPosition, Double> positionValues = Map.of(
        DeflectorPosition.AMP, -6.5,
        DeflectorPosition.STOWED, -55.0,
        DeflectorPosition.PREP, -15.0
    );

    public boolean isAtPosition() {
        return Math.abs(deflectorPID.getGoal().position - getPosition()) < ANGLE_TOLERANCE_DEGREES;
    }

    /**
     * Returns position of the deflector bar in degrees past vertical with forward positive
     * @return
     */
    public double getPosition() {
        return deflectorMotor.getPosition().getValueAsDouble() * 360 / DEFLECTOR_GEAR_RATIO;
    }

    /**
     * Sets target position of deflector bar in degrees past vertical. Forward positive.
     * @param position
     */
    public void setTargetPositionDouble(double position) {
        // deflectorPID.setGoal(position);
        deflectorPID.setGoal(MathUtil.clamp(position, MIN_POSITION, MAX_POSITION));
    }

    /**
     * Sets the target position with a DeflectorPosition type.
     * @param position
     */
    public void setTargetPosition(DeflectorPosition position) {
        deflectorPID.setGoal(MathUtil.clamp(positionValues.get(position), MIN_POSITION, MAX_POSITION));
    }
    
    /**
     * Called at initialization, zeroes the deflector motor and pid against the hard stop
     */
    public void resetPID() {
        deflectorPID.reset(getPosition());
    }

    /**
     * Zeroes the motor
     */
    public void zeroMotor() {
        deflectorMotor.setPosition(ZERO_POSITION / 360 * DEFLECTOR_GEAR_RATIO);
        resetPID();
    }

    public void manualControl(double voltage) {
        manualMode = true;
        deflectorMotor.setVoltage(voltage);
    }

    public void exitManual() {
        deflectorMotor.setVoltage(0);
        setTargetPositionDouble(getPosition());
        deflectorPID.reset(getPosition());
        manualMode = false;
    }

    /**
     * Deflector Position list
     */
    public enum DeflectorPosition {
        STOWED,
        AMP,
        PREP
    }
}