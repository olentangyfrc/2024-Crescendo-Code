package frc.robot.subsystems.drivetrain.modules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;

public class SingleFalconModule extends SwerveModule {
    private static final double WHEEL_RADIUS = 0.0492125;

    private CANSparkMax angleMotor;
    private TalonFX driveMotor;

    private AnalogEncoder angleEncoder;

    private double lastAngularDistance;
    private double lastAngularDistanceMeasurementSeconds;

    private double lastAppliedAngularVoltage = 0.0;
    private double lastAppliedDriveVoltage = 0.0;

    /**
     * Construct a new SingleFalconModule
     * 
     * @param angleCanId CAN ID of the angle motor
     * @param driveCanId CAN ID of the drive motor
     * @param angleEncoderPort Analog port of the module's encoder
     * @param maxSpeed Maximum speed of the drive motor (meters per second)
     * @param driveGearRatio Drive wheel rotations per drive motor output shaft rotation
     * @param angleOffset Angle to be subtracted from the raw angle encoder value so that forwards is 0 degrees (degrees)
     */
    public SingleFalconModule(int angleCanId, int driveCanId, int angleEncoderPort, double maxSpeed, double driveGearRatio, double angleOffset, Rotation2d xAngle) {
        this.maxSpeed = maxSpeed;
        this.driveGearRatio = driveGearRatio;
        this.angleOffset = angleOffset;

        angleMotor = new CANSparkMax(angleCanId, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        angleMotor.setIdleMode(IdleMode.kBrake);
        // angleMotor.setInverted();

        driveMotor = new TalonFX(driveCanId);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);

        // angleEncoder = new AnalogInput(angleEncoderPort);
        angleEncoder = new AnalogEncoder(angleEncoderPort);

        //TODO: Tune control loops
        angleController = new PIDController(10, 0, 0);
        angleController.enableContinuousInput(0.0, 2 * Math.PI);

        driveController = new PIDController(2.6, 0, 0.002);
        driveFF = new SimpleMotorFeedforward(0, 3.8, 0.72058);

        lastAngularDistance = getAngularDistance();
        lastAngularDistanceMeasurementSeconds = Timer.getFPGATimestamp();

        this.xAngle = xAngle;

    }

    @Override
    public Rotation2d getAngle() {
        // Raw angle
        double angle = (-angleEncoder.get() * 2 * Math.PI); // Convert rotations to an angle in radians
        // Convert the offset into radians and subtract it from the angle
        angle -= angleOffset * Math.PI / 180;
        // Add a full rotation in radians to make sure the angle is always positive
        angle += 2 * Math.PI;
        // modulo the angle by a full rotation in radians to restrict it to the range [0,2Pi)
        angle %= 2 * Math.PI;
        return new Rotation2d(angle);
    }

    @Override
    public double getAngularDistance() {
        return -angleEncoder.getDistance() * 2 * Math.PI;
    }

    @Override
    public double getAngularVelocity() {
        double time = Timer.getFPGATimestamp();
        double distance = getAngularDistance();

        double dw = distance - lastAngularDistance;
        double dt = time - lastAngularDistanceMeasurementSeconds;

        lastAngularDistance = distance;
        lastAngularDistanceMeasurementSeconds = time;

        return dw / ((dt != 0)? dt : 0.002);
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * driveGearRatio * WHEEL_RADIUS * 2 * Math.PI; // Multiply by 10 due to sample rate
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }
    
    @Override
    public void setAngleVoltage(double voltage) {
        // System.out.println("Voltage: " + voltage);
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        angleMotor.setVoltage(clampedVoltage);
        lastAppliedAngularVoltage = clampedVoltage;
    }
    
    @Override
    public void setDriveVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        driveMotor.setVoltage(clampedVoltage);
        lastAppliedDriveVoltage = clampedVoltage;
    }

    @Override
    public double getDriveDistance() {
        return driveMotor.getPosition().getValueAsDouble() * driveGearRatio * WHEEL_RADIUS * 2 * Math.PI;
    }

    @Override
    public double getLastAppliedAngularVoltage() {
        return lastAppliedAngularVoltage;
    }

    @Override
    public double getLastAppliedDriveVoltage() {
        return lastAppliedDriveVoltage;
    }
}
