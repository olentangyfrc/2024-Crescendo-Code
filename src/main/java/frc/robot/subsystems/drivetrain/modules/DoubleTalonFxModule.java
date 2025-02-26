package frc.robot.subsystems.drivetrain.modules;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class DoubleTalonFxModule extends SwerveModule {
    private static final double WHEEL_RADIUS = 0.0473075 * 1.3238;

    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private CANcoder angleEncoder;

    private double lastAppliedAngularVoltage = 0.0;
    private double lastAppliedDriveVoltage = 0.0;

    public DoubleTalonFxModule(String name, int angleCanId, int driveCanId, int angleEncoderPort, double maxSpeed, double driveGearRatio, double angleOffset, Rotation2d xAngle) {
        this(name, angleCanId, driveCanId, angleEncoderPort, maxSpeed, driveGearRatio, angleOffset, "rio", xAngle);
    }

    public DoubleTalonFxModule(String name, int angleCanId, int driveCanId, int angleEncoderPort, double maxSpeed, double driveGearRatio, double angleOffset, String canBusName, Rotation2d xAngle) {
        this.maxSpeed = maxSpeed;
        this.driveGearRatio = driveGearRatio;
        this.angleOffset = angleOffset;
        // Angle motor configuration
        angleMotor = new TalonFX(angleCanId, canBusName);

        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotor.getConfigurator().apply(angleConfig);

        // Drive motor configuration
        driveMotor = new TalonFX(driveCanId, canBusName);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.StatorCurrentLimit = 50;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        angleEncoder = new CANcoder(angleEncoderPort, canBusName);

        // Control loops
        angleController = new PIDController(12, 0, 0);
        angleController.enableContinuousInput(0.0, 2 * Math.PI);

        driveController = new PIDController(2, 0, 0.00);
        driveFF = new SimpleMotorFeedforward(0.2278, 2.4176, 0);

        this.xAngle = xAngle;
 }


    @Override
    public Rotation2d getAngle() {
        double sensorPosition = BaseStatusSignal.getLatencyCompensatedValue(angleEncoder.getAbsolutePosition(), angleEncoder.getVelocity());
        double rotations = MathUtil.inputModulus(sensorPosition - (angleOffset / 360), 0, 1);
        if(rotations < 0) {
            rotations += 1;
        }
        return Rotation2d.fromRotations(rotations);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * driveGearRatio * WHEEL_RADIUS * 2 * Math.PI; // Multiply by 10 due to sample rate
    }

    @Override
    public void setAngleVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        angleMotor.setControl(new VoltageOut(clampedVoltage, true, false, false, false));
        lastAppliedAngularVoltage = clampedVoltage;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
        driveMotor.setControl(new VoltageOut(clampedVoltage, true, false, false, false));
        lastAppliedDriveVoltage = clampedVoltage;
    }


    @Override
    public double getAngularDistance() {
        double sensorRotations = BaseStatusSignal.getLatencyCompensatedValue(angleMotor.getPosition(), angleMotor.getVelocity());
        return sensorRotations * driveGearRatio * WHEEL_RADIUS * 2 * Math.PI;
    }


    @Override
    public double getDriveDistance() {
        double sensorRotations = BaseStatusSignal.getLatencyCompensatedValue(driveMotor.getPosition(), driveMotor.getVelocity());
        return sensorRotations * driveGearRatio * WHEEL_RADIUS * 2 * Math.PI;
    }


    @Override
    public double getAngularVelocity() {
        return angleEncoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
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
