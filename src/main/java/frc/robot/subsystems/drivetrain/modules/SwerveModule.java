package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    protected double angleOffset;

    protected PIDController angleController;
    
    protected PIDController driveController;
    protected SimpleMotorFeedforward driveFF;

    protected SwerveModuleState targetState = new SwerveModuleState();

    protected double maxSpeed;
    protected double driveGearRatio;

    protected Rotation2d xAngle;

    /**
     * @return The current angle of the module as a Rotation2D
     */
    public abstract Rotation2d getAngle();

    /**
     * Set the target angle of the module (counterclockwise)
     * 
     * @param angle
     */
    public void setTargetAngle(Rotation2d angle) {
        targetState.angle = wrapAngle(angle);
    }

    /**
     * Get the current velocity of the module
     * 
     * @return Current module drive velocity in meters per second.
     */
    public abstract double getVelocity();

    /**
     * Set the target velocity of the module
     * 
     * @param velocity Target velocity in meters per second.
     */
    public void setTargetVelocity(double velocity) {
        targetState.speedMetersPerSecond = velocity;
    }

    /**
     * @return The current state of the module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * @return The desired state of the module
     */
    public SwerveModuleState getTargetState() {
        return new SwerveModuleState(targetState.speedMetersPerSecond, targetState.angle);
    }

    /**
     * Set the desired state of the module.
     * 
     * @param targetState
     */
    public void setTargetState(SwerveModuleState targetState) {
        if(targetState.speedMetersPerSecond < 0.05) {
            targetState.speedMetersPerSecond = 0;
        }

        //Prevent rotating module if speed is very small. Prevents Jittering.
        // Rotation2d angle = (Math.abs(targetState.speedMetersPerSecond) <= 0.05) ? xAngle : wrapAngle(targetState.angle);
        this.targetState = SwerveModuleState.optimize(new SwerveModuleState(targetState.speedMetersPerSecond, wrapAngle(targetState.angle)), getAngle());
    }

    public void update() {
        update(false);
    }

    public void update(boolean directVoltageDrive) {
        if(!directVoltageDrive) {
            setAngleVoltage(angleController.calculate(getAngle().getRadians(), targetState.angle.getRadians()));
            setDriveVoltage(driveController.calculate(getVelocity(), targetState.speedMetersPerSecond * targetState.angle.minus(getAngle()).getCos()) + driveFF.calculate( targetState.speedMetersPerSecond));
            
        } else {
            setAngleVoltage(angleController.calculate(getAngle().getRadians(), 0));
        }
    }

    /**
     * Get the distance the angular motor has traveled
     * 
     * @return Distance traveled in radians
     */
    public abstract double getAngularDistance();

    public abstract void stop();

    /**
     * Get the distance the drive motor has traveled
     * 
     * @return Distance traveled in meters
     */
    public abstract double getDriveDistance();

    /**
     * Get the angular velocity of the angle motor
     * 
     * @return Angle velocity of angle motor in radians per second
     */
    public abstract double getAngularVelocity();

    /**
     * Do not use this unless characterizing with SysId!!!
     * <p>
     * Set the voltage of the angle motor
     * 
     * @param voltage voltage to apply on interval [-12, 12]. Clamped to interval if outside of it.
     */
    public abstract void setAngleVoltage(double voltage);

    /**
     * Do not use this unless characterizing with SysId!!!
     * <p>
     * Set the voltage of the drive motor
     * 
     * @param voltage voltage to apply on interval [-12, 12]. Clamped to interval if outside of it.
     */
    public abstract void setDriveVoltage(double voltage);

    /**
     * Get the last voltage that was applied to the angular motor
     * <p>
     * This is for use in SysId characterization
     * 
     * @return The last voltage applied in volts
     */
    public abstract double getLastAppliedAngularVoltage();

    /**
     * Get the last voltage that was applied to the drive motor
     * <p>
     * This is for use in SysId characterization
     * 
     * @return The last voltage applied in volts
     */
    public abstract double getLastAppliedDriveVoltage();

    /**
     * Wrap a Rotation2D between 0.0 and 2pi
     * 
     * @param angle The angle to wrap
     * @return The wrapped angle
     */
    private static Rotation2d wrapAngle(Rotation2d angle) {
        return new Rotation2d(MathUtil.inputModulus(angle.getRadians(), 0, 2 * Math.PI));
    }
}
