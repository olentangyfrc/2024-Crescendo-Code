package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.IO;
import frc.robot.IO.IO.XboxControllerName;
import frc.robot.subsystems.DriveSbTab;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.intake.commands.ZeroIntake;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.sysid.Characterizable;

public class Intake extends SubsystemBase implements Characterizable {

    // Ratio of MotorShaft:PivotMechanism
    private static final double MOTOR_SHAFT_TO_PIVOT_MECHANISM = (1.0 / 5) * (1.0 / 3) * (16.0 / 34);

    private static final double MAX_VELOCITY = 80; // Radians per second
    private static final double MAX_ACCELERATION = 8.5; // Radians per second squared

    // In radians
    private static final double PIVOT_TOLERANCE = 3.0 / 360 * 2 * Math.PI;

    // These are percent output [-1, 1]
    private static final double INTAKE_SPEED = 0.6;
    private static final double FEED_SPEED = 0.25;
    private static final double EJECT_SPEED = -1;

    private static final double FEED_DELAY = 0;

    private static final double FAST_PERIOD = 0.01;

    private TalonFX pivotMotor;
    private TalonFX intakeMotor;

    private ProfiledPIDController pivotPID;
    private ArmFeedforward pivotFF;

    private static final int INTAKE_ID = 3;
    private DigitalInput intakeSensor = new DigitalInput(INTAKE_ID);

    // Used for determining time since a state was entered.
    private double feedStartSeconds = 0;

    private IntakeState state;

    private boolean intakeProximity = false;

    private boolean isNoteSnapping = false;
    private boolean centeringNote = false;
    private boolean disableIntakeRollers = false;

    private double noteIntaked = Double.NaN;
    
    /**
     * Construct a new Intake
     */
    public Intake(int pivotCanId, int intakeCanId) {
        
        pivotMotor = new TalonFX(pivotCanId, "*");
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = 15;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(pivotConfig);
        
        intakeMotor = new TalonFX(intakeCanId, "*");
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(intakeConfig);
        
        pivotFF = new ArmFeedforward(0, 0.45, 0);
        pivotPID = new ProfiledPIDController(30, 0, 0.3, new Constraints(MAX_VELOCITY, MAX_ACCELERATION), 0.02);
        pivotPID.setTolerance(PIVOT_TOLERANCE);
        pivotPID.setGoal(getAngle().getRadians());
        
        state = IntakeState.RETRACTED;

        zeroAngle(IntakePosition.ZERO_POSITION.angle);

        new Notifier(this::fast_periodic).startPeriodic(FAST_PERIOD);


        Shuffleboard.getTab("Intake").addNumber("Intake Angle", (() -> getAngle().getDegrees()));
        Shuffleboard.getTab("Intake").addBoolean("Sensor Proximity", () -> intakeSensor.get());
        Shuffleboard.getTab("Intake").add("Zero Intake", new ZeroIntake(Rotation2d.fromRadians(IntakePosition.ZERO_POSITION.getRadians()), this));
        
        Shuffleboard.getTab("Intake").addNumber("Profile setpoint", () -> pivotPID.getSetpoint().position / 2 / Math.PI * 360);
        
        Shuffleboard.getTab("Intake").addNumber("Pivot Current", () -> SubsystemManager.getInstance().getPdp().getCurrent(8));
        Shuffleboard.getTab("Intake").addNumber("Pivot Acceleration", () -> pivotMotor.getAcceleration().getValueAsDouble() * MOTOR_SHAFT_TO_PIVOT_MECHANISM * 2 * Math.PI);
        Shuffleboard.getTab("Intake").addNumber("Pivot Velocity", () -> getPivotVelocity());
        
        Shuffleboard.getTab("Intake").addNumber("Wheel Current", () -> intakeMotor.getTorqueCurrent().getValueAsDouble());

        Shuffleboard.getTab("Intake").addString("State", () -> state.toString());
        
    }

    /**
     * This is similar to the default SubsystemBase periodic method.
     * <p>
     * A custom method is being used to reduce the period to 0.01 second rather than 0.02.
     */
    public void fast_periodic() {
        if(!DriveSbTab.getInstance().isIntakeEnabled()) {
            intakeMotor.stopMotor();
            return;
        }

        intakeProximity = intakeSensor.get();
        double targetRadians = Double.NaN;
        SmartDashboard.putNumber("Intake Rollers", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Deploying", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Note Centering", centeringNote);
        double centeringAngle = 0.0;
        switch(state) {
            case RETRACTED -> {
                if(isNoteSnapping) {
                    isNoteSnapping = false;
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                }

                targetRadians = IntakePosition.RETRACTED.getRadians();
                intakeMotor.stopMotor();
            }
            case DEPLOYING -> {
                targetRadians = IntakePosition.DEPLOYED.getRadians();
                                
                if(!DriverStation.isAutonomous() && DriveSbTab.getInstance().isNoteAlignmentEnabled() && centeringNote) {
                    SubsystemManager.getInstance().getVision().getNoteCamera().run();
                    if (SubsystemManager.getInstance().getVision().getNoteCamera().getPresent()){
                        isNoteSnapping = true;
                        centeringAngle = SubsystemManager.getInstance().getVision().getNoteCamera().getAngle();
                        
                        if( Math.abs(SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees() - SubsystemManager.getInstance().getVision().getNoteCamera().getNoteYaw()) > 3){
                            Rotation2d relativeNoteAngle = Rotation2d.fromDegrees(centeringAngle);
                            SubsystemManager.getInstance().getDrivetrain().setSnapAngle(relativeNoteAngle);
                        }
                    }else{
                        isNoteSnapping = false;
                        SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    }
                }

                // Transfer criteria for Deployed state
                if(Math.abs(getAngle().getRadians() - targetRadians) < PIVOT_TOLERANCE) {
                    state = IntakeState.DEPLOYED;
                }
            }
            case DEPLOYED -> {
                if(disableIntakeRollers) {
                    intakeMotor.stopMotor();
                } else {
                    intakeMotor.set(INTAKE_SPEED);
                }

                if(DriveSbTab.getInstance().isNoteAlignmentEnabled() && SubsystemManager.getInstance().getShooter().getState() != ShooterState.SPEAKER_MOVING_SHOOTING && (centeringNote)) {
                    SubsystemManager.getInstance().getVision().getNoteCamera().run();
                    if (SubsystemManager.getInstance().getVision().getNoteCamera().getPresent()){
                        isNoteSnapping = true;
                        centeringAngle = SubsystemManager.getInstance().getVision().getNoteCamera().getAngle();
                        
                        if( Math.abs(SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees() - SubsystemManager.getInstance().getVision().getNoteCamera().getNoteYaw()) > 1.5){
                            Rotation2d relativeNoteAngle = Rotation2d.fromDegrees(centeringAngle);
                            SubsystemManager.getInstance().getDrivetrain().setSnapAngle(relativeNoteAngle);
                        }
                    }else{
                        isNoteSnapping = false;
                        SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    }
                } else {
                    isNoteSnapping = false;
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                }

                if(intakeProximity){

                    if(Double.isNaN(noteIntaked)){
                        noteIntaked = Timer.getFPGATimestamp();
                    } 
                    
                }
                if(Math.abs(Timer.getFPGATimestamp() - noteIntaked) >= 0.02 && intakeMotor.getTorqueCurrent().getValueAsDouble() > 15){
                    intakeMotor.stopMotor();
                    IO.getInstance().getController().rumble(1);
                    IO.getInstance().getController(XboxControllerName.AUX).rumble(1);

                    if(isNoteSnapping) {
                        SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                        isNoteSnapping = false;
                    }
                    state = SubsystemManager.getInstance().disableTransfer()? IntakeState.RETRACTED : IntakeState.TRANSFER;
                    noteIntaked = Double.NaN;
                }
            }
            case TRANSFER -> {
                if(isNoteSnapping) {
                    isNoteSnapping = false;
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                }
                targetRadians = IntakePosition.FEED.getRadians();
                intakeMotor.stopMotor();
                SubsystemManager.getInstance().getDrivetrain().setFieldOriented(true);

                // Transfer criteria for Feed state
                if(Math.abs(getAngle().getRadians() - targetRadians) < PIVOT_TOLERANCE) {
                    state = IntakeState.FEED;
                    feedStartSeconds = Timer.getFPGATimestamp();
                    SubsystemManager.getInstance().getShooter().setState(ShooterState.FEEDING);

                    IO.getInstance().getController().rumble(0);
                    IO.getInstance().getController(XboxControllerName.AUX).rumble(0);
                }
            }
            case FEED -> {
                if(isNoteSnapping) {
                    isNoteSnapping = false;
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    SubsystemManager.getInstance().getDrivetrain().disableNoteHeadingCorrection();
                }

                //Boop.
                targetRadians = (Timer.getFPGATimestamp() - feedStartSeconds > 0.5)? IntakePosition.SHOOTING.getRadians() : IntakePosition.FEED.getRadians();
                if(Timer.getFPGATimestamp() - feedStartSeconds > FEED_DELAY) {
                    intakeMotor.set(FEED_SPEED);
                }
            }
            case SHOOTING -> {
                targetRadians = IntakePosition.SHOOTING.getRadians();
                intakeMotor.stopMotor();
            }
            case EJECT -> {
                intakeMotor.set(EJECT_SPEED);
            }
            case EXTEND -> {
                targetRadians = IntakePosition.DEPLOYED.getRadians();
            }
        }

        if(!DriverStation.isTest() && !Double.isNaN(targetRadians)) {
            pivotMotor.setVoltage(pivotFF.calculate(getAngle().getRadians(), 0) + pivotPID.calculate(getAngle().getRadians(), targetRadians));
        } else {
            pivotMotor.setVoltage(0);
        }
        
    }

    public void setIntakeRollersDisabled(boolean disabled) {
        disableIntakeRollers = disabled;
    }

    public void setCenteringNote(boolean isCentering){
        centeringNote = isCentering;
    }

    /**
     * Set the state of the intake
     * 
     * @param state The state to set
     */
    public void setState(IntakeState state) {
        this.state = state;
    }

    /**
     * @return The current state of the intake
     */
    public IntakeState getState() {
        return state;
    }

    /**
     * Reset the ProfiledPidController of the Pivot Motor 
     */
    public void resetPivotPid() {
        pivotPID.reset(getAngle().getRadians());
    }

    /**
     * @return True if the pivot motor is within tolerance of its setpoint
     */
    public boolean isAtSetpoint() {
        return pivotPID.atSetpoint();
    }

    /**
     * @return the current pivot angle of the intake
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble() * MOTOR_SHAFT_TO_PIVOT_MECHANISM);
    }
    
    /**
     * @return The current velocity of the pivot motor in radians per second
     */
    public double getPivotVelocity() {
        return pivotMotor.getVelocity().getValueAsDouble() * MOTOR_SHAFT_TO_PIVOT_MECHANISM * 2 * Math.PI;
    }

    /**
     * Reset the pivot angle to a set position.
     * 
     * @param angle
     */
    public void zeroAngle(Rotation2d angle) {
        pivotMotor.setPosition(angle.getRotations() / MOTOR_SHAFT_TO_PIVOT_MECHANISM);
        resetPivotPid();
    }

    public boolean hasNote() {
        return false;
    }
 
    @Override
    public void sysId_driveVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public double sysId_getPosition() {
        return getAngle().getRadians();
    }

    @Override
    public double sysId_getVelocity() {
        return getPivotVelocity();
    }

    @Override
    public double sysId_getVoltage() {
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Used to represent the current state of the intake
     */
    public enum IntakeState {
        /**
         * Intake is raised and idle.
         */
        RETRACTED,
        /**
         * Intake is currently descending into the deployed position, intake wheels are running.
         */
        DEPLOYING,
        /**
         * Intake has been fully deployed and floating, wheels are spinning and intake is waiting for a note.
         */
        DEPLOYED,
        /**
         * Intake has a note and is rising to transfer position.
         */
        TRANSFER,
        /**
         * Intake is feeding the note into the shooter mechanism.
         */
        FEED,
        SHOOTING,
        /**
         * Intake is ejecting the note out of the bot.
         */
        EJECT, 
        /**
         * Intake is down while climbing
         */
        EXTEND,
    }

    /**
     * Key travel locations for the intake
     */
    public enum IntakePosition {
        DEPLOYED(-17.8),
        RETRACTED(94),
        FEED(90),
        SHOOTING(73),
        ZERO_POSITION(96.843);

        private Rotation2d angle;

        private IntakePosition(double angleDegrees) {
            angle = Rotation2d.fromDegrees(angleDegrees);
        }

        public double getRadians() {
            return angle.getRadians();
        }
    }
}
