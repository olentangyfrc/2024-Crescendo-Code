package frc.robot.subsystems.shooter;

import java.util.TimerTask;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSbTab;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.deflector.Deflector.DeflectorPosition;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.ShooterUtils.ShootingParams;
import frc.robot.util.filters.MeanFilter;
import frc.robot.util.vector.Vector2d;

public class Shooter extends SubsystemBase {
    private static final double ANGLE_OFFSET_DEGREES = -176.468;

    private static final double SHOOTER_GEAR_RATIO = 12.0/15.0;
    private static final double ANGLE_GEAR_RATIO = 108.0;
    
    private static final double MAX_ANGLE_VELOCITY = 9999.0, MAX_ANGLE_ACCELERATION = 1000.0;
    private static final double ANGLE_TOLERANCE = 0.3; // Degrees
    private static final double VELOCITY_TOLERANCE = 3;
    
    private static final int BEAM_BREAK_ID = 1;
    
    private static final double INDEXER_INTAKE_SPEED = -0.15;
    private static final double INDEXER_INTAKE_SLOW_SPEED = -0.05;
    private static final double INDEXER_SPEED = 0.7;
    private static final double INDEXER_FEED_SPEED = 0.4;
    
    private static final double FEEDING_ANGLE = 5.0;
    private static final double AMP_SHOOTING_ANGLE = 46.7;
    private static final double AMP_TOP_SHOOTING_VELOCITY = 19.5;
    private static final double AMP_BOTTOM_SHOOTING_VELOCITY = 19.5;
    private static final double SHOOTER_INTAKE_VELOCITY = -8.5;

    private static final double SPEAKER_SHOOTING_SECONDS = 0.7;
    private static final double AMP_SHOOTING_SECONDS = 0.7;
    private static final double SHOOTER_INTAKE_DELAY = 0.14;

    public static final double MAX_PIVOT_ANGLE = 52.3;
    public static final double MIN_PIVOT_ANGLE = -10;
    public static final double SHOOTER_INTAKE_ANGLE = 50.0;

    public static final double FLYWHEEL_IDLE_VELOCITY = 20;

    private static final double AIMING_OFFSET_DEGREES = 0;
    
    private TalonFX angleMotor;
    private TalonFX topShooterMotor;
    private TalonFX bottomShooterMotor;
    private CANSparkMax indexerMotor;

    private DigitalInput beamBreak;
    private DutyCycleEncoder pivotAbsoluteEncoder;
    private MeanFilter pivotCalibrationFilter;
    private boolean isCalibrated = false;

    
    //Revs per second of the fly wheel
    private SimpleMotorFeedforward flywheelFF;
    
    
    private ProfiledPIDController anglePID;
    private ArmFeedforward angleFF;

    private GenericEntry useShuffleboardValues;
    private GenericEntry shooterAngleEntry;
    private GenericEntry shooterSpeedEntry;

    private GenericEntry disableTransferEntry;

    public ShooterState shooterState;

    private double shootingStartSeconds = Double.NaN;
    private double shooterIntakeStartSeconds = Double.NaN;

    private double targetTopShootingVelocity = Double.NaN;
    private double targetBottomShootingVelocity = Double.NaN;
    private double targetPivotAngle = Double.NaN;

    private boolean hasFollowedPath = false;

    private Command ampDriveCommand;

    private java.util.Timer calibrationTimer = new java.util.Timer();

    private boolean ignoreNextDelay = false;

    /**
     * Initializes PID for both the shooter velocity and the angle change and
     * initalizes motors + limit switch
     */
    public Shooter(int angleMotorId, int topShooterId, int bottomShooterId, int indexerId) {
        
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        angleMotor = new TalonFX(angleMotorId, "*");
        angleMotor.getConfigurator().apply(angleConfig);
        
        indexerMotor = new CANSparkMax(indexerId, MotorType.kBrushless);
        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setInverted(true);
        indexerMotor.setIdleMode(IdleMode.kBrake);
        
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        topShooterMotor = new TalonFX(topShooterId, "*");
        topShooterMotor.getConfigurator().apply(shooterConfig);
        
        bottomShooterMotor = new TalonFX(bottomShooterId, "*");
        bottomShooterMotor.getConfigurator().apply(shooterConfig);
        
        topShooterMotor.setPosition(0.0);
        bottomShooterMotor.setPosition(0.0);
        
        beamBreak = new DigitalInput(BEAM_BREAK_ID);
        
        angleFF = new ArmFeedforward(0,0,0.03);
        anglePID = new ProfiledPIDController(0.6, 3, 0, new Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION));
        anglePID.setIZone(0.7);
        anglePID.setTolerance(ANGLE_TOLERANCE);
        
        pivotAbsoluteEncoder = new DutyCycleEncoder(0);
        pivotCalibrationFilter = new MeanFilter(40);

        calibrationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if(isCalibrated) {
                    return;
                }
                double avgRad = pivotCalibrationFilter.calculate(getAbsoluteShooterAngle().getRadians());

                if(pivotCalibrationFilter.getNumVals() >= pivotCalibrationFilter.getWindowSize()) {
                    zeroAngle(new Rotation2d(avgRad));
                    isCalibrated = true;
                }
            }
        }, 0, 2);

        flywheelFF = new SimpleMotorFeedforward(0.0851881, 0.15583);
        
        setState(ShooterState.IDLE);
        
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        tab.addString("Shooter State", () -> shooterState.toString());
        tab.addNumber("Shooter Angle", () -> getShooterAngle().getDegrees());
        tab.addNumber("Shooter Speed", (() -> topShooterMotor.getVelocity().getValueAsDouble() * SHOOTER_GEAR_RATIO));
        tab.addNumber("Angle setpoint", () -> anglePID.getSetpoint().position);
        tab.addNumber("Velocity setpoint", () -> targetTopShootingVelocity);
        tab.addNumber("Pivot Velocity", () -> angleMotor.getVelocity().getValueAsDouble() / ANGLE_GEAR_RATIO * 360);

        useShuffleboardValues = tab.add("Use SB Values", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        shooterAngleEntry = Shuffleboard.getTab("Shooter").add("Target Shooter Angle", 0.0).getEntry();
        shooterSpeedEntry = tab.add("Target Shooter Speed", 0.0).getEntry(); 

        disableTransferEntry = tab.add("Disable Transfer", false).getEntry();

        tab.addNumber("FeedForward", () -> angleFF.calculate(getShooterAngle().getRadians(), 0));
        tab.addBoolean("Beam break", () -> beamBreak.get());
        tab.addBoolean("Is Calibrated", () -> isCalibrated);
    }

    private double movingShootingStartSeconds = Double.NaN;

    private boolean noAngleSnapPls = false;
    private boolean shouldPreAlignNext = false;

    /**
     * Updates motor voltages based on feedForward. Velocity in rps and angle is in degrees
     */
    public void periodic() {
        if(!isCalibrated) {
            angleMotor.stopMotor();
            return;
        } else if(calibrationTimer != null) {
            calibrationTimer.cancel();
            calibrationTimer = null;
        }

        if(!DriveSbTab.getInstance().isShooterEnabled()) {
            angleMotor.stopMotor();
            indexerMotor.stopMotor();
            topShooterMotor.stopMotor();

            return;
            
        }
        
        SubsystemManager.getInstance().setDisableTransfer(disableTransferEntry.getBoolean(false));

        double targetPivot = Double.NaN;
        double targetTopVel = Double.NaN;
        double targetBottomVel = Double.NaN;

        SmartDashboard.putNumber("Top Shooter", topShooterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Shooter", topShooterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Pivot", topShooterMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer", indexerMotor.getOutputCurrent());

        switch(shooterState) {
            case FEEDING -> {
                indexerMotor.set(INDEXER_FEED_SPEED);
                targetPivot = FEEDING_ANGLE;

                if (isBeamBroken()) {
                    if(DriverStation.isAutonomous() && shouldPreAlignNext) {
                        setState(ShooterState.SPEAKER_OPERATOR_ALIGNMENT);
                    } else {
                        setState(ShooterState.HOLDING);
                    }
                    indexerMotor.stopMotor();
                    SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                }

                if(DriverStation.isAutonomous()) {
                    setTargetShooterVelocity(75);
                    targetTopVel = targetTopShootingVelocity;
                }
            }
            case HOLDING -> {
                indexerMotor.stopMotor();
                targetPivot = FEEDING_ANGLE;

                if(DriverStation.isAutonomous()) {
                    setTargetShooterVelocity(75);
                    targetTopVel = targetTopShootingVelocity;
                    targetBottomVel = targetBottomShootingVelocity;
                }
            }
            case SPEAKER_ALIGNMENT -> {
                indexerMotor.stopMotor();

                Translation2d translationToSpeaker = SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation().minus(ShooterUtils.getSpeakerLoc());

                double distance = translationToSpeaker.getNorm();
                setTargetShooterAngle(ShooterUtils.getShooterAngle(distance));
                setTargetShooterVelocity(80);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(translationToSpeaker.getAngle().minus(Rotation2d.fromDegrees(0)));
            }
            case SPEAKER_SHOOTING -> {
                Translation2d translationToSpeaker = SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation().minus(ShooterUtils.getSpeakerLoc());
                double distance = translationToSpeaker.getNorm();

                setTargetShooterAngle(ShooterUtils.getShooterAngle(distance));
                setTargetShooterVelocity(80);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(translationToSpeaker.getAngle().minus(Rotation2d.fromDegrees(0)));

                if (Double.isNaN(shootingStartSeconds) && SubsystemManager.getInstance().getDrivetrain().isAtSnapAngle() && isAtTargetAngle() && isAtTargetVelocity()) {
                    shootingStartSeconds = Timer.getFPGATimestamp();
                    indexerMotor.set(INDEXER_SPEED);
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                    if(SubsystemManager.getInstance().getIntake().getState() == IntakeState.SHOOTING && !DriverStation.isAutonomous()) {
                        SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                    }
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    setState(ShooterState.IDLE);
                    shootingStartSeconds = Double.NaN;
                }
            }
            case SPEAKER_OPERATOR_ALIGNMENT -> {
                indexerMotor.stopMotor();
                
                setTargetShooterVelocity(80);
                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = FEEDING_ANGLE;
                
                if(DriverStation.isAutonomous()) {
                    ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();
                    ShootingParams shooterParams = ShooterUtils.getShootingWhileMovingParams(SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation(), new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));

                    setTargetShooterAngle(shooterParams.shooterPivotDegrees);
                    targetPivot = targetPivotAngle;

                    if(!noAngleSnapPls) {
                        SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees).plus(Rotation2d.fromDegrees(AIMING_OFFSET_DEGREES)));
                    }
                }
            }
            case SPEAKER_MOVING_ALIGNMENT -> {
                indexerMotor.stopMotor();
                ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();

                ShootingParams shooterParams = ShooterUtils.getShootingWhileMovingParams(SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation(), new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));

                setTargetShooterVelocity(80);
                setTargetShooterAngle(shooterParams.shooterPivotDegrees);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                if(!noAngleSnapPls) {
                    SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees).plus(Rotation2d.fromDegrees(AIMING_OFFSET_DEGREES)));
                }
            }
            case SPEAKER_MOVING_SHOOTING -> {
                if(Double.isNaN(movingShootingStartSeconds) && SubsystemManager.getInstance().getDrivetrain().isAtSnapAngle()) {
                    movingShootingStartSeconds = Timer.getFPGATimestamp();
                }

                ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();
                ShootingParams shooterParams = ShooterUtils.getShootingWhileMovingParams(SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation(), new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));

                setTargetShooterVelocity(80);
                setTargetShooterAngle(shooterParams.shooterPivotDegrees);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                if(!noAngleSnapPls) {
                    SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees).plus(Rotation2d.fromDegrees(AIMING_OFFSET_DEGREES)));
                }

                SubsystemManager.getInstance().getIntake().setState(IntakeState.SHOOTING);
                SmartDashboard.putBoolean("Snap Angle", SubsystemManager.getInstance().getDrivetrain().isAtSnapAngle());
                SmartDashboard.putBoolean("Pivot Angle", isAtTargetAngle());
                SmartDashboard.putBoolean("Shooter Velocity", isAtTargetVelocity());
                if (!Double.isNaN(movingShootingStartSeconds)) {
                    if ((ignoreNextDelay || Timer.getFPGATimestamp() - movingShootingStartSeconds > (0.5 + ((DriverStation.isTeleop()? 0.5 : 0)))) &&  Double.isNaN(shootingStartSeconds) && (noAngleSnapPls || SubsystemManager.getInstance().getDrivetrain().isAtSnapAngle()) && isAtTargetAngle() && isAtTargetVelocity()) {

                        shootingStartSeconds = Timer.getFPGATimestamp();
                        indexerMotor.set(INDEXER_SPEED);
                        ignoreNextDelay = false;
                    } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                        if(SubsystemManager.getInstance().getIntake().getState() == IntakeState.SHOOTING) {
                            SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                        }
                        SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                        setState(ShooterState.IDLE);
                        shootingStartSeconds = Double.NaN;
                    }
                }
            }
            case AMP_ALIGNMENT -> {
                if(!hasFollowedPath && (ampDriveCommand == null || !ampDriveCommand.isScheduled()) && DriveSbTab.getInstance().isAmpDriveEnabled()) {
                    Pose2d currentLoc = SubsystemManager.getInstance().getDrivetrain().getLocation();
                    ChassisSpeeds fieldRelChassisSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();
                    ampDriveCommand = SubsystemManager.getInstance().getAutoManager().getDriveToAmpCommand(currentLoc, fieldRelChassisSpeeds);
                    ampDriveCommand.schedule();
                    hasFollowedPath = true;
                }

                indexerMotor.stopMotor();

                setTargetShooterAngle(AMP_SHOOTING_ANGLE);
                setTargetShooterVelocity(AMP_TOP_SHOOTING_VELOCITY, AMP_BOTTOM_SHOOTING_VELOCITY);
                SubsystemManager.getInstance().getDeflector().setTargetPosition(DeflectorPosition.PREP);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(-90));
            }
            case AMP_SHOOTING -> {
                if(ampDriveCommand != null) {
                    stopDrivingToAmp();
                }

                setTargetShooterAngle(AMP_SHOOTING_ANGLE);
                setTargetShooterVelocity(AMP_TOP_SHOOTING_VELOCITY, AMP_BOTTOM_SHOOTING_VELOCITY);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(-90));
                SubsystemManager.getInstance().getDeflector().setTargetPosition(DeflectorPosition.AMP);
                SubsystemManager.getInstance().getIntake().setState(IntakeState.SHOOTING);

                if (Double.isNaN(shootingStartSeconds) && isAtTargetAngle() && isAtTargetVelocity() && (SubsystemManager.getInstance().getDeflector().isAtPosition())) {
                    shootingStartSeconds = Timer.getFPGATimestamp();
                    indexerMotor.set(INDEXER_SPEED);
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > AMP_SHOOTING_SECONDS) {
                    SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                    setState(ShooterState.IDLE);
                    shootingStartSeconds = Double.NaN;
                    SubsystemManager.getInstance().getDeflector().setTargetPosition(DeflectorPosition.STOWED);
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    if(SubsystemManager.getInstance().getIntake().getState() == IntakeState.SHOOTING) {
                        SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                    }
                }
            }
            case MANUAL_ALIGNMENT -> {
                indexerMotor.stopMotor();

                ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();

                ShootingParams shooterParams = ShooterUtils.getShootingWhileMovingParams(SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation(), new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));
                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees).plus(Rotation2d.fromDegrees(AIMING_OFFSET_DEGREES)));

                if(useShuffleboardValues.getBoolean(false)) {
                    setTargetShooterAngle(shooterAngleEntry.getDouble(0));
                    setTargetShooterVelocity(shooterSpeedEntry.getDouble(0));
                }

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;
            }
            case MANUAL_SHOOTING -> {
                if(useShuffleboardValues.getBoolean(false)) {
                    setTargetShooterAngle(shooterAngleEntry.getDouble(0));
                    setTargetShooterVelocity(shooterSpeedEntry.getDouble(0));
                }

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getIntake().setState(IntakeState.SHOOTING);

                if (Double.isNaN(shootingStartSeconds) && isAtTargetAngle() && isAtTargetVelocity()) {
                    shootingStartSeconds = Timer.getFPGATimestamp();
                    indexerMotor.set(INDEXER_SPEED);
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                    if(SubsystemManager.getInstance().getIntake().getState() == IntakeState.SHOOTING) {
                        SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                    }
                    setState(ShooterState.IDLE);
                    shootingStartSeconds = Double.NaN;
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                }
            }
            case PASSING_ALIGNMENT -> {
                Translation2d loc = SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation();
                ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();
                Vector2d botVelocity = new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
                double distance = loc.getDistance(ShooterUtils.getPassingLoc());

                indexerMotor.stopMotor();
                ShootingParams shooterParams = ShooterUtils.getPassingWhileMovingParams(loc, botVelocity, ShooterUtils.getPassingSpeed(distance));

                //Shooter velocity is dependent on shooter velocity, so this line iterates a second time for a more accurate result.
                setTargetShooterVelocity(ShooterUtils.getPassingSpeed(loc.minus(ShooterUtils.getPassingPosition(loc, botVelocity, 40)).getNorm()));
                setTargetShooterAngle(shooterParams.shooterPivotDegrees);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees));
            }
            case PASSING -> {
                Translation2d loc = SubsystemManager.getInstance().getDrivetrain().getLocation().getTranslation();
                ChassisSpeeds currentSpeeds = SubsystemManager.getInstance().getDrivetrain().getFieldRelChassisSpeeds();
                Vector2d botVelocity = new Vector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
                double distance = loc.getDistance(ShooterUtils.getPassingLoc());
                ShootingParams shooterParams = ShooterUtils.getPassingWhileMovingParams(loc, botVelocity, ShooterUtils.getPassingSpeed(distance));

                //Shooter velocity is dependent on shooter velocity, so this line iterates a second time for a more accurate result.
                setTargetShooterVelocity(ShooterUtils.getPassingSpeed(loc.minus(ShooterUtils.getPassingPosition(loc, botVelocity, 40)).getNorm()));
                setTargetShooterAngle(shooterParams.shooterPivotDegrees);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;
                
                SubsystemManager.getInstance().getIntake().setState(IntakeState.SHOOTING);
                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromDegrees(shooterParams.driveSnapDegrees));

                if (Double.isNaN(shootingStartSeconds)) {
                    if(SubsystemManager.getInstance().getDrivetrain().isAtSnapAngle() &&
                        isAtTargetAngle() && isAtTargetVelocity()) {

                        shootingStartSeconds = Timer.getFPGATimestamp();
                        indexerMotor.set(INDEXER_SPEED);
                    }
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                    if(SubsystemManager.getInstance().getIntake().getState() == IntakeState.SHOOTING) {
                        SubsystemManager.getInstance().getIntake().setState(IntakeState.RETRACTED);
                    }
                    SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    shooterState = ShooterState.IDLE;
                    shootingStartSeconds = Double.NaN;
                }
            }
            case SHOOTER_INTAKE -> {
                SubsystemManager.getInstance().getIntake().setState(IntakeState.SHOOTING);
                targetPivot = SHOOTER_INTAKE_ANGLE;

                SubsystemManager.getInstance().getDrivetrain().setSnapAngle(Rotation2d.fromRadians((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? 2.0915 : 1.0465));

                if (!isBeamBroken()) {
                    targetTopVel = SHOOTER_INTAKE_VELOCITY;
                    targetBottomVel = SHOOTER_INTAKE_VELOCITY;
                    indexerMotor.set(INDEXER_INTAKE_SLOW_SPEED);
                } else {
                    targetBottomVel = 0;
                    targetTopVel = 0;
                    indexerMotor.set(INDEXER_INTAKE_SPEED);
                    if (Double.isNaN(shooterIntakeStartSeconds)) {
                        shooterIntakeStartSeconds = Timer.getFPGATimestamp();
                    }
                    if (Timer.getFPGATimestamp() - shooterIntakeStartSeconds > SHOOTER_INTAKE_DELAY) {
                        indexerMotor.stopMotor();
                        setState(ShooterState.HOLDING);
                        shooterIntakeStartSeconds = Double.NaN;

                        SubsystemManager.getInstance().getDrivetrain().removeSnapAngle();
                    }
                }
                
            }
            case EJECT -> {

                setTargetShooterVelocity(60);
                setTargetShooterAngle(0);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                if (Double.isNaN(shootingStartSeconds) && isGreaterThanTargetVelocity() && isAtTargetAngle()) {
                    shootingStartSeconds = Timer.getFPGATimestamp();
                    indexerMotor.set(0.4);
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                    setState(ShooterState.IDLE);
                    shootingStartSeconds = Double.NaN;
                }
            }
            case WEAK_EJECT -> {

                setTargetShooterVelocity(20);
                setTargetShooterAngle(0);

                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
                targetPivot = targetPivotAngle;

                if (Double.isNaN(shootingStartSeconds) && isGreaterThanTargetVelocity() && isAtTargetAngle()) {
                    shootingStartSeconds = Timer.getFPGATimestamp();
                    indexerMotor.set(0.4);
                } else if (Timer.getFPGATimestamp() - shootingStartSeconds > SPEAKER_SHOOTING_SECONDS) {
                    setState(ShooterState.IDLE);
                    shootingStartSeconds = Double.NaN;
                }
            }
            case IDLE -> {
                indexerMotor.stopMotor();
                targetPivot = 0;
                targetPivot = FEEDING_ANGLE;
                if(DriverStation.isAutonomous()) {
                    setTargetShooterVelocity(65);
                } else {
                    setTargetShooterVelocity(0.0);
                }
                targetTopVel = targetTopShootingVelocity;
                targetBottomVel = targetBottomShootingVelocity;
            }
        }
        
        if(!Double.isNaN(targetPivot)) {
            targetPivot = MathUtil.clamp(targetPivot, MIN_PIVOT_ANGLE, MAX_PIVOT_ANGLE);
        }

        if(!DriverStation.isTest() && !Double.isNaN(targetPivot)) {
            anglePID.setGoal(new State(targetPivot, 0));

            angleMotor.setVoltage(
                anglePID.calculate(getShooterAngle().getDegrees()) + 
                angleFF.calculate(getShooterAngle().getRadians(), anglePID.getSetpoint().velocity)
            );
        } else {
            angleMotor.setVoltage(0);
        }

        if(!Double.isNaN(targetTopVel) && !DriverStation.isTest()) {
            topShooterMotor.setControl(new VoltageOut(flywheelFF.calculate(targetTopVel), false, false, false, false));
            bottomShooterMotor.setControl(new VoltageOut(flywheelFF.calculate(targetBottomVel), false, false, false, false));

        } else {
            topShooterMotor.setControl(new VoltageOut(0, false, false, false, false));
            bottomShooterMotor.setControl(new VoltageOut(0, false, false, false, false));
        }

    }

    /**
     * Zeroes the shooter arm on startup or on the click of a button
     * @param angle
     */
    public void zeroAngle(Rotation2d angle) {
        angleMotor.setPosition(angle.getRotations() * ANGLE_GEAR_RATIO);
        resetAnglePid();
    }

    public void plsEnableAngleSnap() {
        noAngleSnapPls = false;
    }

    public void plsDisableAngleSnap() {
        noAngleSnapPls = true;
    }

    public void setIgnoreNextVisionDelay(boolean shouldIgnore) {
        ignoreNextDelay = true;
    }

    /**
     * Resets the angle pid based off the shooter arm position
     */
    public void resetAnglePid() {
        anglePID.reset(getShooterAngle().getDegrees());
    }

    public void stopDrivingToAmp() {
        if(ampDriveCommand != null) {
            ampDriveCommand.cancel();
            ampDriveCommand = null;
        }
    }

    /**
     * Sets the angle in degrees above horizontal of the shooter target PID value
     * @param angle
     */
    public void setTargetShooterAngle(double angle) {
        targetPivotAngle = MathUtil.clamp(angle, MIN_PIVOT_ANGLE, MAX_PIVOT_ANGLE);
    }

    /**
     * Sets target velocity in rps for shooter wheels
     * @param velocity
     */
    public void setTargetShooterVelocity(double velocity) {
        targetTopShootingVelocity = velocity;
        targetBottomShootingVelocity = velocity;
    }
    /**
     * Sets target velocity in rps for shooter wheels
     * @param velocity
     */
    public void setTargetShooterVelocity(double topVelocity, double bottomVelocity) {
        targetTopShootingVelocity = topVelocity;
        targetBottomShootingVelocity = bottomVelocity;
    }

    /**
     * Stops shooter motor
     */
    public void stopShooter() {
        targetTopShootingVelocity = Double.NaN;
        targetBottomShootingVelocity = Double.NaN;
        topShooterMotor.setVoltage(0);
    }

    /**
     * Starts the indexer motor
     */
    public void startIndexerMotor() {
        indexerMotor.set(INDEXER_SPEED);
    }

    /**
     * Stops the feed motor
     */
    public void stopIndexerMotor() {
        indexerMotor.stopMotor();
    }

    /**
     * @return whether or not shooter has reached target velocity
     */
    public boolean isAtTargetVelocity() {
        return Math.abs(getShooterSpeed() - targetTopShootingVelocity) <= VELOCITY_TOLERANCE;
    }

    public boolean isGreaterThanTargetVelocity() {
        return getShooterSpeed() >= targetTopShootingVelocity - VELOCITY_TOLERANCE;
    }

    /**
     * @return whether or not the shooter is at its target angle
     */
    public boolean isAtTargetAngle() {
        return Math.abs(getShooterAngle().getDegrees() - targetPivotAngle) < ANGLE_TOLERANCE;
    }

    /**
     * Gets shooter velocity in rps
     * @return
     */
    public double getShooterSpeed() {
        return topShooterMotor.getVelocity().getValueAsDouble() * SHOOTER_GEAR_RATIO;
    }

    /**
     * Gets shooter angle above parallel to ground in degrees
     * @return
     */
    public Rotation2d getShooterAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble() / ANGLE_GEAR_RATIO);
    }

    /**
     * Get the position from the absolute encoder.
     * 
     * This has lots of noise and should not be used for feedback control.
     * 
     * @return
     */
    public Rotation2d getAbsoluteShooterAngle() {
        return Rotation2d.fromRotations(-pivotAbsoluteEncoder.getAbsolutePosition() - (ANGLE_OFFSET_DEGREES / 360));
    }

    /**
     * Limit switch tells when a note has left the intake and is held by the feeding motors of the shooter.
     * @return
     */
    public boolean isBeamBroken() {
        return !beamBreak.get();
    }

    /**
     * Gets the max shooter pivot angle
     * @return double angle in degrees
     */
    public double getMaxShooterAngle(){
        return MAX_PIVOT_ANGLE;
    }

    /**
     * Gets the min shooter pivot angle
     * @return double angle in degrees
     */
    public double getMinShooterAngle(){
        return MIN_PIVOT_ANGLE;
    }

    public boolean shouldPreAlignNext() {
        return shouldPreAlignNext;
    }

    public void setShouldPreAlignNext(boolean shouldPreAlignNext) {
        this.shouldPreAlignNext = shouldPreAlignNext;
    }



    /**
     * List of shooter states
     */
    public enum ShooterState {
        FEEDING,
        HOLDING,
        SPEAKER_ALIGNMENT,
        SPEAKER_SHOOTING,
        SPEAKER_MOVING_ALIGNMENT,
        SPEAKER_OPERATOR_ALIGNMENT,
        SPEAKER_MOVING_SHOOTING,
        AMP_ALIGNMENT,
        AMP_SHOOTING,
        MANUAL_ALIGNMENT,
        MANUAL_SHOOTING,
        PASSING_ALIGNMENT,
        PASSING,
        SHOOTER_INTAKE,
        EJECT,
        WEAK_EJECT,
        IDLE
    }

    /**
     * Sets the shooter state
     * @param state
     */
    public void setState(ShooterState state) {
        shooterState = state;
        hasFollowedPath = false;
    }

    /**
     * Gets the shooter state
     * @return
     */
    public ShooterState getState() {
        return shooterState;
    }
    
}
