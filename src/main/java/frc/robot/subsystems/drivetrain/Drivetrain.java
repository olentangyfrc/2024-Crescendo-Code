package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSbTab;
import frc.robot.subsystems.drivetrain.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import frc.robot.subsystems.telemetry.OzoneImu;
import frc.robot.subsystems.telemetry.OzoneImu.GyroAxis;
import frc.robot.subsystems.telemetry.OzonePigeon2;

public class Drivetrain extends SubsystemBase {
  private static final String TAB_NAME = "Drivetrain";
  // Max linear speed in meters per second
  public static final double MAX_LINEAR_SPEED = 4.8;
  // Max angular speed in radians per second
  public static final double MAX_ANGULAR_SPEED = 8;

  public static final double SHOOTING_MAX_DRIVE_VELOCITY = 1; // m/s

  private double trackWidth;
  private double wheelBase;

  // private GenericEntry directVoltageToggle;
  // private GenericEntry directVoltageEntry;

  private SwerveModule[] swerveModules;
  private OzoneImu imu;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;

  private PIDController snapAnglePid;

  private boolean activeSnapEnabled = false;
  private boolean passiveSnapEnabled = false;
  private Rotation2d snapAngle;
  private Rotation2d passiveSnapAngle;
  private static final double PASSIVE_SNAP_DELAY_SECONDS = 0.4;
  private double lastAngularInputReceived = Timer.getFPGATimestamp();
  private static final double SNAP_TOL_DEGREES = 3;

  private boolean areWheelsLocked = false;
  private boolean isFieldOriented = true;
  private boolean useNoteHeadingCorrection = false;
  private Rotation2d relativeNoteAngle = new Rotation2d();

  private boolean lockVelocity = false;
  private ChassisSpeeds lockedSpeeds = null;

  // Shuffleboard Configuration
  private boolean showModuleAngles = true;
  private boolean showModuleVelocities = true;

  private Field2d field; // Used for displaying robot position to Glass dashboard

  private PIDConstants translationConstants;
  private PIDConstants rotationConstants;

  private DriveSignal signal = new DriveSignal(new ChassisSpeeds(), true);

  private StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  
  StructPublisher<Pose3d> odo_publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Odometry", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> odo_arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Odometry Pose Array", Pose3d.struct).publish();

  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  /**
   * Initialize a Swerve Drivetrain with modules
   * 
   * @param swerveModules Array of modules ordered according to: FL, FR, BL, BR
   * @param pigeonCanId CAN ID of pigeon 2.0
   * @param trackWidth Distance from center of wheel to center of wheel across the <b>front</b> of the bot in meters
   * @param wheelBase Distance from center of wheel to center of wheel across the <b>side</b> of the bot in meters
   */
  public void init(SwerveModule[] swerveModules, int pigeonCanId, double trackWidth, double wheelBase, PIDConstants translationConstants, PIDConstants rotationConstants, boolean useCanivore) {
    this.swerveModules = swerveModules;
    if(useCanivore) {
      imu = new OzonePigeon2(pigeonCanId, "*");
    } else {
      imu = new OzonePigeon2(pigeonCanId);
    }
    this.trackWidth = trackWidth;
    this.wheelBase = wheelBase;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;

    // Pass in the coordinates of each wheel relative to the center of the bot.
    kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2, trackWidth / 2), // FL
      new Translation2d(wheelBase / 2, -trackWidth / 2), // FR
      new Translation2d(-wheelBase / 2, trackWidth / 2), // BL
      new Translation2d(-wheelBase / 2, -trackWidth / 2) // BR
    );

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, imu.getRotation2d(GyroAxis.YAW), getModulePositions(), new Pose2d());

    snapAnglePid = new PIDController(4, 0, 0);
    snapAnglePid.enableContinuousInput(0.0, 2 * Math.PI);
    snapAnglePid.setTolerance(3.0 / 360 * (2 * Math.PI));

    if(showModuleAngles) {
      Shuffleboard.getTab(TAB_NAME).addNumber("FL Angle", () -> swerveModules[0].getAngle().getDegrees()).withPosition(0, 0);
      Shuffleboard.getTab(TAB_NAME).addNumber("FR Angle", () -> swerveModules[1].getAngle().getDegrees()).withPosition(1, 0);
      Shuffleboard.getTab(TAB_NAME).addNumber("BL Angle", () -> swerveModules[2].getAngle().getDegrees()).withPosition(2, 0);
      Shuffleboard.getTab(TAB_NAME).addNumber("BR Angle", () -> swerveModules[3].getAngle().getDegrees()).withPosition(3, 0);
    }

    if(showModuleVelocities) {
      Shuffleboard.getTab(TAB_NAME).addNumber("FL Velocity", () -> swerveModules[0].getVelocity()).withPosition(0, 1);
      Shuffleboard.getTab(TAB_NAME).addNumber("FR Velocity", () -> swerveModules[1].getVelocity()).withPosition(1, 1);
      Shuffleboard.getTab(TAB_NAME).addNumber("BL Velocity", () -> swerveModules[2].getVelocity()).withPosition(2, 1);
      Shuffleboard.getTab(TAB_NAME).addNumber("BR Velocity", () -> swerveModules[3].getVelocity()).withPosition(3, 1);

      Shuffleboard.getTab(TAB_NAME).addNumber("FL Target Velocity", () -> swerveModules[0].getTargetState().speedMetersPerSecond).withPosition(0, 2);
      Shuffleboard.getTab(TAB_NAME).addNumber("FR Target Velocity", () -> swerveModules[1].getTargetState().speedMetersPerSecond).withPosition(1, 2);
      Shuffleboard.getTab(TAB_NAME).addNumber("BL Target Velocity", () -> swerveModules[2].getTargetState().speedMetersPerSecond).withPosition(2, 2);
      Shuffleboard.getTab(TAB_NAME).addNumber("BR Target Velocity", () -> swerveModules[3].getTargetState().speedMetersPerSecond).withPosition(3, 2);
    }

    Shuffleboard.getTab(TAB_NAME).addBoolean("Is Snap On", this::isActiveSnapEnabled);

    Shuffleboard.getTab(TAB_NAME).addNumber("Gyro Angle Dial", () -> -imu.getAngle(GyroAxis.YAW)).withWidget(BuiltInWidgets.kGyro).withPosition(4, 0);
    Shuffleboard.getTab(TAB_NAME).addNumber("Gyro Angle", () -> imu.getAngle(GyroAxis.YAW));
    Shuffleboard.getTab(TAB_NAME).addNumber("Snap Angle", () -> activeSnapEnabled? snapAngle.getDegrees() : 0);

    Shuffleboard.getTab(TAB_NAME).addNumber("Locked Velocity", () -> (lockedSpeeds == null)? 0 : Math.hypot(lockedSpeeds.vxMetersPerSecond, lockedSpeeds.vyMetersPerSecond));

    field = new Field2d();
    Shuffleboard.getTab(TAB_NAME).add(field);

    Shuffleboard.getTab(TAB_NAME).add(snapAnglePid);

    setDefaultCommand(new DriveCommand(this));
  }

  @Override
  public void periodic() {
    statePublisher.set(getModuleStates());
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), imu.getRotation2d(GyroAxis.YAW), getModulePositions());
    field.setRobotPose(getLocation());

    drive(signal.getSpeeds(), signal.isFieldOriented());
  }

  public void setSignal(DriveSignal signal) {
    this.signal = signal;
  }

  /**
   * Drive the robot according to a given Chassis speeds
   * 
   * @param speeds The target drivetrain speeds
   * @param isFieldOriented True if the robot should drive in field oriented, else bot oriented
   */
  private void drive(ChassisSpeeds speeds, boolean isFieldOriented) {
    
    if(lockVelocity) {
      speeds = lockedSpeeds;
    }

    if(Math.abs(speeds.omegaRadiansPerSecond) > 1e-5) {
      lastAngularInputReceived = Timer.getFPGATimestamp();
    }

    Rotation2d targetAngle = null;

    if(activeSnapEnabled) {
      targetAngle = snapAngle;
    } else {
      // Passive snap logic
      if(!DriverStation.isAutonomous() && Timer.getFPGATimestamp() - lastAngularInputReceived > PASSIVE_SNAP_DELAY_SECONDS) {
        // Passive snap should be enabled
        if(!passiveSnapEnabled && !DriverStation.isAutonomous()) {
          enablePassiveSnap();
        }
        
        targetAngle = passiveSnapAngle;
      } else if(passiveSnapEnabled) {
        disablePassiveSnap();
      }
    }

    Pose2d loc = poseEstimator.getEstimatedPosition();
    
    if(targetAngle != null) {
      speeds.omegaRadiansPerSecond = snapAnglePid.calculate(loc.getRotation().getRadians(), targetAngle.getRadians());
    }

    if(useNoteHeadingCorrection) {
      double driveMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      speeds.vxMetersPerSecond = driveMagnitude * relativeNoteAngle.getCos();
      speeds.vyMetersPerSecond = driveMagnitude * relativeNoteAngle.getSin();

    } else if(isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, loc.getRotation());
    }
    
    SwerveModuleState[] targetStates;

    if(areWheelsLocked) {
      targetStates = new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(145)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(225))
      };
    } else {
      targetStates = kinematics.toSwerveModuleStates(speeds);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_LINEAR_SPEED);

    // Apply target states
    for(int i = 0; i < 4; i++) {
      swerveModules[i].setTargetState(targetStates[i]);
      swerveModules[i].update();
    }
  }

  private void enablePassiveSnap() {
    passiveSnapEnabled = true;
    passiveSnapAngle = poseEstimator.getEstimatedPosition().getRotation();
  }

  public void disablePassiveSnap() {
    passiveSnapEnabled = false;
  }

  public void enableNoteHeadingCorrection(Rotation2d heading) {
    useNoteHeadingCorrection = true;
    relativeNoteAngle = heading;
  }

  public void disableNoteHeadingCorrection() {
    useNoteHeadingCorrection = false;
  }

  /**
   * @return the current positions of the modules
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(swerveModules[0].getDriveDistance(), swerveModules[0].getAngle()),
      new SwerveModulePosition(swerveModules[1].getDriveDistance(), swerveModules[1].getAngle()),
      new SwerveModulePosition(swerveModules[2].getDriveDistance(), swerveModules[2].getAngle()),
      new SwerveModulePosition(swerveModules[3].getDriveDistance(), swerveModules[3].getAngle())
    };
  }

  /**
   * @return the current states of the modules
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      swerveModules[0].getCurrentState(),
      swerveModules[1].getCurrentState(),
      swerveModules[2].getCurrentState(),
      swerveModules[3].getCurrentState(),
    };
  }

  public boolean isAtLockedSpeeds() {
    ChassisSpeeds currentVelocity = getSpeeds();

    return Math.abs(Math.hypot(lockedSpeeds.vxMetersPerSecond, lockedSpeeds.vyMetersPerSecond) - Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond)) < 0.2;
  }

  public boolean hasLockSpeed() {
    return lockVelocity;
  }

  /**
   * @return The current chassis speeds of the robot
   */
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelChassisSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), poseEstimator.getEstimatedPosition().getRotation());
  }

  /**
   * @return Get the length from center of wheel to center of wheel along the side of the robot in meters
   */
  public double getTrackWidth() {
    return trackWidth;
  }

  /**
   * @return Get the length from center of wheel to center of wheel along the front of the robot in meters
   */
  public double getWheelBase() {
    return wheelBase;
  }

  /**
   * Set the drivetrain to snap to a yaw angle
   * 
   * @param angle target angle
   */
  public void setSnapAngle(Rotation2d angle) {
    if(!DriveSbTab.getInstance().isDriveSnapEnabled()) {
      return;
    }

    disablePassiveSnap();
    snapAngle = angle;
    activeSnapEnabled = true;
    snapAnglePid.reset();
  }

  public void lockSpeeds(ChassisSpeeds speeds) {
    lockedSpeeds = speeds;
    lockVelocity = true;
  }

  public void removeLockedSpeeds() {
    lockedSpeeds = new ChassisSpeeds();
    lockVelocity = false;
  }

  /**
   * Disable active angle snapping
   */
  public void removeSnapAngle() {
    activeSnapEnabled = false;
    snapAngle = null;
  }

  /**
   * @return True if the robot is at it's active snap angle
   */
  public boolean isAtSnapAngle() {
    return (!DriveSbTab.getInstance().isDriveSnapEnabled()) ||  (activeSnapEnabled && Math.abs(poseEstimator.getEstimatedPosition().getRotation().getRadians() - snapAngle.getRadians()) < (SNAP_TOL_DEGREES / 360 * 2 * Math.PI));
  }

  public void stop() {
    for(SwerveModule module : swerveModules) {
      module.stop();
    }
  }

  public boolean isActiveSnapEnabled() {
    return activeSnapEnabled;
  }

  /**
   * @return the current estimated position of the robot
   */
  public Pose2d getLocation() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
    return poseEstimator;
  }

  /**
   * @param newLocation Set the estimated position to a new position
   */
  public void resetLocation(Pose2d newLocation) {
    poseEstimator.resetPosition(imu.getRotation2d(GyroAxis.YAW), getModulePositions(), newLocation);
  }

  public void zeroYawToAllianceForwards() {
    Rotation2d rot = Rotation2d.fromDegrees((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? 0 : 180);
    poseEstimator.resetPosition(imu.getRotation2d(GyroAxis.YAW), getModulePositions(), new Pose2d(getLocation().getTranslation(), rot));
  }

  public boolean isFieldOriented() {
    return isFieldOriented;
  }

  public void setFieldOriented(boolean isFieldOriented) {
    this.isFieldOriented = isFieldOriented;
  }

  public PIDConstants getTranslationConstants() {
    return translationConstants;
  }

  public PIDConstants getRotationConstants() {
    return rotationConstants;
  }

  public boolean areWheelsLocked() {
    return areWheelsLocked;
  }

  public void setWheelsLocked(boolean areLocked) {
    areWheelsLocked = areLocked;
  }

  public static class DriveSignal {
    private ChassisSpeeds speeds;
    private boolean fieldOriented;
    
    public DriveSignal(ChassisSpeeds speeds, boolean fieldOriented) {
      this.speeds = speeds;
      this.fieldOriented = fieldOriented;
    }

    public ChassisSpeeds getSpeeds() {
      return speeds;
    }

    public boolean isFieldOriented() {
      return fieldOriented;
    }
  }
}
