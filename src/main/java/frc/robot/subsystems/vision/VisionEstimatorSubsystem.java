package frc.robot.subsystems.vision;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSbTab;
import frc.robot.subsystems.SubsystemManager;

public class VisionEstimatorSubsystem extends SubsystemBase {

  private final Field2d field2d = new Field2d();
  StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Vision", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision Pose Array", Pose3d.struct).publish();

  private static final double xyStdev = 0.003;// uncertainty in meters per meter of distance to tag
  private static final double rotStdev = 50; // uncertainty in radians per meter of distance to tag

  private final TimeInterpolatableBuffer<Double> m_speedBuffer = TimeInterpolatableBuffer.createBuffer((a,b,t) -> {
    return a + (b-a) * t;
  }, 5.0);
  /**
   * Physical location of the right camera on the robot, relative to the center of
   * the robot. Unit is meters and degrees.
   */
  private static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
      new Translation3d(0.005461, -0.1224, 0.59868),
      new Rotation3d(0, (-19.4)/180*(Math.PI), (160.0 / 180 * Math.PI)));

  /**
   * Physical location of the cameras on the robot, relative to the center of
   * the robot.
   */
  private static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(
      new Translation3d(-0.3175, 0.006, .179),
      new Rotation3d(0,(-19.4)/360*2*(Math.PI),(200.0)/360*2*(Math.PI)));

  private final PhotonRunnable frontEstimator = new PhotonRunnable(new PhotonCamera("Right_Cam"),
  ROBOT_TO_FRONT_CAMERA);
  private final PhotonRunnable backEstimator = new PhotonRunnable(new PhotonCamera("Shooter_Cam"), ROBOT_TO_BACK_CAMERA);
  private final NotePhoton noteCamera = new NotePhoton(new PhotonCamera("HD_USB_Camera"));

  private final Notifier allNotifier = new Notifier(() -> {
    frontEstimator.run();
    noteCamera.run();
    backEstimator.run();
  });

  private final SwerveDrivePoseEstimator poseEstimator = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();


  // Boolean to disable vision
  public static boolean useVision = true;

  private double lastTime = 0;

  public void init() {
    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
  }

  private Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  private Vector<N3> confidenceCalculator(EstimatedRobotPose pose, double confidenceScale){

    // Combined inverse variance equals the sum of the inverse variances. This makes sense as our combined variance should be smaller
    // https://stats.stackexchange.com/questions/538622/product-of-two-normal-distributions-for-bayes-rule-is-not-product-of-normal-ou
    // Variance is squared standard deviation

    int tagCount = pose.targetsUsed.size();
    double min = 1.0;
    for(int i = 0; i < tagCount; i++){
      if((pose.targetsUsed.get(i).getFiducialId() == 14 || pose.targetsUsed.get(i).getFiducialId() == 13)){
        SmartDashboard.putNumber("Tag #" + pose.targetsUsed.get(i).getFiducialId() + " Ambiguity ", pose.targetsUsed.get(i).getPoseAmbiguity());
        if(pose.targetsUsed.get(i).getPoseAmbiguity() < min){
            min = pose.targetsUsed.get(i).getPoseAmbiguity();
        }
        if(Math.abs(min - pose.targetsUsed.get(i).getPoseAmbiguity())> 0.01){
          confidenceScale *= 4.0;
        }
      }
    }
    double sum = 0;
    for(int i = 0; i < tagCount; i++){
      sum += pose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getNorm();
    }
    double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
    double xyStd = xyStdev * stdScale * confidenceScale;
    double rotStd = rotStdev * stdScale * confidenceScale;
    
    return VecBuilder.fill(xyStd, xyStd, rotStd);
  }

  /**
   * Add the Pose3d returned by vision to the odermetry with standard devs calculated from confidence levels
   * @param estimator
   */
  private void estimatorChecker(PhotonRunnable estimator, double confidenceScale) {
    // New pose from vision
    EstimatedRobotPose lastestVisionPose = estimator.grabLatestEstimatedPose();

    ChassisSpeeds speeds = SubsystemManager.getInstance().getDrivetrain().getSpeeds();
    // Adds linear speed and translational speed of the rotational component of the tag relative to the bot.
    double blurrinessFactor = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) + Math.abs(speeds.omegaRadiansPerSecond) * 3.0;
    SmartDashboard.putNumber("Blurniess Factor", blurrinessFactor);
    m_speedBuffer.addSample(Timer.getFPGATimestamp(), blurrinessFactor);
    if (lastestVisionPose != null){
      if(Math.abs(lastestVisionPose.timestampSeconds - lastTime) > 0.1) {
        Optional<Double> captureTimeSpeed = m_speedBuffer.getSample(lastestVisionPose.timestampSeconds);
        if (captureTimeSpeed.isPresent() && captureTimeSpeed.get() < 2.7) {
          SmartDashboard.putNumber("Latest Vision Time", lastestVisionPose.timestampSeconds);
          SmartDashboard.putNumber("Vision Delay", Timer.getFPGATimestamp()-lastestVisionPose.timestampSeconds);
          publisher.set(lastestVisionPose.estimatedPose);
          arrayPublisher.set(new Pose3d[] {lastestVisionPose.estimatedPose, new Pose3d(getCurrentPose().getX(), getCurrentPose().getY(), 0, new Rotation3d(0,0,getCurrentPose().getRotation().getRadians()))});
          poseEstimator.addVisionMeasurement(lastestVisionPose.estimatedPose.toPose2d(), lastestVisionPose.timestampSeconds, confidenceCalculator(lastestVisionPose, confidenceScale));
        }
      }
        lastTime = lastestVisionPose.timestampSeconds;
    }

  }
  
  /**
   * Turns off vision
   * @param Boolean that turns off vision if set to false
   */
  public void setVision(boolean visionStatus){
    useVision = visionStatus;
  }

  @Override
  public void periodic() {
    useVision = DriveSbTab.getInstance().isVisionEnabled();
    
    // Update pose estimator with vision
    if (useVision) {
      estimatorChecker(frontEstimator, 1.4);
      estimatorChecker(backEstimator, 1);
    }

    // Set the pose on the dashboard
    Pose2d dashboardPose = poseEstimator.getEstimatedPosition();

    field2d.setRobotPose(dashboardPose);
  }

  public NotePhoton getNoteCamera(){
    return noteCamera;
  }
}
