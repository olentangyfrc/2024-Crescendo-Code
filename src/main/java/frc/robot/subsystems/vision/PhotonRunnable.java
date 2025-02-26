package frc.robot.subsystems.vision;



import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final Transform3d transform;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  private static final double FIELD_LENGTH_METERS = 16.51;
  private static final double FIELD_WIDTH_METERS = 8.2112358;
  private static final double MIN_TARGETS = 2;
  private static final double Z_TOLERANCE = 0.1;



  public PhotonRunnable(PhotonCamera camera, Transform3d robotToCamera) {
    this.photonCamera = camera;
    this.transform = robotToCamera;
    PhotonPoseEstimator photonPoseEstimator = null;
    AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    if (photonCamera != null) {
      photonPoseEstimator = new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    if (photonPoseEstimator != null && photonCamera != null) {
      PhotonPipelineResult photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets()
          && (photonResults.targets.size() > MIN_TARGETS || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field 
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS
                && Math.abs(estimatedPose.getZ()) <= Z_TOLERANCE) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
           //atomicEstimatedRobotPose.set(estimatedRobotPose);
        });

      }
    }
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If
   * it returns a non-null value, it is a new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance.
   * 
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.get();
  }

  public enum Type{
    SPEAKER,
    AMP
  }


  
  private Translation3d getTranslation(PhotonTrackedTarget target){
      return target.getBestCameraToTarget().getTranslation().minus(new Translation3d(Math.sqrt(Math.pow(transform.getX(),2)+Math.pow(transform.getY(),2)), transform.getRotation()));
  }
  
  /**
   * Gets Distance from Speaker/Amp
   * @param enum of Source or Amp
   * @return distance in meters
   */
  public Translation3d getDistanceFromTarget(Type target) {
    PhotonTrackedTarget fiducial = photonCamera.getLatestResult().getBestTarget();
    Translation3d translation = (DriverStation.getAlliance().get() == Alliance.Blue) ?
        (target == Type.SPEAKER) ?
          (fiducial.getFiducialId() == 3 || fiducial.getFiducialId() == 4) ?  getTranslation(fiducial) : new Translation3d(): //Blue Speaker Ids
          (fiducial.getFiducialId() == 5) ? getTranslation(fiducial) : new Translation3d(): //Blue Amp Ids
        (target == Type.SPEAKER) ?
          (fiducial.getFiducialId() == 7 || fiducial.getFiducialId() == 8) ?  getTranslation(fiducial) : new Translation3d() : //Red Speaker Ids
          (fiducial.getFiducialId() == 6) ? getTranslation(fiducial) : new Translation3d(); //Red Amp Ids
    return translation;
  }


}
