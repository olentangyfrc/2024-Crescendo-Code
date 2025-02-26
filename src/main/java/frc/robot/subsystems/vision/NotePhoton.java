package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SubsystemManager;

public class NotePhoton {
    private final PhotonCamera photonCamera;
    private PhotonTrackedTarget note;
    private PhotonPipelineResult photonResults;
    private double lastNoteTime = -20.0;

    public NotePhoton(PhotonCamera camera) {
        this.photonCamera = camera;
    }

    public void run(){
        if (photonCamera != null){
            try {
                photonResults = photonCamera.getLatestResult();
                int target_id = 0;
                if (photonResults.hasTargets()){
                    for(int i =0; i < photonResults.targets.size(); i++){
                        if (photonResults.targets.get(i).getPitch() > 0){
                            photonResults.targets.remove(i--);
                        } 
                    }
                    double pitch_Priority = 5.0;
                    double yaw_Priority = 5.0;
                    double minimumSum = Double.POSITIVE_INFINITY; 
                    for(int i=0; i < photonResults.targets.size(); i++){
                        pitch_Priority = 0.4 * Math.abs(photonResults.targets.get(i).getPitch() + 18);
                        yaw_Priority = Math.abs(photonResults.targets.get(i).getYaw());
                        double sum = pitch_Priority + yaw_Priority;                   
                        if (sum < minimumSum){
                            minimumSum = sum;
                            target_id = i;
                        }
                    }
                    lastNoteTime = Timer.getFPGATimestamp();
                    if (!photonResults.targets.isEmpty()) {
                        note = photonResults.targets.get(target_id);
                    }
                }
            } catch(Exception ex) {
                ex.printStackTrace();
            }
        }
    }

    public double getAngle(){
        SmartDashboard.putNumber("Note Yaw", note.getYaw());
        SmartDashboard.putNumber("Centering Intake Angle", (note.getYaw() + SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees()) % 360.0 );
        return (-2*(note.getYaw()) + SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees()) % 360.0;

    }

    public double getNoteYaw(){
        return note.getYaw();
    }

    public boolean getPresent(){
        return Math.abs(Timer.getFPGATimestamp() - lastNoteTime) < 0.2;
    }
}
