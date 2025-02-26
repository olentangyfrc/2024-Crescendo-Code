package frc.robot.subsystems.telemetry;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class is to be used for different Pigeon IMUs and NavXs
 */
public abstract class OzoneImu {
    protected Map<GyroAxis, Double> axisOffsets = new HashMap<>(Map.of(
        GyroAxis.YAW, 0.0,
        GyroAxis.PITCH, 0.0,
        GyroAxis.ROLL, 0.0
    ));

    protected Map<GyroAxis, Boolean> axisInversions = new HashMap<>(Map.of(
        GyroAxis.YAW, false,
        GyroAxis.PITCH, false,
        GyroAxis.ROLL, false
    ));

    /**
     * Get the raw degrees of an axis
     * 
     * @param axis The axis to query
     * @return The axis' raw angle in degrees, without an offset or inversion
     */
    protected abstract double getRawAxisDegrees(GyroAxis axis);

    /**
     * Get the angle of an axis in degrees
     * 
     * @param axis The axis to query
     * @return The axis' angle in degrees
     */
    public double getAngle(GyroAxis axis) {
        double angleDeg = getRawAxisDegrees(axis) - axisOffsets.get(axis);
        if(axisInversions.get(axis)) {
            angleDeg *= -1;
        }

        return angleDeg;
    }
    
    /**
     * Get the angle of an axis as a Rotation2d
     * 
     * @param axis The axis to query
     * @return The axis' angle as a Rotation2d
     */
    public Rotation2d getRotation2d(GyroAxis axis) {
        return Rotation2d.fromDegrees(getAngle(axis));
    }

    /**
     * Reset the value of an axis to a new value
     * 
     * @param axis The axis to reset
     * @param newValue The new value in degrees
     */
    public void zeroAxis(GyroAxis axis, double newValue) {
        axisOffsets.put(axis, getRawAxisDegrees(axis) - newValue);
    }

    /**
     * Reset the value of an axis to zero
     * 
     * @param axis The axis to reset
     */
    public void zeroAxis(GyroAxis axis) {
        zeroAxis(axis, 0.0);
    }

    /**
     * Reset the yaw to zero if on blue alliance or 180 if on red alliance
     */
    public void zeroYawToAllianceForwards() {
        zeroAxis(GyroAxis.YAW, (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? 0 : 180);
    }

    /**
     * Simultaniously reset Yaw, Pitch, and Roll to zero
     */
    public void zeroAllAxes() {
        zeroAxis(GyroAxis.YAW);
        zeroAxis(GyroAxis.PITCH);
        zeroAxis(GyroAxis.ROLL);
    }


    public enum GyroAxis {
        YAW,
        PITCH,
        ROLL,
    }
}