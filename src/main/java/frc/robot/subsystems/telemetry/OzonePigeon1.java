package frc.robot.subsystems.telemetry;

import java.util.Map;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class OzonePigeon1 extends OzoneImu {
    private WPI_PigeonIMU basePigeon;

    /**
     * Construct a new PigeonIMU
     * <p>
     * Use OzonePigeon2 for the Pigeon 2.0!
     * 
     * @param canId CAN ID of the device
     * @param invertYaw True if yaw should be inverted
     * @param invertPitch True if pitch should be inverted
     * @param invertRoll True if roll should be inverted
     */
    public OzonePigeon1(int canId, boolean invertYaw, boolean invertPitch, boolean invertRoll) {
        basePigeon = new WPI_PigeonIMU(canId);

        // Reset configuration
        basePigeon.configFactoryDefault();

        axisInversions.putAll(Map.of(
            GyroAxis.YAW, invertYaw,
            GyroAxis.PITCH, invertPitch,
            GyroAxis.ROLL, invertRoll
        ));
    }

    /**
     * Construct a new PigeonIMU
     * <p>
     * Use OzonePigeon2 for the Pigeon 2.0!
     * 
     * @param canId CAN ID of the device
     * @param invertYaw True if yaw should be inverted
     */
    public OzonePigeon1(int canId, boolean invertYaw) {
        this(canId, invertYaw, false, false);
    }

    /**
     * Construct a new PigeonIMU
     * <p>
     * Use OzonePigeon2 for the Pigeon 2.0!
     * 
     * @param canId CAN ID of the device
     */
    public OzonePigeon1(int canId) {
        this(canId, false, false, false);
    }

    /**
     * Get the raw angle of an axis
     * 
     * @return The raw angle of an axis in degrees
     */
    @Override
    protected double getRawAxisDegrees(GyroAxis axis) {
        switch(axis) {
            case YAW:
                return basePigeon.getYaw();
            case PITCH:
                return basePigeon.getPitch();
            case ROLL:
                return basePigeon.getRoll();
            default:
                return 0.0; // Should be unreachable
        }
    }
    
}
