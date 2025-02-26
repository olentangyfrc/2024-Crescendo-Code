package frc.robot.subsystems.telemetry;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class OzonePigeon2 extends OzoneImu {
    private Pigeon2 basePigeon;

    /**
     * Construct a new Pigeon 2.0
     * 
     * @param canId CAN ID of the device
     * @param invertYaw True if yaw should be inverted
     * @param invertPitch True if pitch should be inverted
     * @param invertRoll True if roll should be inverted
     */
    public OzonePigeon2(int canId, String canBus, boolean invertYaw, boolean invertPitch, boolean invertRoll) {
        basePigeon = new Pigeon2(canId, canBus);

        // Reset configuration
        Pigeon2Configuration config = new Pigeon2Configuration();
        basePigeon.getConfigurator().apply(config);
        axisInversions.put(GyroAxis.YAW, invertYaw);
        axisInversions.put(GyroAxis.PITCH, invertPitch);
        axisInversions.put(GyroAxis.ROLL, invertRoll);
    }
    
    /**
     * Construct a new Pigeon 2.0
     * 
     * @param canId CAN ID of the device
     */
    public OzonePigeon2(int canId, String canBus) {
        this(canId, canBus, false, false, false);
    }

    /**
     * Construct a new Pigeon 2.0
     * 
     * @param canId CAN ID of the device
     */
    public OzonePigeon2(int canId) {
        this(canId, "rio", false, false, false);
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
                return basePigeon.getYaw().getValueAsDouble() % 360;
            case PITCH:
                return basePigeon.getPitch().getValueAsDouble() % 360;
            case ROLL:
                return basePigeon.getRoll().getValueAsDouble() % 360;
            default:
                return 0.0; // Should be unreachable
        }
    }
    
}
