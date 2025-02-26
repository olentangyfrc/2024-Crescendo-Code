package frc.robot.util.sysid;

public interface Characterizable {

    /**
     * DO NOT USE EXCEPT FOR SYSID CHARACTERIZATION
     * 
     * Sets the voltage of the mechanism
     * 
     * @param voltage
     */
    public void sysId_driveVoltage(double voltage);

    /**
     * @return the position of the mechanism (either meters or radians)
     */
    public double sysId_getPosition();

    /**
     * @return Supplies the velocity of the mechanism (either meters per second or radians per second)
     */
    public double sysId_getVelocity();

    /**
     * @return Supplies the current voltage supplied to the mechanism (volts)
     */
    public double sysId_getVoltage();
}
