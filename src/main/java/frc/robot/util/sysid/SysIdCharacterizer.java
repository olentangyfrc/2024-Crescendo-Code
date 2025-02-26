package frc.robot.util.sysid;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIdCharacterizer {
    private String name;
    private SysIdRoutine routine;
    private Command QF, QR, DF, DR;
    private Supplier<Double> voltageSupplier;

    /**
     * Construct a new SysIdCharacterizer
     * 
     * @param rampRate Speed at which applied voltage increases in quasistatic tests (Volts per second)
     * @param stepVoltage Initial voltage applied during dynamic tests to measure acceleration. (Volts)
     * @param timeout Seconds until a test will automatically stop.
     * @param system The Characterizable system to be characterized.
     * @param mechanismType The type of mechanism
     * @param name The name of the mechanism. ex: "Flywheel"
     */
    public SysIdCharacterizer(double rampRate, double stepVoltage, double timeout, Characterizable system, MechanismType type, String name) {
        this(rampRate, stepVoltage, timeout, system::sysId_driveVoltage, system::sysId_getPosition, system::sysId_getVelocity, system::sysId_getVoltage, type, name);
    }

    /**
     * Construct a new SysIdCharacterizer
     * 
     * @param rampRate Speed at which applied voltage increases in quasistatic tests (Volts per second)
     * @param stepVoltage Initial voltage applied during dynamic tests to measure acceleration. (Volts)
     * @param timeout Seconds until a test will automatically stop.
     * @param positionSupplier Supplies the position of the mechanism (either meters or radians)
     * @param velocitySupplier Supplies the velocity of the mechanism (Either meters per second or radians per second)
     * @param voltageSupplier Supplies the voltage applied to the mechanism (volts)
     * @param mechanismType The type of mechanism
     * @param name The name of the mechanism. ex: "Flywheel"
     */
    public SysIdCharacterizer(
        double rampRate,
        double stepVoltage,
        double timeout,
        Consumer<Double> drive,
        Supplier<Double> positionSupplier,
        Supplier<Double> velocitySupplier,
        Supplier<Double> voltageSupplier,
        MechanismType mechanismType,
        String name
    ) {
        this.name = name;
        this.voltageSupplier = voltageSupplier;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
            Units.Volts.per(Units.Second).of(rampRate), // ramp rate
            Units.Volts.of(stepVoltage), // Step Voltage
            Units.Seconds.of(timeout)
        );

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(mv -> drive.accept(mv.magnitude()), log -> {
            if(mechanismType == MechanismType.ANGULAR) {
                log.motor(name)
                    .angularPosition(Units.Radians.of(positionSupplier.get()))
                    .angularVelocity(Units.RadiansPerSecond.of(velocitySupplier.get()))
                    .voltage(Units.Volts.of(voltageSupplier.get()));
            } else {
                log.motor(name)
                    .linearPosition(Units.Meters.of(positionSupplier.get()))
                    .linearVelocity(Units.MetersPerSecond.of(velocitySupplier.get()))
                    .voltage(Units.Volts.of(voltageSupplier.get()));
            }
        }, new SubsystemSkeleton(), name);

        routine = new SysIdRoutine(config, mechanism);

        QF = routine.quasistatic(Direction.kForward).andThen(Commands.runOnce(() -> drive.accept(0.0)));
        QR = routine.quasistatic(Direction.kReverse).andThen(Commands.runOnce(() -> drive.accept(0.0)));
        DF = routine.dynamic(Direction.kForward).andThen(Commands.runOnce(() -> drive.accept(0.0)));
        DR = routine.dynamic(Direction.kReverse).andThen(Commands.runOnce(() -> drive.accept(0.0)));

        createDashboard();
    }

    /**
     * Initialize a Shuffleboard dashboard for running SysId routines
     */
    private void createDashboard() {
        String tabName = name + " SysId";

        Shuffleboard.getTab(tabName).add("Quasistatic Forward", Commands.print("RUN TEST").andThen(QF)).withPosition(0, 0);
        Shuffleboard.getTab(tabName).add("Quasistatic Reverse", QR).withPosition(1, 0);
        Shuffleboard.getTab(tabName).add("Dynamic Forward", DF).withPosition(2, 0);
        Shuffleboard.getTab(tabName).add("Dynamic Reverse", DR).withPosition(3, 0);

        Shuffleboard.getTab(tabName).add("End Test", new InstantCommand() {
            @Override
            public void initialize() {
                CommandScheduler.getInstance().cancel(QF, QR, DF, DR);
            }
        }).withSize(4, 1).withPosition(0, 1);

        Shuffleboard.getTab(tabName).addNumber("Voltage", () -> voltageSupplier.get()).withPosition(4, 0);
    }

    public enum MechanismType {
        LINEAR, // Ex: An elevator
        ANGULAR, // Ex: A shooter's velocity, or a deployable intake
    }

    // Used to fulfill SysIdRoutine.Mechanism's requirement for a subsystem
    private static class SubsystemSkeleton extends SubsystemBase {}
}