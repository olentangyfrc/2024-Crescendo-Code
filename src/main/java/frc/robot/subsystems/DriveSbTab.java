package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DriveSbTab {
    private static final int WIDTH = 10;

    private static DriveSbTab instance;

    public static DriveSbTab getInstance() {
        if(instance == null) {
            instance = new DriveSbTab();
        }

        return instance;
    }

    private static ShuffleboardTab tab = Shuffleboard.getTab("Driver Tab");

    private static ShuffleboardTab disableTab = Shuffleboard.getTab("Disable Buttons");

    private static GenericEntry climberEnable = disableTab.add("Climb", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 1, 0).getEntry();
    private static GenericEntry deflectorEnable= disableTab.add("Deflector", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 1, 1).getEntry();
    private static GenericEntry intakeEnable = disableTab.add("Intake", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 1, 2).getEntry();
    private static GenericEntry shooterEnable = disableTab.add("Shooter", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 1, 3).getEntry();

    private static GenericEntry noteAlignmentEnabled = tab.add("Note Alignment", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 3, 0).withSize(1, 1).getEntry();
    private static GenericEntry driveSnapEnabled = tab.add("Drive Snap", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 3, 1).withSize(1, 1).getEntry();
    private static GenericEntry visionEnabled = tab.add("Vision", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 3, 2).withSize(1, 1).getEntry();

    private static GenericEntry ampDriveEnabled = tab.add("Amp Drive", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(WIDTH - 2, 0).withSize(1, 1).getEntry();

    public DriveSbTab() {
        SubsystemManager sm = SubsystemManager.getInstance();

        // tab.addCamera("Intake Camera", "Intake", null)

        tab.addString("Intake State", () -> (sm.getIntake() == null)? "None" :  sm.getIntake().getState().toString()).withPosition(7, 3);
        tab.addString("Shooter State", () -> (sm.getShooter() == null)? "None" :  sm.getShooter().getState().toString()).withPosition(8, 3);
        tab.addDouble("Match Time", DriverStation::getMatchTime).withPosition(8, 1).withSize(2, 2);
        // tab.add()
    }

    public boolean isClimberEnabled() {
        return climberEnable.getBoolean(true);
    }
    
    public boolean isDeflectorEnabled() {
        return deflectorEnable.getBoolean(true);
    }

    public boolean isIntakeEnabled() {
        return intakeEnable.getBoolean(true);
    }

    public boolean isShooterEnabled() {
        return shooterEnable.getBoolean(true);
    }

    public boolean isNoteAlignmentEnabled() {
        return noteAlignmentEnabled.getBoolean(false);
    }

    public boolean isDriveSnapEnabled() {
        return driveSnapEnabled.getBoolean(true);
    }

    public boolean isVisionEnabled() {
        return visionEnabled.getBoolean(true);
    }

    public boolean isAmpDriveEnabled() {
        return ampDriveEnabled.getBoolean(false);
    }

}
