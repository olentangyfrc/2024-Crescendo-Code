package frc.robot.auton;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DriveSignal;

/**
 * The AutoManager manages auton routines.
 * 
 */
public class AutoManager {
    private static final String TAB_NAME = "Auto Manager";
    private static final boolean SHOW_CHOOSER = true;
    
    private SendableChooser<Command> autoChooser;

    private static final Translation2d AMP_LOCATION = new Translation2d(1.84, 7.66);

    /**
     * Construct a new AutoManager
     * 
     * @param drivetrain
     * @param namedCommands
     */
    public AutoManager(Drivetrain drivetrain, Map<String, Command> namedCommands) {

        NamedCommands.registerCommands(namedCommands);

        AutoBuilder.configureHolonomic(
            drivetrain::getLocation,
            drivetrain::resetLocation,
            drivetrain::getSpeeds,
            speeds -> drivetrain.setSignal(new DriveSignal(speeds, false)),
            new HolonomicPathFollowerConfig(
                drivetrain.getTranslationConstants(),
                drivetrain.getRotationConstants(),
                Drivetrain.MAX_LINEAR_SPEED,
                Math.hypot(drivetrain.getTrackWidth() / 2, drivetrain.getWheelBase() / 2),
                new ReplanningConfig(true, true)
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drivetrain
        );

        autoChooser = AutoBuilder.buildAutoChooser();

        if(SHOW_CHOOSER) {
            Shuffleboard.getTab(TAB_NAME).add(autoChooser);
        }
    }

    /**
     * @return The auto command that has been selected on shuffleboard
     */
    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Get an auto command by name
     * 
     * @param auto The auto to get
     * @return The auto command
     */
    public Command getAutoCommand(Auto auto) {
        return new PathPlannerAuto(auto.name);
    }

    public Command getDriveToAmpCommand(Pose2d currentLocation, ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d initialHeading = new Rotation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

        // If we aren't moving (or moving very very slowly), point initial heading to the amp.
        if(Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond) < 0.05) {
            initialHeading = AMP_LOCATION.minus(currentLocation.getTranslation()).getAngle();
        }


        List<Translation2d> pathPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(currentLocation.getTranslation(), initialHeading),
            new Pose2d(AMP_LOCATION, Rotation2d.fromDegrees(90))
        );

        PathPlannerPath path = new PathPlannerPath(
            pathPoints,
            new PathConstraints(
                Drivetrain.MAX_LINEAR_SPEED * 0.8,
                Drivetrain.MAX_LINEAR_SPEED * 0.8,
                2 * Math.PI,
                4 * Math.PI
            ), 
            new GoalEndState(0, Rotation2d.fromDegrees(90), true)
        );

        return AutoBuilder.followPath(path).andThen(Commands.runOnce(() -> SubsystemManager.getInstance().getDrivetrain().setSignal(new DriveSignal(new ChassisSpeeds(), true))));
    }

    public enum Auto {
        TEST("Test Auto");

        public String name;

        private Auto(String name) {
            this.name = name;
        }
    }
}
