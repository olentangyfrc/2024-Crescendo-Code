// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.vector.Vector2d;

/** Add your docs here. */
public class ShooterUtils {
    public static final Translation2d SPEAKER_OFFSET = new Translation2d(0.1, 0);
    public static final Translation2d BLUE_SPEAKER_POSITION = new Translation2d(0.0, 5.55);
    public static final Translation2d RED_SPEAKER_POSITION = new Translation2d(16.58, 5.55);
    public static final Translation2d BLUE_SPEAKER_ADJUSTED_POSITION = new Translation2d(0.0, 5.55).plus(SPEAKER_OFFSET);
    public static final Translation2d RED_SPEAKER_ADJUSTED_POSITION = new Translation2d(16.58, 5.55).minus(SPEAKER_OFFSET);


    private static final double TIME_TO_SHOOT = 0.07; //seconds
    private static final double NOTE_VELOCITY = 25; //meters/second
    public static final Translation2d BLUE_PASSING_POSITION = new Translation2d(2, 6.00);
    public static final Translation2d RED_PASSING_POSITION = new Translation2d(14.58, 7.05);


    // private static final double TIME_TO_SHOOT = 0.07; //seconds
    // private static final double ACCELERATION_NOTE = 0.097763974; // meter/seconds^2
    // private static final double MAX_CAPPED_VELOCITY = 0.17; //meters/second

    // private static final double BASE_SHOOTING_VELOCITY = 75;

    private static InterpolatingDoubleTreeMap passingAngleMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap passingSpeedMap = new InterpolatingDoubleTreeMap();

    private static Set<ShooterState> alignmentStates = Set.of(
        ShooterState.AMP_ALIGNMENT, 
        ShooterState.MANUAL_ALIGNMENT, 
        ShooterState.SPEAKER_ALIGNMENT, 
        ShooterState.SPEAKER_MOVING_ALIGNMENT, 
        ShooterState.PASSING_ALIGNMENT
    );

    private static Set<ShooterState> shootingStates = Set.of(
        ShooterState.AMP_SHOOTING, 
        ShooterState.MANUAL_SHOOTING, 
        ShooterState.SPEAKER_SHOOTING, 
        ShooterState.SPEAKER_MOVING_SHOOTING, 
        ShooterState.PASSING
    );

    private static InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    // static {
    //     shooterAngleMap.put(1.25, 55.0);
    //     shooterAngleMap.put(1.72, 49.0);
    //     shooterAngleMap.put(2.28, 43.0);
    //     shooterAngleMap.put(3.02, 37.0);
    //     shooterAngleMap.put(3.7, 35.6);
    //     shooterAngleMap.put(4.28, 34.5);
    //     shooterAngleMap.put(5.01, 33.6);
        

    // }
    // static {
    //     shooterAngleMap.put(1.39, 55.0);
    //     shooterAngleMap.put(1.72, 50.0);
    //     shooterAngleMap.put(2.2, 45.0);
    //     shooterAngleMap.put(2.47, 42.0);
    //     shooterAngleMap.put(2.8, 38.0);
    //     shooterAngleMap.put(3.16, 36.0);
    //     shooterAngleMap.put(3.46, 34.0);
    //     shooterAngleMap.put(3.77, 33.2); 
    //     shooterAngleMap.put(3.9, 31.4); 
    //     shooterAngleMap.put(4.33, 29.7); 
    //     shooterAngleMap.put(4.61, 28.4); 
    //     shooterAngleMap.put(5.0, 29.4); 
    //     shooterAngleMap.put(5.4, 28.4); 

    // }

    static {
        shooterAngleMap.put(1.39, 53.768);
        shooterAngleMap.put(1.47, 51.0);
        shooterAngleMap.put(1.79, 47.0);
        shooterAngleMap.put(2.23, 40.25);
        shooterAngleMap.put(2.56, 37.9);
        shooterAngleMap.put(2.8, 36.4); 

        shooterAngleMap.put(3.05, 34.5); 
        shooterAngleMap.put(3.47, 31.3); 
        shooterAngleMap.put(3.8, 30.5); 
        shooterAngleMap.put(4.25, 29.1); 
        shooterAngleMap.put(4.36, 27.0);
        shooterAngleMap.put(4.7, 27.0);
        shooterAngleMap.put(5.06, 24.6);
        shooterAngleMap.put(5.61, 24.15);
    }

    static {
        passingAngleMap.put(0.0, 30.0);
        passingAngleMap.put(5.0,50.0);
        passingAngleMap.put(6.0,50.0);
        passingAngleMap.put(7.0,50.0);
        passingAngleMap.put(8.0,45.0);
        passingAngleMap.put(9.0,40.0);
        passingAngleMap.put(10.0, 42.0);

        passingSpeedMap.put(0.0, 10.0);
        passingSpeedMap.put(5.0, 28.0);
        passingSpeedMap.put(6.0, 32.0);
        passingSpeedMap.put(7.0, 38.0);
        passingSpeedMap.put(8.0, 42.0);
        passingSpeedMap.put(9.0, 46.5);
        passingSpeedMap.put(10.0, 49.0);
        passingSpeedMap.put(11.0, 54.0);
    }

    public static double getShooterAngle(double distance) {
        return shooterAngleMap.get(distance);
    }
  
    public static double getPassingAngle(double distance) {
        return passingAngleMap.get(distance);
    }

    public static double getPassingSpeed(double distance) {
        return passingSpeedMap.get(distance);
    }

    public static Translation2d getSpeakerLoc() {
        return getSpeakerLoc(true);
    }

    public static Translation2d getSpeakerLoc(boolean adjusted) {
        if(adjusted) {
            return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? BLUE_SPEAKER_ADJUSTED_POSITION : RED_SPEAKER_ADJUSTED_POSITION;
        } else {
            return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? BLUE_SPEAKER_POSITION : RED_SPEAKER_POSITION;
        }
    }

    public static Translation2d getPassingLoc() {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)? BLUE_PASSING_POSITION : RED_PASSING_POSITION;
    }


    public static Translation2d getShotPosition(Translation2d robotFieldRelativePosition, Vector2d robotFieldRelativeVelocity) {
        double distanceToSpeaker = robotFieldRelativePosition.getDistance(getSpeakerLoc());
        double t = TIME_TO_SHOOT + distanceToSpeaker / NOTE_VELOCITY;

        double x = getSpeakerLoc().getX() - (robotFieldRelativeVelocity.getX() * t);
        double y = getSpeakerLoc().getY() - (robotFieldRelativeVelocity.getY() * t);
        return new Translation2d(x,y);
    }
    
    public static ShootingParams getShootingWhileMovingParams(Translation2d robotFieldRelativePosition, Vector2d robotFieldRelativeVelocity){
        Translation2d shotPosition = getShotPosition(robotFieldRelativePosition, robotFieldRelativeVelocity);
        double shooterAngle = getShooterAngle(shotPosition.minus(robotFieldRelativePosition).getNorm());
        return new ShootingParams(shooterAngle, shotPosition.minus(robotFieldRelativePosition).getAngle().plus(Rotation2d.fromDegrees(180)).getDegrees());
    }

    public static Translation2d getPassingPosition(Translation2d robotFieldRelativePosition, Vector2d robotFieldRelativeVelocity, double shotRPS) {
        double distanceToPassPoint = robotFieldRelativePosition.getDistance(getPassingLoc());
        double x = getPassingLoc().getX() - (robotFieldRelativeVelocity.getX() * (distanceToPassPoint / (NOTE_VELOCITY)));
        double y = getPassingLoc().getY() - (robotFieldRelativeVelocity.getY() * (distanceToPassPoint / (NOTE_VELOCITY)));
        return new Translation2d(x,y);
    }

    public static ShootingParams getPassingWhileMovingParams(Translation2d robotFieldRelativePosition, Vector2d robotFieldRelativeVelocity, double shooterRPS) {
        Translation2d shotPosition = getPassingPosition(robotFieldRelativePosition, robotFieldRelativeVelocity, shooterRPS);
        double shooterAngle = getPassingAngle(shotPosition.minus(robotFieldRelativePosition).getNorm());
        return new ShootingParams(shooterAngle, shotPosition.minus(robotFieldRelativePosition).getAngle().plus(Rotation2d.fromDegrees(180 + 4)).getDegrees());
    }

    public static Boolean isShooting() {
        return shootingStates.contains(SubsystemManager.getInstance().getShooter().getState());
    }

    public static Boolean isAligning() {
        return alignmentStates.contains(SubsystemManager.getInstance().getShooter().getState());
    }
  

    public static class ShootingParams {
        public double shooterPivotDegrees;
        public double driveSnapDegrees;

        public ShootingParams(double shooterPivotDegrees, double driveSnapDegrees) {
            this.shooterPivotDegrees = shooterPivotDegrees;
            this.driveSnapDegrees = driveSnapDegrees;
        }
    }
}
