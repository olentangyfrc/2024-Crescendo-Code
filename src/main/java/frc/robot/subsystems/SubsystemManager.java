package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.IO.IO;
import frc.robot.IO.IO.XboxControllerName;
import frc.robot.IO.controllers.OzoneController;
import frc.robot.IO.controllers.OzoneController.ControllerButton;
import frc.robot.IO.controllers.OzoneHid.ButtonActionType;
import frc.robot.auton.AutoManager;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberPosition;
import frc.robot.subsystems.deflector.Deflector;
import frc.robot.subsystems.deflector.Deflector.DeflectorPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.DisableAngleSnap;
import frc.robot.subsystems.drivetrain.commands.EnableAngleSnap;
import frc.robot.subsystems.drivetrain.modules.DoubleTalonFxModule;
import frc.robot.subsystems.drivetrain.modules.SingleFalconModule;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.commands.ManualShoot;
import frc.robot.subsystems.shooter.commands.ManualShoot.ShootPosition;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.vision.VisionEstimatorSubsystem;


/**
 * This class instantiates and initializes all of the subsystems and stores
 * references to them.
 */
public class SubsystemManager {
    // ---------------------------------------------------
    // Singleton Logic
    // ---------------------------------------------------
    private static SubsystemManager instance;

    GenericEntry climberVoltage = Shuffleboard.getTab("Climber").add("Climber voltage", 0.0).getEntry();
    
    public static SubsystemManager getInstance() {
        if(instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }


    private final Logger logger = Logger.getLogger(this.getClass().getName());

    // MAC addresses of different robots, each has one for USB and radio
    private static final Map<String, BotType> allMACs = Map.of(
      "00:80:2F:37:3E:5A", BotType.CRESCENDO_COMP,
      "00:80:2F:30:DB:F8", BotType.COVID,
      "00:80:2F:30:DB:F9", BotType.COVID,
      "00:80:2F:25:B4:CA", BotType.RAPID_REACT,
      "00:80:2F:28:64:39", BotType.RIO99,
      "00:80:2F:28:64:38", BotType.RIO99,
      "00:80:2F:35:54:1E", BotType.CRESCENDO_CHASSIS,
      "00:80:2F:17:D7:4B", BotType.RIO2,
      "00:80:2F:27:1D:E9", BotType.BLUE
    );

    private BotType botType;

    // Subsystem declarations
    private PowerDistribution pdp;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Climber climber;
    private Deflector deflector;
    private Leds leds;

    private VisionEstimatorSubsystem vision;
    private AutoManager autoManager;

    private boolean disableTransfer = false;

    // ---------------------------------------------------
    // Initialization Methods
    // ---------------------------------------------------

    /**
     * Initialize SubsystemManager
     */
    public void init() {
        botType = getBotType();
        pdp = new PowerDistribution(1, ModuleType.kRev);

        switch(botType) {
            case CRESCENDO_CHASSIS:
                initCrescendoChassis2();
                break;
            case CRESCENDO_COMP:
                initCrescendoComp();
                break;
            default:
                logger.warning("Unrecognized Bot!");
        }
    }

    /**
     * Initialize Crescendo Chassis
     */
    public void initCrescendoChassis2() {
        drivetrain = new Drivetrain();
        drivetrain.init(
            new SwerveModule[] {
                new SingleFalconModule(15, 31, 0, Drivetrain.MAX_LINEAR_SPEED, 1 / 8.07, 321.62, Rotation2d.fromDegrees(45)),
                new SingleFalconModule(6, 30, 3, Drivetrain.MAX_LINEAR_SPEED, 1 / 8.07, 40.3, Rotation2d.fromDegrees(-45)),
                new SingleFalconModule(62, 32, 1, Drivetrain.MAX_LINEAR_SPEED, 1 / 8.07, 138.5, Rotation2d.fromDegrees(135)),
                new SingleFalconModule(14, 33, 2, Drivetrain.MAX_LINEAR_SPEED, 1 / 8.07, 4.4, Rotation2d.fromDegrees(-135))
            },
            1,
            0.495,
            0.545,
            new PIDConstants(0, 0, 0),
            new PIDConstants(8, 0, 0),
            false
        );

        autoManager = new AutoManager(drivetrain, new HashMap<>());

        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y.VALUE, Commands.runOnce(drivetrain::zeroYawToAllianceForwards).andThen(drivetrain::disablePassiveSnap));
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp.VALUE, new EnableAngleSnap(drivetrain, Rotation2d.fromDegrees(0)));
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialDown.VALUE, new EnableAngleSnap(drivetrain, Rotation2d.fromDegrees(180)));
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialLeft.VALUE, new EnableAngleSnap(drivetrain, Rotation2d.fromDegrees(90)));
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialRight.VALUE, new EnableAngleSnap(drivetrain, Rotation2d.fromDegrees(-90)));
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Back.VALUE, new DisableAngleSnap(drivetrain));
        
        IO.getInstance().getController().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A.VALUE, Commands.runOnce(() -> drivetrain.setFieldOriented(!drivetrain.isFieldOriented())));
    }
    
    /**
     * Initialize Crescendo Chassis
     */
    public void initCrescendoComp() {
        DriveSbTab.getInstance();

        drivetrain = new Drivetrain();

        drivetrain.init(
            new SwerveModule[] {
                new DoubleTalonFxModule("FL", 21, 20, 19, Drivetrain.MAX_LINEAR_SPEED, 1 / 7.7142857, 46.106, "*", Rotation2d.fromDegrees(45)),
                new DoubleTalonFxModule("FR", 23, 22, 18, Drivetrain.MAX_LINEAR_SPEED, 1 / 7.7142857, 1.8425, "*", Rotation2d.fromDegrees(-45)),
                new DoubleTalonFxModule("BL", 25, 24, 17, Drivetrain.MAX_LINEAR_SPEED, 1 / 7.7142857, 262.2657, "*", Rotation2d.fromDegrees(135)),
                new DoubleTalonFxModule("BR", 27, 26, 16, Drivetrain.MAX_LINEAR_SPEED, 1 / 7.7142857, 55.8984, "*", Rotation2d.fromDegrees(-135))
            },
            51,
            0.55,
            0.55,
            new PIDConstants(2, 0, 0),
            new PIDConstants(2, 0, 0),
            true
        );

        intake = new Intake(31, 42);
        shooter = new Shooter(35, 36, 37, 47);
        climber = new Climber(10);
        deflector = new Deflector(45);
        leds = new Leds(60);


        // SysIdCharacterizer intakeCharacterizer = new SysIdCharacterizer(0.33, 3, 30, intake, MechanismType.ANGULAR, "Intake");

        autoManager = new AutoManager(drivetrain, Map.of(
            "DeployIntake", Commands.runOnce(() ->intake.setState(IntakeState.DEPLOYING)),
            "Pre-Align", Commands.runOnce(() -> shooter.setShouldPreAlignNext(true)),
            "Shoot", Commands.either(
                // Wait for note then shoot
                new ParallelRaceGroup(new WaitCommand(1), new WaitUntilCommand(() -> shooter.getState() == ShooterState.HOLDING || shooter.getState() == ShooterState.SPEAKER_OPERATOR_ALIGNMENT)).andThen(
                    Commands.either(
                        new Shoot(shooter, drivetrain, true),
                        Commands.none(),
                        shooter::isBeamBroken
                    )
                ),
                Commands.runOnce(() -> shooter.setShouldPreAlignNext(false)),
                () -> intake.getState() == IntakeState.TRANSFER || shooter.getState() == ShooterState.FEEDING || shooter.getState() == ShooterState.HOLDING || shooter.getState() == ShooterState.SPEAKER_OPERATOR_ALIGNMENT
            ),
            "IgnoreNextDelay", Commands.runOnce(() -> shooter.setIgnoreNextVisionDelay(true)),


            // "Shoot", new Shoot(shooter, drivetrain, true).andThen(Commands.runOnce(() -> intake.setState(IntakeState.DEPLOYING))),
            "ShootNoRotate", new Shoot(shooter, drivetrain, false),
            "AlignNote", Commands.runOnce(() -> intake.setCenteringNote(true)),
            "StopAlignNote", Commands.runOnce(() -> intake.setCenteringNote(false))
            // "Eject", Commands.runOnce(() -> shooter.)
        ));

        vision = new VisionEstimatorSubsystem();
        vision.init();
        
        
        ///////////////////////////////////////////
        /*            Main Controller            */
        ///////////////////////////////////////////

        OzoneController mainController = IO.getInstance().getController();

        // Reset Robot Systems
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Start.VALUE, Commands.runOnce(() -> {
            shooter.setState(ShooterState.HOLDING);
            shooter.stopDrivingToAmp();
            intake.setState(IntakeState.SHOOTING);
            drivetrain.removeSnapAngle();
            drivetrain.removeLockedSpeeds();
            deflector.setTargetPosition(DeflectorPosition.STOWED);
        }));

        // Zero Gyro
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y.VALUE, Commands.runOnce(drivetrain::zeroYawToAllianceForwards).andThen(drivetrain::disablePassiveSnap));
        // Intake
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightTriggerButton.VALUE, Commands.runOnce(() -> intake.setState(IntakeState.DEPLOYING)));
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RightTriggerButton.VALUE, Commands.either(
            Commands.runOnce(() -> intake.setState(IntakeState.RETRACTED)),
            Commands.none(),
            () -> intake.getState() == IntakeState.DEPLOYED || intake.getState() == IntakeState.DEPLOYING
        ));
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftBumper.VALUE, Commands.runOnce(() -> intake.setCenteringNote(true)));
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftBumper.VALUE, Commands.runOnce(() -> intake.setCenteringNote(false)));
        // Eject
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton.VALUE, Commands.runOnce(() -> intake.setState(IntakeState.EJECT)));
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftTriggerButton.VALUE, Commands.runOnce(() -> intake.setState(IntakeState.RETRACTED)));
        // Auto Shoot
        Map<ShooterState, Command> shootCommands = new HashMap<>();
        shootCommands.put(ShooterState.AMP_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.AMP_SHOOTING)));
        shootCommands.put(ShooterState.SPEAKER_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_SHOOTING)));
        shootCommands.put(ShooterState.PASSING_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.PASSING)));
        shootCommands.put(ShooterState.SPEAKER_MOVING_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_MOVING_SHOOTING)));
        shootCommands.put(ShooterState.SPEAKER_OPERATOR_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_MOVING_SHOOTING)));
        shootCommands.put(ShooterState.HOLDING, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_MOVING_SHOOTING)));
        shootCommands.put(ShooterState.IDLE, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_MOVING_SHOOTING)));
        shootCommands.put(ShooterState.MANUAL_ALIGNMENT, Commands.runOnce(() -> shooter.setState(ShooterState.MANUAL_SHOOTING)));
        shootCommands.put(ShooterState.AMP_SHOOTING, Commands.runOnce(() -> shooter.setState(ShooterState.AMP_SHOOTING)));
        shootCommands.put(ShooterState.EJECT, Commands.none());
        shootCommands.put(ShooterState.FEEDING, Commands.none());
        shootCommands.put(ShooterState.MANUAL_SHOOTING, Commands.none());
        shootCommands.put(ShooterState.SHOOTER_INTAKE, Commands.none());
        shootCommands.put(ShooterState.SPEAKER_MOVING_SHOOTING, Commands.none());
        shootCommands.put(ShooterState.PASSING, Commands.none());
        shootCommands.put(ShooterState.SPEAKER_SHOOTING, Commands.none());

        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper.VALUE, Commands.select(
            shootCommands,
            shooter::getState
        ));
        // Alignment
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X.VALUE,
            new ParallelRaceGroup(new WaitCommand(1), new WaitUntilCommand(() -> shooter.getState() == ShooterState.HOLDING || shooter.getState() == ShooterState.SPEAKER_OPERATOR_ALIGNMENT))
            .andThen(() -> shooter.setState(ShooterState.AMP_ALIGNMENT))
        );
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.X.VALUE, Commands.runOnce(() -> shooter.stopDrivingToAmp()));
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A.VALUE, Commands.runOnce(() -> shooter.setState(ShooterState.PASSING)));
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B.VALUE, Commands.runOnce(() -> shooter.setState(ShooterState.MANUAL_ALIGNMENT)));
        // Climber
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp.VALUE, Commands.runOnce(() -> climber.enablePositionControl(ClimberPosition.EXTENDED)).alongWith(Commands.runOnce(() -> intake.setState(IntakeState.EXTEND))));
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialDown.VALUE, Commands.runOnce(() -> climber.setManual(-12)));
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RadialDown.VALUE, Commands.runOnce(() -> climber.exitManual()));
        // Bot-oriented drive
        mainController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Back.VALUE, Commands.runOnce(() -> drivetrain.setFieldOriented(false)));
        mainController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.Back.VALUE, Commands.runOnce(() -> drivetrain.setFieldOriented(true)));


        
        ///////////////////////////////////////////
        /*             Aux Controller            */
        ///////////////////////////////////////////
        OzoneController auxController = IO.getInstance().getController(XboxControllerName.AUX);
        // Manual Shots
        auxController.bind(ButtonActionType.WHEN_HELD, ControllerButton.RadialUp.VALUE, new ManualShoot(shooter, ShootPosition.PODIUM));
        auxController.bind(ButtonActionType.WHEN_HELD, ControllerButton.RadialLeft.VALUE, new ManualShoot(shooter, ShootPosition.AMP));
        auxController.bind(ButtonActionType.WHEN_HELD, ControllerButton.RadialDown.VALUE, new ManualShoot(shooter, ShootPosition.SUBWOOFER));
        // Restart Robot Systems
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Start.VALUE, Commands.runOnce(() -> {
            shooter.setState(ShooterState.HOLDING);
            shooter.stopDrivingToAmp();
            intake.setState(IntakeState.SHOOTING);
            drivetrain.removeSnapAngle();
            drivetrain.removeLockedSpeeds();
            deflector.setTargetPosition(DeflectorPosition.STOWED);
            CommandScheduler.getInstance().cancelAll();

        }));
        // Climb Manual
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftStickUp.VALUE, Commands.runOnce(() -> climber.setManual(12)));
        auxController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftStickUp.VALUE, Commands.runOnce(() -> climber.exitManual()));
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftStickDown.VALUE, Commands.runOnce(() -> climber.setManual(-12)));
        auxController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftStickDown.VALUE, Commands.runOnce(() -> climber.exitManual()));
        // Deflector Manual
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton.VALUE, Commands.runOnce(() -> deflector.manualControl(-1)));
        auxController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftTriggerButton.VALUE, Commands.runOnce(() -> deflector.exitManual()));
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightTriggerButton.VALUE, Commands.runOnce(() -> deflector.manualControl(1)));
        auxController.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RightTriggerButton.VALUE, Commands.runOnce(() -> deflector.exitManual()));
        // Shooter Intake
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper.VALUE, Commands.runOnce(() -> shooter.setState(ShooterState.SHOOTER_INTAKE)));
        // Shooter Eject
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B.VALUE, Commands.runOnce(() -> shooter.setState(ShooterState.EJECT)));
        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X.VALUE, Commands.runOnce((() -> {
            if(shooter.getState() != ShooterState.PASSING) {
                shooter.setState(ShooterState.PASSING_ALIGNMENT);
            }
        } )));

        auxController.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A.VALUE, Commands.runOnce(() -> shooter.setState(ShooterState.SPEAKER_OPERATOR_ALIGNMENT)));
    }


    // ---------------------------------------------------
    // Subsystem Getters
    // ---------------------------------------------------
    
    public PowerDistribution getPdp() {
        return pdp;
    }

    public Leds getLeds() {
        return leds;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public AutoManager getAutoManager() {
        return autoManager;
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climber getClimber() {
        return climber;
    }

    public Deflector getDeflector() {
        return deflector;
    }

    public VisionEstimatorSubsystem getVision() {
        return vision;
    }

    public boolean disableTransfer() {
        return disableTransfer;
    }

    public void setDisableTransfer(boolean disableTransfer) {
        this.disableTransfer = disableTransfer;
    }

    // ---------------------------------------------------
    // Logic for determining bot type
    // ---------------------------------------------------


    /**
    * @return The active bot type
    * @throws Exception
    */
    private BotType getBotType() {
        try {
            Enumeration<NetworkInterface> networks;
            networks = NetworkInterface.getNetworkInterfaces();
            BotType bot = BotType.UNRECOGNIZED;
            for (NetworkInterface net : Collections.list(networks)) {
                String mac = formatMACAddress(net.getHardwareAddress());
                logger.info("Network #" + net.getIndex() + " " + net.getName() + " " + mac);
                if (allMACs.containsKey(mac)) {
                    bot = allMACs.get(mac);
                    logger.info("   this MAC is for " + bot);
                }
            }

            return bot;
        } catch (SocketException ex) {
            return BotType.UNRECOGNIZED;
        }
    }

    /**
    * Formats the byte array representing the mac address as more human-readable
    * form
    * 
    * @param hardwareAddress byte array
    * @return string of hex bytes separated by colons
    */
    private String formatMACAddress(byte[] hardwareAddress) {
        if (hardwareAddress == null || hardwareAddress.length == 0) {
            return "";
        }

        StringBuilder mac = new StringBuilder(); 
        for (int k = 0; k < hardwareAddress.length; k++) {
            int i = hardwareAddress[k] & 0xFF; // unsigned integer from byte
            String hex = Integer.toString(i, 16);
            if (hex.length() == 1) { // we want to make all bytes two hex digits
                hex = "0" + hex;
            }
            mac.append(hex.toUpperCase());
            mac.append(":");
        }
        mac.setLength(mac.length() - 1); // trim off the trailing colon
        return mac.toString();
    }

    /**
    * Known bots to prevent user error
    */
    public enum BotType {
        CRESCENDO_COMP,
        COVID,
        RAPID_REACT,
        BLUE,
        RIO99,
        CRESCENDO_CHASSIS,
        RIO2,
        UNRECOGNIZED,
    }
}
