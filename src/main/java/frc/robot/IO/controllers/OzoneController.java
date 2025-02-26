package frc.robot.IO.controllers;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OzoneController extends OzoneHid {

    private Logger logger = Logger.getLogger(this.getClass().getName());

    private CommandXboxController baseController;

    public OzoneController(int port) {
        if(!DriverStation.getJoystickIsXbox(port)) {
            logger.warning("!!!Attempted to create controller on port [" + port + "], which is not an Xbox Controller!!!");
        }
        baseController = new CommandXboxController(port);

        populateButtons();
    }

    @Override
    protected void populateButtons() {
        buttons = new HashMap<>();

        // Default buttons
        for(int i = 1; i < 11; i++) {
            buttons.put(i, baseController.button(i));
        }

        // Custom buttons
        buttons.put(11, baseController.rightTrigger(NOMINAL_ANALOG_VALUE));
        buttons.put(12, baseController.leftTrigger(NOMINAL_ANALOG_VALUE));
        buttons.put(13, baseController.povUp());
        buttons.put(14, baseController.povRight());
        buttons.put(15, baseController.povDown());
        buttons.put(16, baseController.povLeft());
        buttons.put(17, new Trigger(() -> baseController.getLeftY() < -NOMINAL_ANALOG_VALUE));
        buttons.put(18, new Trigger(() -> baseController.getLeftY() > NOMINAL_ANALOG_VALUE));
        buttons.put(19, new Trigger(() -> baseController.getLeftX() < -NOMINAL_ANALOG_VALUE));
        buttons.put(20, new Trigger(() -> baseController.getLeftX() > NOMINAL_ANALOG_VALUE));
        buttons.put(21, new Trigger(() -> baseController.getRightY() < -NOMINAL_ANALOG_VALUE));
        buttons.put(22, new Trigger(() -> baseController.getRightY() > NOMINAL_ANALOG_VALUE));
        buttons.put(23, new Trigger(() -> baseController.getRightX() < -NOMINAL_ANALOG_VALUE));
        buttons.put(24, new Trigger(() -> baseController.getRightX() > NOMINAL_ANALOG_VALUE));
    }

    /**
     * Squares the input and applys a deadzone to it
     * 
     * @param raw Raw value, from -1 to 1
     * @return Filtered value
     */
    protected static double filterInput(double raw) {
        double x = Math.copySign(Math.pow(raw, 2), raw); // Square input
        return MathUtil.applyDeadband(x, DEADZONE); // Apply deadzone and return
    }

    /**
     * @return Get the horizontal axis of the left stick on the Xbox controller
     */
    public double getLeftX(){
        return filterInput(baseController.getLeftX());
    }

    /**
     * @return Get the horizontal axis of the right stick on the Xbox controller
     */
    public double getRightX(){
        return filterInput(baseController.getRightX());
    }
    
    /**
     * @return Get the vertical axis of the left stick on the Xbox controller
     */
    public double getLeftY(){
        return filterInput(-baseController.getLeftY());
    }

    /**
     * @return Get the vertical axis of the right stick on the Xbox controller
     */
    public double getRightY(){
        return filterInput(-baseController.getRightY());
    }

    /**
     * Rumble the controller
     * 
     * @param value Rumble value [0, 1]
     */
    public void rumble(double value) {
        baseController.getHID().setRumble(RumbleType.kBothRumble, MathUtil.clamp(value, 0, 1));
    }

    public enum ControllerButton {
        A(1),
        B(2),
        X(3),
        Y(4),
        LeftBumper(5),
        RightBumper(6),
        Back(7),
        Start(8),
        LeftStick(9),
        RightStick(10),
        RightTriggerButton(11),
        LeftTriggerButton(12),
        RadialUp(13),
        RadialRight(14),
        RadialDown(15),
        RadialLeft(16),
        LeftStickUp(17),
        LeftStickDown(18),
        LeftStickLeft(19),
        LeftStickRight(20),
        RightStickUp(21),
        RightStickDown(22),
        RightStickLeft(23),
        RightStickRight(24);

        public final int VALUE;
        
        ControllerButton(int VALUE) {
            this.VALUE = VALUE;
        }
    }
}
