package frc.robot.IO.controllers;

import java.util.HashMap;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class ButtonBox extends OzoneHid {
    private Logger logger = Logger.getLogger(this.getClass().getName());

    private CommandGenericHID leftButtonBox;
    private CommandGenericHID rightButtonBox;

    public ButtonBox(int leftPort, int rightPort) {
        if(!DriverStation.isJoystickConnected(leftPort) || !DriverStation.isJoystickConnected(rightPort)) {
            logger.warning("!!!Attempted to create button box on ports [" + leftPort + ", " + rightPort + "], no joysticks found!!!");
        }

        leftButtonBox = new CommandGenericHID(rightPort);
        rightButtonBox = new CommandGenericHID(rightPort);

        populateButtons();
    }

    @Override
    protected void populateButtons() {
        buttons = new HashMap<>();

        for(int i = 1; i < 12; i++) {
            buttons.put(i, leftButtonBox.button(i));
            buttons.put(i + 11, rightButtonBox.button(i));
        }
    }
    
    /**
     * The joystick buttons
     */
    public enum ButtonBoxButton {
        LEFT_1(1),
        LEFT_2(2),
        LEFT_3(3),
        LEFT_4(4),
        LEFT_5(5),
        LEFT_6(6),
        LEFT_7(7),
        LEFT_8(8),
        LEFT_9(9),
        LEFT_10(10),
        LEFT_11(11),
        RIGHT_1(12),
        RIGHT_2(13),
        RIGHT_3(14),
        RIGHT_4(15),
        RIGHT_5(16),
        RIGHT_6(17),
        RIGHT_7(18),
        RIGHT_8(19),
        RIGHT_9(20),
        RIGHT_10(21),
        RIGHT_11(22);

        public int VALUE;

        private ButtonBoxButton(int value) {
            this.VALUE = value;
        }
    }
}
