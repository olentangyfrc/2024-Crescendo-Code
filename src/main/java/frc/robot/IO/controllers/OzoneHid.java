package frc.robot.IO.controllers;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class OzoneHid {
    protected static final double DEADZONE = 0.02;
    protected static final double NOMINAL_ANALOG_VALUE = 0.5;

    protected Map<Integer, Trigger> buttons;

    protected abstract void populateButtons();

    /**
     * Bind a command to a button on the HID interface.
     * 
     * @param type The type of binding to perform
     * @param xboxButton The button on the HID interface to bind to
     * @param command The command to bind
     */
    public void bind(ButtonActionType type, int buttonNum, Command command) {
        Trigger button = buttons.get(buttonNum);
        
        switch(type) {
            case TOGGLE_WHEN_PRESSED:
                button.toggleOnTrue(command);
                break;
            case WHEN_HELD:
                button.whileTrue(command);
                break;
            case WHEN_PRESSED:
                button.onTrue(command);
                break;
            case WHEN_RELEASED:
                button.onFalse(command);
                break;
            case WHILE_HELD:
                button.whileTrue(new RepeatCommand(command));
                break;
        }
    }

    /**
     * Determine if a button is currently pressed
     * 
     * @param button The button to test
     * @return True if the button is pressed, else, false.
     */
    public boolean isButtonPressed(int buttonNum) {
        return buttons.get(buttonNum).getAsBoolean();
    }

    /**
     * Types of button bindings
     * 
     * @see <a href="https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html">See the button class for definitions of types.</a>
     */
    public enum ButtonActionType {
        WHEN_HELD,
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED;
    }
}
