package frc.robot.IO;

import java.util.HashMap;
import java.util.Map;

import frc.robot.IO.controllers.ButtonBox;
import frc.robot.IO.controllers.OzoneController;

/**
 * The IO class manages the driver input to the robot. This includes getting axis values and binding buttons.
 */
public class IO {

    private static final int MAIN_XBOX_PORT = 0;
    private static final int AUX_XBOX_PORT = 1;

    public static final int LEFT_BUTTON_BOX_PORT = 3;
    public static final int RIGHT_BUTTON_BOX_PORT = 4;


    private ButtonBox buttonBox;

    private Map<XboxControllerName, OzoneController> xboxControllers = new HashMap<>();

    // Code for singleton
    private static IO instance;

    /**
     * Get the pre-existing instance of the IO class, or create one if there isn't one.
     * 
     * @return An instance of the IO class
     */
    public static IO getInstance() {
        if(instance == null) {
            instance = new IO();
            instance.init();
        }
        return instance;
    }

    /**
     * Initialize or reinitialize the IO class.
     * <p>
     * This does not need to be called when initializing with the getInstance() method.
     */
    public void init() {
        xboxControllers.put(XboxControllerName.MAIN, new OzoneController(MAIN_XBOX_PORT));
        xboxControllers.put(XboxControllerName.AUX, new OzoneController(AUX_XBOX_PORT));
    }

    /**
     * 
     * @return the main xbox controller
     */
    public OzoneController getController() {
        return xboxControllers.get(XboxControllerName.MAIN);
    }

    /**
     * 
     * @param controllerName the controller you want returned
     * @return the controller specified
     */
    public OzoneController getController(XboxControllerName controllerName) {
        return xboxControllers.get(controllerName);
    }

    public ButtonBox getButtonBox() {
        return buttonBox;
    }

    public enum XboxControllerName {
        MAIN,
        AUX
    }
}