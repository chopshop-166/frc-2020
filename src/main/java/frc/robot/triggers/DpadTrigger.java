package frc.robot.triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A trigger that responds to the Dpad of an xbox controller
 */
public class DpadTrigger extends Trigger {

    /**
     * Creates a Dpad Trigger
     * 
     * @param controller The controller to use
     * @param angle      The angle of dpad button (0 is up)
     */
    public DpadTrigger(XboxController controller, int angle) {
        super(() -> {
            return (controller.getPOV(0) == angle);
        });
    }

    /**
     * Creates a trigger for the up button
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadUp(XboxController controller) {
        return new DpadTrigger(controller, 0);
    }

    /**
     * Creates a trigger for the up and right buttons
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadUpRight(XboxController controller) {
        return new DpadTrigger(controller, 45);
    }

    /**
     * Creates a trigger for the right button
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadRight(XboxController controller) {
        return new DpadTrigger(controller, 90);
    }

    /**
     * Creates a trigger for the right and down buttons
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadDownRight(XboxController controller) {
        return new DpadTrigger(controller, 135);
    }

    /**
     * Creates a trigger for the down button
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadDown(XboxController controller) {
        return new DpadTrigger(controller, 180);
    }

    /**
     * Creates a trigger for the down and left buttons
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadDownLeft(XboxController controller) {
        return new DpadTrigger(controller, 225);
    }

    /**
     * Creates a trigger for the left button
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadLeft(XboxController controller) {
        return new DpadTrigger(controller, 270);
    }

    /**
     * Creates a trigger for the up and left buttons
     * 
     * @param controller The controller to use
     * @return The Dpad Trigger
     */
    public static DpadTrigger DpadUpLeft(XboxController controller) {
        return new DpadTrigger(controller, 315);
    }
}