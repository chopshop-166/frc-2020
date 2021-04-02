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
    public DpadTrigger(XboxController controller, DpadDirection angle) {
        super(() -> {
            return (controller.getPOV(0) == angle.getAngle());
        });
    }

    public enum DpadDirection {
        Up(0), UpRight(45), Right(90), DownRight(135), Down(180), DownLeft(225), Left(270), LeftUp(315);

        private int dPadRotation;

        // Returning an interger to compare whether we're in the right place or not
        private int getAngle() {
            return this.dPadRotation;
        }

        // Returning the level the lift is at (top middle or bottom)
        private DpadDirection(int rotation) {
            this.dPadRotation = rotation;
        }
    }
}