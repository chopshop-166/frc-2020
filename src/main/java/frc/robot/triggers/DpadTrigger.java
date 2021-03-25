package frc.robot.triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DpadTrigger extends Trigger {

    public DpadTrigger(XboxController controller, int angle) {
        super(() -> {
            return (controller.getPOV(0) == angle);
        });
    }

    public static DpadTrigger DpadUp(XboxController controller) {
        return new DpadTrigger(controller, 0);
    }

    public static DpadTrigger DpadDown(XboxController controller) {
        return new DpadTrigger(controller, 180);
    }

    public static DpadTrigger DpadLeft(XboxController controller) {
        return new DpadTrigger(controller, 270);
    }

    public static DpadTrigger DpadRight(XboxController controller) {
        return new DpadTrigger(controller, 90);
    }
}