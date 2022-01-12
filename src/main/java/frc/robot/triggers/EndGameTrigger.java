package frc.robot.triggers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndGameTrigger extends Trigger {

    public EndGameTrigger(double matchTime) {
        super(() -> {
            double time = Timer.getMatchTime();
            return time >= matchTime || !DriverStation.isFMSAttached();
        });
    }

}