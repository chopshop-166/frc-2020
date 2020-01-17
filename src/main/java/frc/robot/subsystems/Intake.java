package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Intake extends SubsystemBase {
    private SendableSpeedController roller;

    public Intake(RobotMap.IntakeMap map) {
        super();
        roller = map.roller();
    }
}