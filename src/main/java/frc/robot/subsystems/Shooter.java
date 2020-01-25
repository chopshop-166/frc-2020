package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * What does it do?
 * 
 * What modes does it have?
 * 
 * what interactions does it have with other subsystems?
 * 
 * How is it triggered -> OI?
 * 
 * Does it store any state?
 * 
 * Sensors?
 * 
 */
public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor = map.shooterWheel();
    }

}