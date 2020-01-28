package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * What does it do? Starts spinning the belt after the button is toggled on.
 * 
 * What modes does it have? On and off.
 * 
 * what interactions does it have with other subsystems? Tells the indexer if the shooter is up to speed.
 * 
 * How is it triggered/OI? A Toggle Conveyer button (y).
 * 
 * Does it store any state? Spinning or not spinning.
 * 
 * Sensors? No.
 */

public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor = map.shooterWheel();
    }

    public 
}