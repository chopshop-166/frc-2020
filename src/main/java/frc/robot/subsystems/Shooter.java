package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * What does it do? When the A button is pressed- it shoots a ball.
 * What modes does it have? Semi-Auto and DUMP.
 * what interactions does it have with other subsystems? Asks the Indexer and 'vision' if it's ready to shoot.
 * How is it triggered/OI? A semi-auto button A- or X for shoot all.
 * Does it store any state? No.
 * Sensors? No.
 */
public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor = map.shooterWheel();
    }
    
    public spinMotor(double speed) {
        return new StartEndCommand(() -> {
            shooterWheelMotor.set(speed);
        } () -> {
            shooterWheelMotor.stopMotor();
        },this);
    }
}