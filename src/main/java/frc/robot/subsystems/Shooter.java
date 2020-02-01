package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * What does it do? When the A button is pressed- it shoots a ball. What modes
 * does it have? Semi-Auto and DUMP. what interactions does it have with other
 * subsystems? Asks the Indexer and 'vision' if it's ready to shoot. How is it
 * triggered/OI? A semi-auto button A- or X for shoot all. Does it store any
 * state? No. Sensors? No.
 */
public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor1;
    private SendableSpeedController shooterWheelMotor2;

    private double speed = -.85;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor1 = map.shooterWheel1();
        shooterWheelMotor2 = map.shooterWheel2();
    }

    public CommandBase spinMotor() {
        return new InstantCommand(() -> {
            shooterWheelMotor1.set(speed);
            shooterWheelMotor2.set(speed);
        }, this);
    }

    public CommandBase stopMotor() {
        return new InstantCommand(() -> {
            shooterWheelMotor1.stopMotor();
            shooterWheelMotor2.stopMotor();
        }, this);
    }
}