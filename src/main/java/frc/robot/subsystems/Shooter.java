package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor;

    private static final double shooterWheelMotorSpeed = 1;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor = map.shooterWheel();
    }

    public CommandBase up() {
        return new StartEndCommand(() -> {
            shooterWheelMotor.set(-shooterWheelMotorSpeed);
        }, () -> {
            shooterWheelMotor.stopMotor();
        }, this);
    }

    public CommandBase down() {
        return new StartEndCommand(() -> {
            shooterWheelMotor.set(shooterWheelMotorSpeed);
        }, () -> {
            shooterWheelMotor.stopMotor();
        }, this);
    }
}