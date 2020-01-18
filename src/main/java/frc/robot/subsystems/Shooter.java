package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Shooter extends SubsystemBase {

    private SendableSpeedController shooterWheelMotor;

    public Shooter(RobotMap.ShooterMap map) {
        super();
        shooterWheelMotor = map.shooterWheel();
    }

}