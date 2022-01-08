package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

@RobotMapFor("Hyperion")
public class HyperionMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap(
                new SmartMotorController(new WPI_TalonSRX(4), new WPI_TalonSRX(1)),
                new SmartMotorController(new WPI_TalonSRX(2), new WPI_TalonSRX(3)),
                1.0);

    }
}