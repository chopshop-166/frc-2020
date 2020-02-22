package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

@RobotMapFor("Hyperion")
public class HyperionMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public SendableSpeedController getRight() {
                return SendableSpeedController.group(new WPI_TalonSRX(1), new WPI_TalonSRX(2));
            }

            @Override
            public SendableSpeedController getLeft() {
                return SendableSpeedController.group(new WPI_TalonSRX(3), new WPI_TalonSRX(4));
            }
        };

    }
}