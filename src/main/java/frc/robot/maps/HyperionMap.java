package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

@RobotMapFor("Hyperion")
public class HyperionMap extends RobotMap {

    @Override
    public DriveKinematics getDriveMap() {
        return new DriveKinematics() {

            @Override
            public SmartSpeedController getRight() {
                return SmartSpeedController.group(new WPI_TalonSRX(1), new WPI_TalonSRX(2));
            }

            @Override
            public SmartSpeedController getLeft() {
                return SmartSpeedController.group(new WPI_TalonSRX(3), new WPI_TalonSRX(4));
            }
        };

    }
}