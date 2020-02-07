package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

@RobotMapFor("Hyperion")
public class HyperionMap extends RobotMap {

    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                SendableSpeedController rightGroup = SendableSpeedController.group(new WPI_TalonSRX(1),
                        new WPI_TalonSRX(2));
                return EncodedSpeedController.join(rightGroup, new MockEncoder());
            }

            @Override
            public EncodedSpeedController getLeft() {
                SendableSpeedController leftGroup = SendableSpeedController.group(new WPI_TalonSRX(3),
                        new WPI_TalonSRX(4));
                return EncodedSpeedController.join(leftGroup, new MockEncoder());
            }
        };

    }
}