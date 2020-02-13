package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

@RobotMapFor("Tempest")
public class TempestMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                final SendableSpeedController rightGroup = SendableSpeedController.group(new WPI_TalonSRX(2),
                        new WPI_TalonSRX(3));
                final WEncoder encoder = new WEncoder(1, 0);
                final double distancePerPulse = (1.0 / 4096.0) * (4.0 * Math.PI);
                encoder.setDistancePerPulse(distancePerPulse);
                return EncodedSpeedController.join(rightGroup, encoder);
            }

            @Override
            public EncodedSpeedController getLeft() {
                final SendableSpeedController leftGroup = SendableSpeedController.group(new WPI_TalonSRX(1),
                        new WPI_TalonSRX(4));
                final WEncoder encoder = new WEncoder(3, 2);
                final double distancePerPulse = (1.0 / 4096.0) * (4.0 * Math.PI);
                encoder.setDistancePerPulse(distancePerPulse);
                return EncodedSpeedController.join(leftGroup, encoder);
            }
        };
    }
}
