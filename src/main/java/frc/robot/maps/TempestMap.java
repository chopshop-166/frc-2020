package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TempestMap implements RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                SendableSpeedController rightGroup = SendableSpeedController.group(new WPI_TalonSRX(1),
                        new WPI_TalonSRX(4));
                return EncodedSpeedController.join(rightGroup, new MockEncoder());
            }

            @Override
            public EncodedSpeedController getLeft() {
                SendableSpeedController leftGroup = SendableSpeedController.group(new WPI_TalonSRX(2),
                        new WPI_TalonSRX(3));
                return EncodedSpeedController.join(leftGroup, new MockEncoder());
            }
        };

    }

    @Override
    public ShooterMap getShooterMap() {
        return new ShooterMap() {
        };
    }

    @Override
    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
        };
    }

    @Override
    public LiftMap getLiftMap() {
        return new LiftMap() {
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        // TODO Auto-generated method stub
        return null;
    }
}