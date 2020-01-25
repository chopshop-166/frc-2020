package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class HyperionMap implements RobotMap {

    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                SendableSpeedController rightGroup = SendableSpeedController.group(new WPI_TalonSRX(3),
                        new WPI_TalonSRX(4));
                return EncodedSpeedController.join(rightGroup, new MockEncoder());
            }

            @Override
            public EncodedSpeedController getLeft() {
                SendableSpeedController leftGroup = SendableSpeedController.group(new WPI_TalonSRX(2),
                        new WPI_TalonSRX(1));
                return EncodedSpeedController.join(leftGroup, new MockEncoder());
            }
        };

    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
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
}