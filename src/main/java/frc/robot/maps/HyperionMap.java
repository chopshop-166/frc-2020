package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class HyperionMap implements RobotMap {

    @Override
    public DriveMap getDriveMap() {
        return new DriveMap() {
            @Override
            public SendableSpeedController left() {
                return SendableSpeedController.wrap(new SpeedControllerGroup(new WPI_TalonSRX(4), new WPI_TalonSRX(3)));
            }

            @Override
            public SendableSpeedController right() {
                return SendableSpeedController.wrap(new SpeedControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(1)));
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public MockSpeedController roller() {
                return new MockSpeedController();
            }
        };

    }

    @Override
    public ShooterMap getShooterMap() {
        return new ShooterMap() {
            @Override
            public MockSpeedController shooterWheel() {
                return new MockSpeedController();
            }
        };
    }

    @Override
    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
            @Override
            public MockSpeedController spinner() {
                return new MockSpeedController();
            }
        };
    }

    @Override
    public LiftMap getLiftMap() {
        return new LiftMap() {
            @Override
            public MockSpeedController elevator() {
                return new MockSpeedController();
            }
        };
    }
}