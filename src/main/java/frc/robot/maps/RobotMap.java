package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

public interface RobotMap {
    public interface DriveMap {
        default public SendableSpeedController left() {
            return new MockSpeedController();
        }

        default public SendableSpeedController right() {
            return new MockSpeedController();
        }
    }

    default public DriveMap getDriveMap() {
        return new DriveMap() {
        };
    }

    public interface IntakeMap {
        default public SendableSpeedController roller() {
            return new MockSpeedController();
        }
    }

    default public IntakeMap getIntakeMap() {
        return new IntakeMap() {
        };
    }

    public interface ShooterMap {
        default public SendableSpeedController shooterWheel() {
            return new MockSpeedController();
        }
    }

    default public ShooterMap getShooterMap() {
        return new ShooterMap() {
        };
    }

    public interface ControlPanelMap {
        default public SendableSpeedController spinner() {
            return new MockSpeedController();
        }
    }

    default public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
        };
    }

    public interface LiftMap {
        default public SendableSpeedController elevator() {
            return new MockSpeedController();
        }
    }

    default public LiftMap getLiftMap() {
        return new LiftMap() {
        };
    }
}