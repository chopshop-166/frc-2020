package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

public class RobotMap {
    public static class DriveMap {
        public SendableSpeedController left() {
            return new MockSpeedController();
        }

        public SendableSpeedController right() {
            return new MockSpeedController();
        }
    }

    public DriveMap getDriveMap() {
        return new DriveMap();
    }

    public static class IntakeMap {
        public SendableSpeedController roller() {
            return new MockSpeedController();
        }
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ShooterMap {
        public SendableSpeedController shooterWheel() {
            return new MockSpeedController();
        }

        public double shooterHeight() {
            return 45;
        }
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();

    }

    public static class ControlPanelMap {
        public SendableSpeedController spinner() {
            return new MockSpeedController();
        }
    }

    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap();
    }

    public static class LiftMap {
        public SendableSpeedController elevator() {
            return new MockSpeedController();
        }
    }

    public LiftMap getLiftMap() {
        return new LiftMap();
    }
}