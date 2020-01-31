package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.InstantCommand;

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

    public static class IndexMap {
        public SendableSpeedController setIndex() {
            return new MockSpeedController();
        }

        public SendableSpeedController quicklyOutput() {
            return new MockSpeedController();
        }

        public IndexMap getIndexMap() {
            return new IndexMap();
        }

        public SendableSpeedController indexMotor() {
            return null;
        }
    }

    public static class IntakeMap {
        public SendableSpeedController roller() {
            return new MockSpeedController();
        }

        public IDSolenoid deployPiston() {
            return new MockDSolenoid();
        }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ShooterMap {
        public SendableSpeedController shooterWheel() {
            return new MockSpeedController();
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