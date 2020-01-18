package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

public interface RobotMap {
    public interface DriveMap {
        public SendableSpeedController left();

        public SendableSpeedController right();
    }

    public DriveMap getDriveMap();

    public interface IntakeMap {
        public SendableSpeedController roller();
    }

    public IntakeMap getIntakeMap();

    public interface ShooterMap {
        public SendableSpeedController shooterWheel();
    }

    public ShooterMap getShooterMap();

    public interface ControlPanelMap {
        public SendableSpeedController spinner();
    }

    public ControlPanelMap getControlPanelMap();

    public interface LiftMap {
        public SendableSpeedController elevator();
    }

    public LiftMap getLiftMap();
}