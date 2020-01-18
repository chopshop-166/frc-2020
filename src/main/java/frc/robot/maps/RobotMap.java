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

    public interface ControlPanelMap {
        public SendableSpeedController spinner();
    }

    public ControlPanelMap getControlPanelMap();
}