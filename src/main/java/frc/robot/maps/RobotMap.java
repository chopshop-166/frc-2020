package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.InvertDigitalInput;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

public interface RobotMap {
    public interface DriveMap {
        public SendableSpeedController left();

        public SendableSpeedController right();
    }

    public DriveMap getDriveMap();

    public interface IntakeMap {
        default public SendableSpeedController roller() {
            return new MockSpeedController();
        }
    }

    public IntakeMap getIntakeMap();

    public interface ShooterMap {
        default public SendableSpeedController shooterWheel() {
            return new MockSpeedController();
        }
    }

    public ShooterMap getShooterMap();

    public interface ControlPanelMap {
        default public SendableSpeedController spinner() {
            return new MockSpeedController();
        }
    }

    public ControlPanelMap getControlPanelMap();

    // TODO replace these values once true ports are determined

    public interface LiftMap {
        default public SendableSpeedController elevatorLeft() {
            return new MockSpeedController();
        }

        default public SendableSpeedController elevatorRight() {
            return new MockSpeedController();
        }

        default public Solenoid liftBrake() {
            return new Solenoid(0);
        }

        default public InvertDigitalInput upperLiftLimit() {
            return new InvertDigitalInput(0);
        }

        default public InvertDigitalInput lowerLiftLimit() {
            return new InvertDigitalInput(0);
        }

        default public Encoder getLeftEncoder() {
            return new Encoder(6, 7);
        }

        default public Encoder getRightEncoder() {
            return new Encoder(8, 9);
        }
    }

    public LiftMap getLiftMap();

}