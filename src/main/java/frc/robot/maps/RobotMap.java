package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockAnalogInput;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;

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
        public SendableSpeedController intake() {
            return new MockSpeedController();
        }

        public SendableSpeedController singulator() {
            return new MockSpeedController();
        }

        public SendableSpeedController pierre() {
            return new MockSpeedController();
        }

        public IDSolenoid deployIntake() {
            return new MockDSolenoid();
        }

    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ShooterMap {
        public SendableSpeedController shooterWheel1() {
            return new MockSpeedController();
        }

        public SendableSpeedController shooterWheel2() {
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

    public static class IndexMap {
        public SendableSpeedController singulator() {
            return new MockSpeedController();
        }

        public SendableSpeedController pierreMotor() {
            return new MockSpeedController();
        }

        public BooleanSupplier frontIntakeIR(){
            return ()-> false;
        }

        public BooleanSupplier bottomPierreIR() {
            return () -> false;
        }
        
        public BooleanSupplier topPierreIR() {
            return () -> false;
        }

        public BooleanSupplier backIntakeIR() {
            return () -> false;
        }


        // AnalogTrigger frontIntakeIR = new AnalogTrigger(0);
        // AnalogTrigger bottomPierreIR = new AnalogTrigger(1);
        // AnalogTrigger topPierreIR = new AnalogTrigger(2);
        // AnalogTrigger backIntakeIR = new AnalogTrigger(3);
    }

    public IndexMap getIndexerMap() {
        return new IndexMap();
    }

}