package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockPIDSpeedController;
import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;

public class RobotMap {

    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                return getMock();
            }

            @Override
            public EncodedSpeedController getLeft() {
                return getMock();
            }

            private EncodedSpeedController getMock() {
                return EncodedSpeedController.join(new MockSpeedController(), new MockEncoder());
            }
        };
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
        public PIDSpeedController shooterWheel() {
            return new MockPIDSpeedController();
        }

        public double shooterHeight() {
            return 0;
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

        public BooleanSupplier frontIntakeIR() {
            return () -> false;
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
    }

    public IndexMap getIndexerMap() {
        return new IndexMap();
    }

}