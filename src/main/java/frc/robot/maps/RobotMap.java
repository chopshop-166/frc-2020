package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.pneumatics.IDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.ISolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockDSolenoid;
import com.chopshop166.chopshoplib.pneumatics.MockSolenoid;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;

public class RobotMap {

    public void setBAGCurrentLimits(TalonSRX talon) {
        talon.configContinuousCurrentLimit(15, 0);
        talon.configPeakCurrentLimit(25, 0);
        talon.configPeakCurrentDuration(100, 0);
        talon.enableCurrentLimit(true);
    }

    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap(new SmartMotorController(), new SmartMotorController(), 1.0);
    }

    public static class IntakeMap {

        public SmartMotorController intake() {
            return new SmartMotorController();
        }

        public IDSolenoid deployIntake() {
            return new MockDSolenoid();
        }

    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ShooterMap {
        public SmartMotorController shooterWheel() {
            return new SmartMotorController();
        }

        public double shooterHeight() {
            return 0;
        }

    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();

    }

    public static class ControlPanelMap {
        public SmartMotorController spinner() {
            return new SmartMotorController();
        }
    }

    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap();
    }

    public static class LiftMap {

        public SmartMotorController elevator() {
            return new SmartMotorController();
        }

        public ISolenoid liftBrake() {
            return new MockSolenoid();
        }

        public BooleanSupplier upperLiftLimit() {
            return new MockDigitalInput()::getAsBoolean;
        }

        public BooleanSupplier lowerLiftLimit() {
            return new MockDigitalInput()::getAsBoolean;
        }

        public IEncoder getLiftEncoder() {
            return new MockEncoder();
        }

    }

    public LiftMap getLiftMap() {
        return new LiftMap();
    }

    public static class IndexMap {
        public SmartMotorController singulator() {
            return new SmartMotorController();
        }

        public SmartMotorController pierreMotor() {
            return new SmartMotorController();
        }

        public BooleanSupplier frontIntakeIR() {
            return new MockDigitalInput()::getAsBoolean;
        }

        public BooleanSupplier bottomPierreIR() {
            return new MockDigitalInput()::getAsBoolean;
        }

        public BooleanSupplier topPierreIR() {
            return new MockDigitalInput()::getAsBoolean;
        }

        public BooleanSupplier backIntakeIR() {
            return new MockDigitalInput()::getAsBoolean;
        }
    }

    public IndexMap getIndexerMap() {
        return new IndexMap();
    }

    public static class LEDMap {

        public ISolenoid visionRingLightSolenoid() {
            return new MockSolenoid();
        }

        public AddressableLED robotLED() {
            return new AddressableLED(1);
        }

        public int RobotBufferLength() {
            return 0;
        }

    }

    public LEDMap getLEDMap() {
        return new LEDMap();
    }

}