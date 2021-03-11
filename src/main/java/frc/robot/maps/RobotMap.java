package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockPIDSpeedController;
import com.chopshop166.chopshoplib.outputs.MockSolenoid;
import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class RobotMap {

    public void setBAGCurrentLimits(TalonSRX talon) {
        talon.configContinuousCurrentLimit(15, 0);
        talon.configPeakCurrentLimit(25, 0);
        talon.configPeakCurrentDuration(100, 0);
        talon.enableCurrentLimit(true);
    }

    public static interface DriveKinematics extends DifferentialDriveMap {
        default DifferentialDriveKinematics getKinematics() {
            return new DifferentialDriveKinematics(0.642);
        }
    }

    public DriveKinematics getDriveMap() {
        return new DriveKinematics() {

            @Override
            public SmartSpeedController getRight() {
                return new MockSpeedController();
            }

            @Override
            public SmartSpeedController getLeft() {
                return new MockSpeedController();
            }
        };
    }

    public static class IntakeMap {

        public SmartSpeedController intake() {
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

    public static class IndexMap {
        public SmartSpeedController singulator() {
            return new MockSpeedController();
        }

        public SmartSpeedController pierreMotor() {
            return new MockSpeedController();
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