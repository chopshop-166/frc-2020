package frc.robot.maps;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.outputs.SwPIDSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

@RobotMapFor("Tempest")
public class TempestMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        final double distancePerPulse = (1.0 / 256.0) * (4.0 * Math.PI);
        return new DifferentialDriveMap() {
            @Override
            public EncodedSpeedController getLeft() {

                SendableSpeedController motors = SendableSpeedController.group(new WPI_TalonSRX(4),
                        new WPI_TalonSRX(1));
                return EncodedSpeedController.join(motors, new MockEncoder());

            }

            @Override
            public EncodedSpeedController getRight() {
                final SendableSpeedController rightGroup = SendableSpeedController.group(new WPI_TalonSRX(2),
                        new WPI_TalonSRX(3));
                final WEncoder encoder = new WEncoder(1, 0);

                encoder.setDistancePerPulse(distancePerPulse);
                return EncodedSpeedController.join(rightGroup, encoder);
            }
        };
    }

    @Override
    public IndexMap getIndexerMap() {
        return new IndexMap() {

            @Override
            public SendableSpeedController pierreMotor() {
                final Victor pierreMotor = new Victor(5);
                return SendableSpeedController.wrap(pierreMotor);
            }

            public BooleanSupplier frontIntakeIR() {
                AnalogTrigger frontIntakeIR = new AnalogTrigger(0);
                frontIntakeIR.setLimitsVoltage(1.2, 1.4);
                return frontIntakeIR::getTriggerState;
            }

            public BooleanSupplier bottomPierreIR() {
                AnalogTrigger bottomPierreIR = new AnalogTrigger(1);
                bottomPierreIR.setLimitsVoltage(1.2, 1.4);
                return bottomPierreIR::getTriggerState;
            }

            public BooleanSupplier topPierreIR() {
                AnalogTrigger topPierreIR = new AnalogTrigger(2);
                topPierreIR.setLimitsVoltage(1.8, 2.4);
                return topPierreIR::getTriggerState;
            }

            public BooleanSupplier backIntakeIR() {
                AnalogTrigger backIntakeIR = new AnalogTrigger(3);
                backIntakeIR.setLimitsVoltage(1.2, 1.4);
                return backIntakeIR::getTriggerState;
            }

        };
    }

    @Override
    public ShooterMap getShooterMap() {
        return new ShooterMap() {
            @Override
            public PIDSpeedController shooterWheel() {
                final Talon rollerMotor = new Talon(0);
                final PIDController pid = new PIDController(0, 0, 0);
                final DoubleSupplier measurement = new MockEncoder()::getRate;
                return new SwPIDSpeedController(rollerMotor, pid, measurement);
            }

            @Override
            public PIDSpeedController shooterWheel2() {
                final Talon rollerMotor = new Talon(1);
                final PIDController pid = new PIDController(0, 0, 0);
                final DoubleSupplier measurement = new MockEncoder()::getRate;
                return new SwPIDSpeedController(rollerMotor, pid, measurement);
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SendableSpeedController intake() {
                final Victor intakeMotor = new Victor(3);
                return SendableSpeedController.wrap(intakeMotor);
            }

        };

    }
}