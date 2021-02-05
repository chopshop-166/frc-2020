package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.ModSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.chopshop166.chopshoplib.outputs.SwPIDSpeedController;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;

@RobotMapFor("Tempest")
public class TempestMap extends RobotMap {

    @Override
    public DriveKinematics getDriveMap() {
        final double distancePerPulse = (1.0 / 256.0) * (4.0 * Math.PI);
        return new DriveKinematics() {
            @Override
            public SmartSpeedController getLeft() {
                return SmartSpeedController.group(new WPI_TalonSRX(4), new WPI_TalonSRX(1));
            }

            @Override
            public SmartSpeedController getRight() {
                final WEncoder encoder = new WEncoder(1, 0);
                encoder.setDistancePerPulse(distancePerPulse);

                return new ModSpeedController(new SpeedControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(3)),
                        encoder);
            }
        };
    }

    @Override
    public IndexMap getIndexerMap() {
        return new IndexMap() {

            AnalogTrigger frontIntakeIR = new AnalogTrigger(0);
            AnalogTrigger bottomPierreIR = new AnalogTrigger(1);
            AnalogTrigger topPierreIR = new AnalogTrigger(2);
            AnalogTrigger backIntakeIR = new AnalogTrigger(3);

            @Override
            public SmartSpeedController pierreMotor() {
                final Victor pierreMotor = new Victor(5);
                return SmartSpeedController.wrap(pierreMotor);
            }

            public BooleanSupplier frontIntakeIR() {
                frontIntakeIR.setLimitsVoltage(1.2, 1.4);
                return frontIntakeIR::getTriggerState;
            }

            public BooleanSupplier bottomPierreIR() {
                bottomPierreIR.setLimitsVoltage(1.2, 1.4);
                return bottomPierreIR::getTriggerState;
            }

            public BooleanSupplier topPierreIR() {
                topPierreIR.setLimitsVoltage(1.8, 2.4);
                return topPierreIR::getTriggerState;
            }

            public BooleanSupplier backIntakeIR() {
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
                final Talon rollerMotor2 = new Talon(1);
                final SmartSpeedController bothRollers = SmartSpeedController.group(rollerMotor, rollerMotor2);
                final PIDController pid = new PIDController(0, 0, 0);
                return SwPIDSpeedController.velocity(bothRollers, pid);
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SmartSpeedController intake() {
                final Victor intakeMotor = new Victor(3);
                return SmartSpeedController.wrap(intakeMotor);
            }

        };

    }
}