package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.ModSpeedController;
import com.chopshop166.chopshoplib.outputs.Modifier;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.outputs.SparkMaxSendable;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.chopshop166.chopshoplib.outputs.WSolenoid;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.InvertDigitalInput;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.chopshop166.chopshoplib.sensors.SparkMaxEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.GyroBase;

@RobotMapFor("Francois")
public class FrancoisMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        // 1/12.27 is the gear ratio multiplied by the circumfrence of the wheel
        final int averageCount = 15;
        final double distancePerPulse = (1.0 / 12.27) * (6.0 * Math.PI);
        return new DifferentialDriveMap() {
            CANSparkMax rightLeader = new CANSparkMax(27, MotorType.kBrushless);
            CANSparkMax rightFollower = new CANSparkMax(22, MotorType.kBrushless);

            CANSparkMax leftLeader = new CANSparkMax(29, MotorType.kBrushless);
            CANSparkMax leftFollower = new CANSparkMax(25, MotorType.kBrushless);

            @Override
            public SendableSpeedController getRight() {
                rightFollower.follow(rightLeader);

                rightLeader.getEncoder().setPositionConversionFactor(distancePerPulse);
                SparkMaxSendable sendleader = new SparkMaxSendable(rightLeader);

                return new ModSpeedController(sendleader, Modifier.rollingAverage(averageCount));
            }

            @Override
            public SendableSpeedController getLeft() {
                leftFollower.follow(leftLeader);

                leftLeader.getEncoder().setPositionConversionFactor(distancePerPulse);
                SparkMaxSendable sendleader = new SparkMaxSendable(leftLeader);

                return new ModSpeedController(sendleader, Modifier.rollingAverage(averageCount));

            }

            @Override
            public GyroBase getGyro() {
                return new PigeonGyro(new WPI_TalonSRX(42));
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SendableSpeedController intake() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(42));
            }

            @Override
            public WDSolenoid deployIntake() {
                return new WDSolenoid(1, 2);
            }
        };
    }

    @Override
    public ShooterMap getShooterMap() {
        return new ShooterMap() {
            CANSparkMax leader = new CANSparkMax(23, MotorType.kBrushless);
            CANSparkMax follower = new CANSparkMax(26, MotorType.kBrushless);

            @Override
            public PIDSparkMax shooterWheel() {
                leader.setInverted(true);
                follower.follow(leader, true);

                return new PIDSparkMax(leader);
            }
        };
    }

    @Override
    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
            @Override
            public SendableSpeedController spinner() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(49));
            }
        };
    }

    @Override
    public IndexMap getIndexerMap() {
        return new IndexMap() {

            @Override
            public SendableSpeedController pierreMotor() {
                final WPI_TalonSRX pierreMotor = new WPI_TalonSRX(40);
                return SendableSpeedController.wrap(pierreMotor);
            }

            public SendableSpeedController singulator() {
                final WPI_TalonSRX singulator = new WPI_TalonSRX(41);
                singulator.setInverted(true);
                return SendableSpeedController.wrap(singulator);
            }

            public BooleanSupplier topPierreIR() {
                AnalogTrigger topPierreIR = new AnalogTrigger(0);
                topPierreIR.setLimitsVoltage(1.2, 1.4);
                return topPierreIR::getTriggerState;
            }

            public BooleanSupplier bottomPierreIR() {
                AnalogTrigger bottomPierreIR = new AnalogTrigger(1);
                bottomPierreIR.setLimitsVoltage(1.2, 1.4);
                return bottomPierreIR::getTriggerState;
            }

            public BooleanSupplier backIntakeIR() {
                AnalogTrigger backIntakeIR = new AnalogTrigger(2);
                backIntakeIR.setLimitsVoltage(1.2, 2.6);
                return backIntakeIR::getTriggerState;
            }

            public BooleanSupplier frontIntakeIR() {
                AnalogTrigger frontIntakeIR = new AnalogTrigger(3);
                frontIntakeIR.setLimitsVoltage(1.2, 1.4);
                return frontIntakeIR::getTriggerState;
            }

        };
    }

    @Override
    public LiftMap getLiftMap() {
        return new LiftMap() {
            CANSparkMax follower = new CANSparkMax(21, MotorType.kBrushless);
            CANSparkMax leader = new CANSparkMax(28, MotorType.kBrushless);
            InvertDigitalInput upperLimit = new InvertDigitalInput(0);

            @Override
            public PIDSparkMax elevator() {
                leader.setInverted(true);
                follower.follow(leader, true);
                leader.setIdleMode(IdleMode.kBrake);
                follower.setIdleMode(IdleMode.kBrake);

                return new PIDSparkMax(leader);
            }

            @Override
            public ISolenoid liftBrake() {
                WSolenoid brake = new WSolenoid(0);
                return brake;
            }

            @Override
            public BooleanSupplier upperLiftLimit() {
                upperLimit.setInverted(true);
                return upperLimit::get;
            }

            @Override
            public BooleanSupplier lowerLiftLimit() {
                return upperLimit::get;
            }

            @Override
            public IEncoder getLiftEncoder() {
                SparkMaxEncoder getEncoder = new SparkMaxEncoder(leader.getEncoder());
                getEncoder.setScaleFactor(1);
                return getEncoder;
            }
        };
    }

}
