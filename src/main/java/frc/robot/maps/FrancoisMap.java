package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.ModSpeedController;
import com.chopshop166.chopshoplib.outputs.Modifier;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.chopshop166.chopshoplib.outputs.WSolenoid;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.chopshop166.chopshoplib.sensors.WDigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

@RobotMapFor("Francois")
public class FrancoisMap extends RobotMap {
    // controlPanel is defined here due to the gyro being plugged into this speed
    // controller as well as the control panel motor
    WPI_TalonSRX controlPanel = new WPI_TalonSRX(43);

    @Override
    public DriveKinematics getDriveMap() {
        // 1/12.27 is the gear ratio multiplied by the circumfrence of the wheel
        // multiplied by a correcion factor found by comparing distance traveled to
        // encoder values
        final int averageCount = 15;
        final double distancePerRev = Units.inchesToMeters((1.0 / 12.27) * (6.0 * Math.PI)) * 0.9533562075675308;

        return new RobotMap.DriveKinematics() {
            CANSparkMax rightLeader = new CANSparkMax(27, MotorType.kBrushless);
            CANSparkMax rightFollower = new CANSparkMax(22, MotorType.kBrushless);

            CANSparkMax leftLeader = new CANSparkMax(29, MotorType.kBrushless);
            CANSparkMax leftFollower = new CANSparkMax(25, MotorType.kBrushless);

            @Override
            public DifferentialDriveKinematics getKinematics() {
                return new DifferentialDriveKinematics(0.642);
            }

            @Override
            public SmartSpeedController getRight() {
                // We invert the motor so the controller outputs are aligned
                rightLeader.setInverted(true);
                rightFollower.follow(rightLeader);

                PIDSparkMax sendLeader = new PIDSparkMax(rightLeader);
                sendLeader.getEncoder().setPositionScaleFactor(distancePerRev);
                // The distance being divided by 60 essentially takes distance and converts it
                // to velocity (in m/s)
                sendLeader.getEncoder().setVelocityScaleFactor(distancePerRev / 60);
                SendableRegistry.add(sendLeader.getEncoder(), "Right Drive");
                SendableRegistry.enableLiveWindow(sendLeader.getEncoder());
                return new ModSpeedController(sendLeader, sendLeader.getEncoder(),
                        Modifier.rollingAverage(averageCount));
            }

            @Override
            public SmartSpeedController getLeft() {
                leftFollower.follow(leftLeader);

                PIDSparkMax sendLeader = new PIDSparkMax(leftLeader);
                sendLeader.getEncoder().setPositionScaleFactor(distancePerRev);
                // The distance being divided by 60 essentially takes distance and converts it
                // to velocity (in m/s)
                sendLeader.getEncoder().setVelocityScaleFactor(distancePerRev / 60);
                SendableRegistry.add(sendLeader.getEncoder(), "Left Drive");
                SendableRegistry.enableLiveWindow(sendLeader.getEncoder());
                return new ModSpeedController(sendLeader, sendLeader.getEncoder(),
                        Modifier.rollingAverage(averageCount));

            }

            @Override
            public GyroBase getGyro() {
                return new PigeonGyro(controlPanel);
            }
        };
    };

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SmartSpeedController intake() {
                return SmartSpeedController.wrap(new WPI_TalonSRX(42));
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
            PIDSparkMax pidLeader = new PIDSparkMax(leader);

            @Override
            public PIDSparkMax shooterWheel() {
                leader.setIdleMode(IdleMode.kCoast);
                follower.setIdleMode(IdleMode.kCoast);
                leader.setInverted(true);
                follower.follow(leader, true);

                pidLeader.setP(0.00038);
                pidLeader.setI(0.0000001);
                pidLeader.setD(0);
                pidLeader.setF(0.00017);

                // kp = .00045, kF = .0002

                return pidLeader;
            }

        };
    }

    @Override
    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
            @Override
            public SmartSpeedController spinner() {
                setBAGCurrentLimits(controlPanel);
                return SmartSpeedController.wrap(controlPanel);
            }
        };
    }

    @Override
    public IndexMap getIndexerMap() {
        return new IndexMap() {
            AnalogTrigger topPierreIR = new AnalogTrigger(0);
            AnalogTrigger bottomPierreIR = new AnalogTrigger(1);
            AnalogTrigger backIntakeIR = new AnalogTrigger(2);
            AnalogTrigger frontIntakeIR = new AnalogTrigger(3);

            @Override
            public SmartSpeedController pierreMotor() {
                final WPI_TalonSRX pierreMotor = new WPI_TalonSRX(40);
                setBAGCurrentLimits(pierreMotor);
                return SmartSpeedController.wrap(pierreMotor);
            }

            public SmartSpeedController singulator() {
                final WPI_TalonSRX singulator = new WPI_TalonSRX(41);
                setBAGCurrentLimits(singulator);
                return SmartSpeedController.wrap(singulator);
            }

            // TODO Find values for voltage limits
            public BooleanSupplier topPierreIR() {
                topPierreIR.setLimitsVoltage(1.1, 2);
                return topPierreIR::getTriggerState;
            }

            // TODO Find values for voltage limits; test 1-3.5 and then test 1.5-3.5
            public BooleanSupplier bottomPierreIR() {
                bottomPierreIR.setLimitsVoltage(1.0, 1.2);
                return bottomPierreIR::getTriggerState;
            }

            public BooleanSupplier backIntakeIR() {
                backIntakeIR.setLimitsVoltage(1.2, 2.6);
                return backIntakeIR::getTriggerState;
            }

            public BooleanSupplier frontIntakeIR() {
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
            PIDSparkMax pidLeader = new PIDSparkMax(leader);
            WDigitalInput upperLimit = new WDigitalInput(0);
            WDigitalInput lowerLimit = new WDigitalInput(1);
            double distancePerRev = (1.0 / 81.0) * (2.551 * Math.PI);

            @Override
            public PIDSparkMax elevator() {
                leader.setInverted(true);
                follower.follow(leader, true);
                leader.setIdleMode(IdleMode.kBrake);
                follower.setIdleMode(IdleMode.kBrake);

                return pidLeader;
            }

            @Override
            public ISolenoid liftBrake() {
                WSolenoid brake = new WSolenoid(0);
                return brake;
            }

            @Override
            public BooleanSupplier upperLiftLimit() {
                upperLimit.setInverted(true);
                SmartDashboard.putData(upperLimit);
                return upperLimit::get;
            }

            @Override
            public BooleanSupplier lowerLiftLimit() {
                lowerLimit.setInverted(true);
                SmartDashboard.putData(lowerLimit);
                return lowerLimit::get;
            }

            @Override
            public IEncoder getLiftEncoder() {
                pidLeader.getEncoder().setPositionScaleFactor(distancePerRev);
                return pidLeader.getEncoder();
            }
        };
    }

    @Override
    public LEDMap getLEDMap() {
        return new LEDMap() {

            public ISolenoid visionRingLightSolenoid() {
                return new WSolenoid(7);
            }

        };
    }

}