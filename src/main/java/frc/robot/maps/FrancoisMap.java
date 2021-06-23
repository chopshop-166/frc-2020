package frc.robot.maps;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.ModSpeedController;
import com.chopshop166.chopshoplib.outputs.Modifier;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.chopshop166.chopshoplib.outputs.WSolenoid;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.logger.RobotLogger;
import frc.robot.logger.SubsystemLogger;

@RobotMapFor("Francois")
public class FrancoisMap extends RobotMap {
    // controlPanel is defined here due to the gyro being plugged into this speed
    // controller as well as the control panel motor
    WPI_TalonSRX gyroTalon = new WPI_TalonSRX(43);

    public FrancoisMap(final RobotLogger logger) {
        super(logger);
    }

    @Override
    public DriveKinematics getDriveMap() {
        // 1/12.27 is the gear ratio multiplied by the circumfrence of the wheel
        // multiplied by a correcion factor found by comparing distance traveled to
        // encoder values
        final int averageCount = 15;
        final double distancePerRev = Units.inchesToMeters((1.0 / 12.27) * (6.0 * Math.PI)) * 0.9533562075675308;
        final SubsystemLogger driveLogger = logger.addSubsystem("Drive");

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
                PIDSparkMax sendFollower = new PIDSparkMax(rightFollower);

                driveLogger.register(sendLeader, Map.of("Side", "Right", "Motor", "A"));
                driveLogger.register(sendFollower, Map.of("Side", "Right", "Motor", "B"));
                driveLogger.register(sendLeader.getEncoder(), Map.of("Side", "Right", "Motor", "A"));
                driveLogger.register(sendFollower.getEncoder(), Map.of("Side", "Right", "Motor", "B"));

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
                PIDSparkMax sendFollower = new PIDSparkMax(leftFollower);

                driveLogger.register(sendLeader, Map.of("Side", "Left", "Motor", "A"));
                driveLogger.register(sendFollower, Map.of("Side", "Left", "Motor", "B"));
                driveLogger.register(sendLeader.getEncoder(), Map.of("Side", "Left", "Motor", "A"));
                driveLogger.register(sendFollower.getEncoder(), Map.of("Side", "Left", "Motor", "B"));

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
                final PigeonGyro gyro = new PigeonGyro(gyroTalon);
                driveLogger.register(gyro);
                return gyro;
            }
        };
    };

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {

            final SubsystemLogger intakeLogger = logger.addSubsystem("Intake");

            @Override
            public SmartSpeedController intake() {
                SmartSpeedController intake = SmartSpeedController.wrap(new WPI_TalonSRX(42));
                intakeLogger.register(intake, Map.of("Motor", "IntakeA"));
                return intake;
            }

            @Override
            public WDSolenoid deployIntake() {
                WDSolenoid solenoid = new WDSolenoid(1, 2);
                intakeLogger.register(solenoid);
                return solenoid;
            }
        };
    }

    @Override
    public ShooterMap getShooterMap() {
        return new ShooterMap() {
            private SubsystemLogger shooterLogger = logger.addSubsystem("Intake");
            CANSparkMax leader = new CANSparkMax(23, MotorType.kBrushless);
            CANSparkMax follower = new CANSparkMax(26, MotorType.kBrushless);
            PIDSparkMax pidLeader = new PIDSparkMax(leader);
            PIDSparkMax pidFollower = new PIDSparkMax(follower);

            @Override
            public PIDSparkMax shooterWheel() {
                shooterLogger.register(pidLeader, Map.of("Motor", "A"));
                shooterLogger.register(pidFollower, Map.of("Motor", "B"));
                shooterLogger.register(pidLeader.getEncoder(), Map.of("Motor", "A"));
                shooterLogger.register(pidFollower.getEncoder(), Map.of("Motor", "B"));

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
    public IndexMap getIndexerMap() {
        return new IndexMap() {
            private SubsystemLogger indexerLogger = logger.addSubsystem("Indexer");
            AnalogInput topPierreIRAnalog = new AnalogInput(0);
            AnalogTrigger topPierreIR = new AnalogTrigger(topPierreIRAnalog);

            AnalogInput bottomPierreIRAnalog = new AnalogInput(1);
            AnalogTrigger bottomPierreIR = new AnalogTrigger(bottomPierreIRAnalog);

            @Override
            public SmartSpeedController pierreMotor() {
                final WPI_TalonSRX pierreMotor = new WPI_TalonSRX(40);
                setBAGCurrentLimits(pierreMotor);
                indexerLogger.register(pierreMotor, Map.of("Motor", "Pierre"));
                return SmartSpeedController.wrap(pierreMotor);
            }

            public SmartSpeedController singulator() {
                final WPI_TalonSRX singulator = new WPI_TalonSRX(41);
                singulator.setInverted(true);
                setBAGCurrentLimits(singulator);
                indexerLogger.register(singulator, Map.of("Motor", "Singulator"));
                return SmartSpeedController.wrap(singulator);
            }

            // TODO Find values for voltage limits
            public BooleanSupplier topPierreIR() {
                topPierreIR.setLimitsVoltage(1.2, 1.4);
                indexerLogger.register(topPierreIRAnalog, Map.of("Sensor", "TopPierre"));
                indexerLogger.register(topPierreIR, Map.of("Sensor", "TopPierre"));
                return topPierreIR::getTriggerState;
            }

            // TODO Find values for voltage limits; test 1-3.5 and then test 1.5-3.5
            public BooleanSupplier bottomPierreIR() {
                bottomPierreIR.setLimitsVoltage(1.0, 1.2);
                indexerLogger.register(bottomPierreIRAnalog, Map.of("Sensor", "BottomPierre"));
                indexerLogger.register(bottomPierreIR, Map.of("Sensor", "BottomPierre"));
                return bottomPierreIR::getTriggerState;
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