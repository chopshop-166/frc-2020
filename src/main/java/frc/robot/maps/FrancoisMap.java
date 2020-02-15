package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.GyroBase;

@RobotMapFor("Francois")
public class FrancoisMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        final double distancePerPulse = (1.0 / 46.0) * (1.0 / 12.27) * (6.0 * Math.PI);
        return new DifferentialDriveMap() {
            CANSparkMax rightLeader = new CANSparkMax(27, MotorType.kBrushless);
            CANSparkMax rightFollower = new CANSparkMax(22, MotorType.kBrushless);

            CANSparkMax leftLeader = new CANSparkMax(29, MotorType.kBrushless);
            CANSparkMax leftFollower = new CANSparkMax(25, MotorType.kBrushless);

            @Override
            public EncodedSpeedController getRight() {

                CANEncoder leadEncoder = new CANEncoder(rightLeader, EncoderType.kQuadrature, 42);
                rightFollower.follow(rightLeader);

                leadEncoder.setPositionConversionFactor(distancePerPulse);

                return EncodedSpeedController.wrap(rightLeader);
            }

            @Override
            public EncodedSpeedController getLeft() {
                CANEncoder leadEncoder = new CANEncoder(leftLeader, EncoderType.kQuadrature, 42);
                leftFollower.follow(leftLeader);

                leadEncoder.setPositionConversionFactor(distancePerPulse);

                return EncodedSpeedController.wrap(leftLeader);
            }

            @Override
            public GyroBase getGyro() {
                PigeonIMU gyro = new PigeonIMU(new WPI_TalonSRX(42));
                return new GyroBase() {

                    @Override
                    public void close() throws Exception {
                        // NoOp

                    }

                    @Override
                    public void reset() {
                        gyro.setFusedHeading(0);

                    }

                    @Override
                    public double getRate() {
                        double[] xyz = new double[3];
                        gyro.getRawGyro(xyz);
                        return xyz[2];
                    }

                    @Override
                    public double getAngle() {
                        return gyro.getFusedHeading();
                    }

                    @Override
                    public void calibrate() {
                        // NoOp
                    }
                };
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SendableSpeedController intake() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(40));
            }

            @Override
            public WDSolenoid deployIntake() {
                return new WDSolenoid(0, 1);
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
                follower.follow(leader);

                return new PIDSparkMax(leader);
            }
        };
    }

    @Override
    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap() {
            @Override
            public SendableSpeedController spinner() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(41));
            }
        };
    }

    @Override
    public LiftMap getLiftMap() {
        return new LiftMap() {
            CANSparkMax leader = new CANSparkMax(21, MotorType.kBrushless);
            CANSparkMax follower = new CANSparkMax(28, MotorType.kBrushless);

            @Override
            public PIDSparkMax elevator() {
                follower.follow(leader);

                return new PIDSparkMax(leader);
            }
        };
    }

}
