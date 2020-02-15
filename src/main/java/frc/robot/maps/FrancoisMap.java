package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

@RobotMapFor("Francois")
public class FrancoisMap extends RobotMap {

    @Override
    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {

                CANSparkMax leader = new CANSparkMax(20, MotorType.kBrushless);
                CANSparkMax follower = new CANSparkMax(21, MotorType.kBrushless);
                follower.follow(leader);

                return EncodedSpeedController.wrap(leader);
            }

            @Override
            public EncodedSpeedController getLeft() {
                CANSparkMax leader = new CANSparkMax(22, MotorType.kBrushless);
                CANSparkMax follower = new CANSparkMax(23, MotorType.kBrushless);
                follower.follow(leader);

                return EncodedSpeedController.wrap(leader);
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
            @Override
            public PIDSparkMax shooterWheel() {
                CANSparkMax leader = new CANSparkMax(25, MotorType.kBrushless);
                CANSparkMax follower = new CANSparkMax(26, MotorType.kBrushless);
                leader.setInverted(true);
                follower.setInverted(true);
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
            @Override
            public PIDSparkMax elevator() {
                CANSparkMax leader = new CANSparkMax(27, MotorType.kBrushless);
                CANSparkMax follower = new CANSparkMax(28, MotorType.kBrushless);
                follower.follow(leader);

                return new PIDSparkMax(leader);
            }
        };
    }

}
