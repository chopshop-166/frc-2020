package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.chopshop166.chopshoplib.outputs.WSolenoid;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.InvertDigitalInput;
import com.chopshop166.chopshoplib.sensors.SparkMaxEncoder;
import com.chopshop166.chopshoplib.sensors.WEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

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

            CANSparkMax leader = new CANSparkMax(27, MotorType.kBrushless);

            @Override
            public PIDSparkMax elevator() {
                CANSparkMax follower = new CANSparkMax(28, MotorType.kBrushless);
                follower.follow(leader);

                return new PIDSparkMax(leader);
            }

            @Override
            public ISolenoid liftBrake() {
                // TODO Change this to a real Channel
                WSolenoid brake = new WSolenoid(3);

                return brake;
            }

            @Override
            public BooleanSupplier upperLiftLimit() {
                // TODO Change this to a real Channel
                InvertDigitalInput upperLimit = new InvertDigitalInput(0);

                return upperLimit::get;
            }

            @Override
            public IEncoder getLiftEncoder() {
                // TODO Change this to a real Channel
                SparkMaxEncoder getEncoder = new SparkMaxEncoder(leader.getEncoder());

                return getEncoder;
            }
        };
    }

}
