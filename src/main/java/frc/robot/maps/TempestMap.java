package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;

@RobotMapFor("Tempest")
public class TempestMap extends RobotMap {

    @Override
    public DriveMap getDriveMap() {
        return new DriveMap() {
            @Override
            public SendableSpeedController left() {
                return SendableSpeedController.wrap(new SpeedControllerGroup(new WPI_TalonSRX(4), new WPI_TalonSRX(1)));
            }

            @Override
            public SendableSpeedController right() {
                return SendableSpeedController.wrap(new SpeedControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(3)));
            }
        };
    }

    @Override
    public IntakeMap getIntakeMap() {
        return new IntakeMap() {
            @Override
            public SendableSpeedController roller() {
                final Talon rollerMotor = new Talon(0);
                rollerMotor.setInverted(true);
                return SendableSpeedController.wrap(rollerMotor);
            }
        };
    }
}