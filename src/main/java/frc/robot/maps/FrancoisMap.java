package frc.robot.maps;

import com.chopshop166.chopshoplib.RobotMapFor;
import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

@RobotMapFor("Tempest")
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
}
