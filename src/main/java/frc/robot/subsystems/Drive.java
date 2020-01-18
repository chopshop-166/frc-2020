package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Drive extends SubsystemBase {

    private final SendableSpeedController rightMotorGroup;
    private final SendableSpeedController leftMotorGroup;
    private final DifferentialDrive driveTrain;

    public Drive(RobotMap.DriveMap map) {
        super();
        rightMotorGroup = map.right();
        leftMotorGroup = map.left();
        driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    public CommandBase drive(DoubleSupplier forward, DoubleSupplier turn) {
        return new RunCommand(() -> {
            double yAxis = forward.getAsDouble();
            double xAxis = turn.getAsDouble();

            driveTrain.arcadeDrive(yAxis, xAxis);
        }, this);
    }
}
