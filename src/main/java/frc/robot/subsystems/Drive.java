package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final SpeedController rightMotorGroup;
    private final SpeedController leftMotorGroup;
    private final DifferentialDrive driveTrain;

    public Drive() {
        super();
        rightMotorGroup = new SpeedControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(3));
        leftMotorGroup = new SpeedControllerGroup(new WPI_TalonSRX(1), new WPI_TalonSRX(4));
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
