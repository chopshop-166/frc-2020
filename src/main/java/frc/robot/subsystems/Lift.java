package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import io.github.oblarg.oblog.annotations.Log;

public class Lift extends SubsystemBase {

    private SendableSpeedController elevatorMotor;

    @Log
    public String tabName = "Constants";
    private static final double elevatorMotorSpeed = 1;

    public Lift(RobotMap.LiftMap map) {
        super();
        elevatorMotor = map.elevator();
    }

    public CommandBase up() {
        return new StartEndCommand(() -> {
            elevatorMotor.set(-elevatorMotorSpeed);
        }, () -> {
            elevatorMotor.stopMotor();
        }, this);
    }

    public CommandBase down() {
        return new StartEndCommand(() -> {
            elevatorMotor.set(elevatorMotorSpeed);
        }, () -> {
            elevatorMotor.stopMotor();
        }, this);
    }
}