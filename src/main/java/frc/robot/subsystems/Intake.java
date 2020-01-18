package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Intake extends SubsystemBase {
    private SendableSpeedController rollerMotor;

    public Intake(RobotMap.IntakeMap map) {
        super();
        rollerMotor = map.roller();
    }

    public CommandBase intake() {
        return new StartEndCommand(() -> {
            rollerMotor.set(0.85);
        }, () -> {
            rollerMotor.set(0);
        }, this);
    }

    public CommandBase intakeReverse() {
        return new StartEndCommand(() -> {
            rollerMotor.set(-0.85);
        }, () -> {
            rollerMotor.set(0);
        }, this);
    }
}