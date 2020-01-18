package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class ControlPanel extends SubsystemBase {
    private SendableSpeedController spinnerMotor;

    public ControlPanel(RobotMap.ControlPanelMap map) {
        super();
        spinnerMotor = map.spinner();
    }

    public CommandBase spinForwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(-0.85);
        }, () -> {
            spinnerMotor.set(0);
        }, this);
    }

    public CommandBase spinBackwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(0.85);
        }, () -> {
            spinnerMotor.set(0);
        }, this);
    }
}