package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

//Spins the control panel for a specific number of rotaions and stops on a specific color
//Has a mode to spin for a number of rotations and a mode to spin to stop on a specific color
//Does not have any interactions with other systems other than the space it shares on the robot
//Triggered by motors to spin the wheels and color sensors to be able to check colors and stop on them

public class ControlPanel extends SubsystemBase {

    private SendableSpeedController spinnerMotor;

    private static final double spinnerMotorSpeed = 1;

    public ControlPanel(RobotMap.ControlPanelMap map) {
        super();
        spinnerMotor = map.spinner();
    }

    public CommandBase spinForwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(-spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }

    public CommandBase spinBackwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }
}