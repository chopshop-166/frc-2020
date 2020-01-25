package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * 1) What does it do? Spins the control panel for a specific number of rotaions
 * and stops on a specific color.
 * 
 * 2) What modes does it have? Has a mode to spin for a number of rotations and
 * a mode to spin to stop on a specific color.
 * 
 * 3) What interactions does it have with other subsystems? Does not have any
 * interactions with other systems other than the space it shares on the robot
 * 
 * 4) How is it triggered -> OI? Triggered by motors to spin the wheels and
 * color sensors to be able to check colors and stop on them.
 * 
 * 5) Does it store any state? Stores the control panel current color and stores
 * how many rotations the panel has spun.
 * 
 * 6) Sensors? It has a color sensor
 */
public class ControlPanel extends SubsystemBase {

    private final SendableSpeedController spinnerMotor;

    private static final double spinnerMotorSpeed = 1;

    /**
     * Defines a motor
     * 
     * @param map represents the control panel map
     */
    public ControlPanel(final RobotMap.ControlPanelMap map) {
        super();
        spinnerMotor = map.spinner();
    }

    /**
     * Spin Forwards
     * 
     * This command runs the motor in a positive set direction
     * 
     * @return {@link StartEndCommand} for running the motor at spinnerMotorSpeed
     */
    public CommandBase spinForwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }

    /**
     * Spin Backswards
     * 
     * This command runs the motor in a negative set direction
     * 
     * @return {@link StartEndCommand} for running the motor at spinnerMotorSpeed
     */
    public CommandBase spinBackwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(-spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }
}