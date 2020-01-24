package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

//The intake takes in the balls from the ground, sorts them, stores them, and puts them into the shooter
//There is only one speed for the intake motor
//The intake interacts with the shooter
//It uses motors and wheels connected by belts and bands

public class Intake extends SubsystemBase {
    private SendableSpeedController rollerMotor;

    private static final double rollerMotorSpeed = 0.85;

    public Intake(RobotMap.IntakeMap map) {
        super();
        rollerMotor = map.roller();
    }

    private CommandBase runRoller(double motorSpeed) {
        return new StartEndCommand(() -> {
            rollerMotor.set(motorSpeed);
        }, () -> {
            rollerMotor.stopMotor();
        }, this);
    }

    public CommandBase intake() {
        return runRoller(rollerMotorSpeed);
    }

    public CommandBase discharge() {
        return runRoller(-rollerMotorSpeed);
    }
}