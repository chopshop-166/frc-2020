package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

//Lifts the robot to be able to climb the bar in the last 30 seconds of the game
//It should have a mode to extend the lift, hook onto the bar, and retract the robot to lift it up
//It doesn't interact with any other subsystems other than the space it shares
//It is triggered through motors and a wynch to be able to extend the lift and lift up the robot (and a d-clip to lock onto the bar)

public class Lift extends SubsystemBase {

    private SendableSpeedController elevatorMotor;

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