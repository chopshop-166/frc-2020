package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

<<<<<<< Updated upstream
/**
 * SUB REVIEW Lift Subsystem What Does It Do? 1) Raises and lowers lift Manually
 * using two different moters in sync during the last 30 seconds of the game 2)
 * Uses ratchet mechanism to break
 * 
 * What Modes Does It Have? 1) Up (Extend) 2) Down (Retract) 3) Break (Ratchet)
 * 4) Hook onto Bar using d-clip
 * 
 * What Interactions Does It Have With Other Subsystems? 1) None
 * 
 * How Is It Triggered? 1) User Input
 * 
 * Does It Store Any State? 1) isBreaked 2) Limit switch values 3) Encoder
 * values
 * 
 * Sensors? 1) Encoders 2) Limit Switches
 */
=======
//Lifts the robot to be able to climb the bar in the last 30 seconds of the game
//It should have a mode to extend the lift and retract to lift the robot at different heights depending on the position of the bar
//It doesn't interact with any other subsystems other than the space it shares
//It is triggered through motors and a wynch to be able to extend the lift and lift up the robot (and a d-clip to lock onto the bar)
//Stores the distance that its gone up in order to lift back up the same amount
//Do not know of any sensors at the moment, could possibly have sensors to tell the height of the bar (LYDAR)

//Commands needed would be: going up, and pulling the robot up (putting the lift down)
>>>>>>> Stashed changes

public class Lift extends SubsystemBase {

    private SendableSpeedController elevatorMotor;

    private static final double elevatorMotorSpeed = 1;

    // TODO Find a way to sync the elevatorLeft and elevatorRight motors
    public Lift(RobotMap.LiftMap map) {
        super();
        elevatorMotor = map.elevatorLeft();
    }

    public CommandBase extend() {
        return new StartEndCommand(() -> {
            elevatorMotor.set(-elevatorMotorSpeed);
        }, () -> {
            elevatorMotor.stopMotor();
        }, this);
    }

    public CommandBase retract() {
        return new StartEndCommand(() -> {
            elevatorMotor.set(elevatorMotorSpeed);
        }, () -> {
            elevatorMotor.stopMotor();
        }, this);
    }
}