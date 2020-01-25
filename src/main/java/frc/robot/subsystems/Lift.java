package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

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
 * Does It Store Any State? 1) isBreaked
 */

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