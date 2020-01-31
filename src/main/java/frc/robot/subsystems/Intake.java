package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/**
 * The intake takes in the balls from the ground and it is raised and retracted
 * by pneumatic pistons There is only one speed for the intake motor(Member:
 * Motor) The intake interacts with the Indexer It uses motors, wheels, and
 * pistons connected by belts and bands(The controller) The intake needs to make
 * sure the balls pass through it and don't get jammed(Member: Motor)
 * 
 */

public class Intake extends SubsystemBase {
    private final SendableSpeedController rollerMotor;
    private final IDSolenoid deployPiston;
    private final IDSolenoid rollerPlacement;

    private static final double rollerMotorSpeed = 0.85;

    public Intake(final RobotMap.IntakeMap map) {
        super();
        rollerMotor = map.roller();
        deployPiston = map.deployPiston();
        rollerPlacement = map.placeRoller();
    }

    private CommandBase runRoller(final double motorSpeed) {
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

    // Infrared Proximity Sensor
    // pneumatic pistons to raise the intake up so that balls can get underneath

    public CommandBase placeRoller() {
        return new StartEndCommand(() -> {
            rollerPlacement.set(Value.kForward);
        }, () -> {
            rollerPlacement.set(Value.kReverse);
        }, this);
    }

    public InstantCommand setPiston(final Value direction) {
        return new InstantCommand(() -> {
            deployPiston.set(direction);
        }, this);
    }

    public InstantCommand deployPiston() {
        return setPiston(Value.kForward);
    }

    public InstantCommand retractPiston() {
        return setPiston(Value.kReverse);
    }

    public class DeployIntake extends ParallelCommandGroup {
        public DeployIntake() {
            addCommands(runRoller(rollerMotorSpeed), placeRoller());
        }
    };
}