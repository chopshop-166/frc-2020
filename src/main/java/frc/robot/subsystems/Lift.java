package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.InvertDigitalInput;

import org.checkerframework.checker.units.qual.Speed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

/*
 * SUB REVIEW Lift Subsystem What Does It Do? 1) Raises and lowers lift Manually
 * using two different moters in sync during the last 30 seconds of the game 2)
 * Uses ratchet mechanism to break
 * 
 * What Modes Does It Have? 1) Up (Extend) 2) Down (Retract) 3) Brake (Ratchet)
 * 4) Hook onto Bar using d-clip 5) Go to Middle Bar Position 6) Go to Top Bar
 * Position
 * 
 * What Interactions Does It Have With Other Subsystems? 1) None
 * 
 * How Is It Triggered? 1) User Input
 * 
 * Does It Store Any State? 1) isBraked 2) Limit switch values 3) Encoder values
 * 
 * Sensors? 1) Encoders 2) Limit Switches
 */

public class Lift extends SubsystemBase {

    private PIDSpeedController elevatorMotor;
    private ISolenoid elevatorBrake;
    private IEncoder liftEncoder;
    private BooleanSupplier upperLimitSwitch;
    private BooleanSupplier lowerLimitSwitch;
    private static final double elevatorMotorSpeed = 1;

    boolean isBraked = elevatorBrake.get();

    // TODO Find a way to sync the elevatorLeft and elevatorRight motors
    public Lift(RobotMap.LiftMap map) {
        super();
        elevatorMotor = map.elevator();
        liftEncoder = map.getLiftEncoder();
        elevatorBrake = map.liftBrake();
        upperLimitSwitch = map.upperLiftLimit();
        lowerLimitSwitch = map.lowerLiftLimit();
    }

    public void liftSpeed(double speed) {
        if (speed > 0) {
            isBraked = false;
            if (upperLimitSwitch.getAsBoolean()) {
                speed = 0;
            }
        } else if (speed <= 0) {
            isBraked = true;
            if (lowerLimitSwitch.getAsBoolean()) {
                speed = 0;
            }
        }
        elevatorBrake.set(isBraked);
        elevatorMotor.set(speed);

    }

    // TODO Confgure encoders to actually read a position
    int getPosition = liftHeights.Bottom.value();

    // TODO make these heights reflect where we actually want them to be
    public enum liftHeights {
        Top(3), Middle(2), Bottom(1);

        private int iPosition;

        // Returning an interger to compare whether we're in the right place or not
        private int value() {
            return this.iPosition;
        }

        // Returning the level the lift is at (top middle or bottom)
        private liftHeights(int iPosition) {
            this.iPosition = iPosition;
        }
    }

    public CommandBase moveLift(DoubleSupplier speed) {
        // The command is named "Move Lift" and requires this subsystem.
        return new FunctionalCommand(() -> {
            elevatorBrake.set(false);
            isBraked = false;
        }, () -> {
            liftSpeed(speed.getAsDouble());
        }, (Boolean interrupted) -> {
            elevatorBrake.set(true);
            isBraked = true;
        }, () -> {
            return false;
        }, this);
    }

    public InstantCommand engageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(true);
            isBraked = true;
        }, this);
    }

    public InstantCommand disengageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(false);
            isBraked = false;
        }, this);
    }

    public InstantCommand toggleBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(!isBraked);
        }, this);
    }

    public CommandBase goToHeight(liftHeights iPoint) {
        return new FunctionalCommand(() -> {
            elevatorBrake.set(false);
        }, () -> {
            if (getPosition > 2) {
                liftSpeed(elevatorMotorSpeed);
            } else {
                liftSpeed(-elevatorMotorSpeed);
            }
        }, (Boolean interrupted) -> {
            liftSpeed(0);
            elevatorBrake.set(true);
            isBraked = true;
        }, () -> {
            return getPosition == iPoint.value();
        }, this);
    }
}