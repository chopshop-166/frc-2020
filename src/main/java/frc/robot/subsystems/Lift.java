package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.sensors.InvertDigitalInput;
import com.google.common.collect.Range;

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
    private static final double TOLERANCE_RANGE = 5;

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
            elevatorBrake.set(false);
            if (upperLimitSwitch.getAsBoolean()) {
                speed = 0;
            }
        } else if (speed <= 0) {
            elevatorBrake.set(true);
            if (lowerLimitSwitch.getAsBoolean()) {
                speed = 0;
            }
        }
        elevatorMotor.set(speed);
    }

    // TODO make these heights reflect where we actually want them to be
    public enum liftHeights {
        Top(52.25), Middle(63), Bottom(78.125);

        private double mPosition;

        private double minLiftHeight = 51;

        // Returning an double to compare whether we're in the right place or not
        public double value() {
            return this.mPosition - minLiftHeight;
        }

        // Returning the level the lift is at (top middle or bottom)
        private liftHeights(double mPosition) {
            this.mPosition = mPosition;
        }
    }

    public CommandBase moveLift(DoubleSupplier speed) {
        // The command is named "Move Lift" and requires this subsystem.
        return new FunctionalCommand(() -> {
            liftSpeed(0);
        }, () -> {
            liftSpeed(speed.getAsDouble());
        }, (Boolean interrupted) -> {
            liftSpeed(0);
        }, () -> {
            return false;
        }, this);
    }

    public InstantCommand toggleBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(!elevatorBrake.get());
        }, this);
    }

    public CommandBase goToHeight(liftHeights iPoint) {
        return new FunctionalCommand(() -> {
            elevatorBrake.set(false);
        }, () -> {
            double getPosition = liftEncoder.getDistance();
            if (getPosition > iPoint.value()) {
                liftSpeed(elevatorMotorSpeed);
            } else {
                liftSpeed(-elevatorMotorSpeed);
            }
        }, (Boolean interrupted) -> {
            liftSpeed(0);
        }, () -> {
            double getPosition = liftEncoder.getDistance();
            return Math.abs(iPoint.value() - getPosition) <= TOLERANCE_RANGE;
        }, this);
    }
}