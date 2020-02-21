package frc.robot.subsystems;

import java.sql.Time;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private static final double elevatorMotorSpeed = 1;
    private static final double TOLERANCE_RANGE_INCHES = .5;
    // TODO I don't know what unit tolerance range is in but I set it to .5 assuming
    // it's in inches

    // TODO Find a way to sync the elevatorLeft and elevatorRight motors
    public Lift(RobotMap.LiftMap map) {
        super();
        elevatorMotor = map.elevator();
        liftEncoder = map.getLiftEncoder();
        elevatorBrake = map.liftBrake();
        upperLimitSwitch = map.upperLiftLimit();
    }

    // sets the ratchet to either be activated or deactivated depending on liftSpeed
    public void liftSpeed(double speed) {
        if (Math.abs(speed) <= .1) {
            speed = 0.0;
        }
        if (speed < 0) {
            if (upperLimitSwitch.getAsBoolean() || !elevatorBrake.get()) {
                speed = 0;
            }
        } else if (speed > 0) {
            elevatorBrake.set(false);
        }
        elevatorMotor.set(speed);
    }

    // Heights of the bar at max, min, and middle
    public enum liftHeights {
        Bottom(52.25), Middle(63), Top(78.125);

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

    public SequentialCommandGroup disengageRatchet(DoubleSupplier speed) {
        return new SequentialCommandGroup(moveTicks(1, 0.10), turnOffBrake(), new WaitCommand(.5), moveLift(speed));

    }

    public CommandBase turnOffBrake() {
        return new InstantCommand(() -> elevatorBrake.set(true), this);
    }

    public CommandBase moveTicks(double ticks, double speed) {
        return new FunctionalCommand(() -> {
            liftEncoder.reset();
        }, () -> {
            SmartDashboard.putNumber("Lift Encoder", liftEncoder.getDistance());
            elevatorMotor.set(speed);
        }, (interrupted) -> {
            elevatorMotor.stopMotor();
        }, () -> {
            return Math.abs(liftEncoder.getDistance()) >= ticks;
        }, this);
    }

    public CommandBase moveLift(DoubleSupplier speed) {
        return new FunctionalCommand(() -> {
            liftSpeed(0);
        }, () -> {
            liftSpeed(speed.getAsDouble());
        }, (Boolean interrupted) -> {
            liftSpeed(0);
        }, () -> false, this);
    }

    // allows to toggle the break. Pretty self explanitory
    public InstantCommand toggleBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(!elevatorBrake.get());
        }, this);
    }

    // Sets the lift go to the point where to bar would be based off the encoder
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
            return Math.abs(iPoint.value() - getPosition) <= TOLERANCE_RANGE_INCHES;
        }, this);
    }
}