package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.outputs.ISolenoid;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    private IEncoder leftEncoder;
    private IEncoder rightEncoder;
    private SendableSpeedController elevatorMotor;
    private ISolenoid elevatorBrake;

    private static final double elevatorMotorSpeed = 1;

    boolean isBraked = elevatorBrake.get();

    // TO DO Find a way to sync the elevatorLeft and elevatorRight motors
    public Lift(RobotMap.LiftMap map) {
        super();
        leftEncoder = map.getLeftEncoder();
        rightEncoder = map.getRightEncoder();
        elevatorMotor = map.elevatorLeft();
        elevatorBrake = map.liftBrake();
    }

    // TO DO Confgure encoders to actually read a position
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
        return new CommandBase() {

            @Override
            public void initialize() {
                disengageBrake();

            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void execute() {
                elevatorMotor.set(speed.getAsDouble());

            }

            @Override
            public void end(final boolean interrupted) {
                engageBrake();

            }

        };
    }

    public InstantCommand engageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(true);
        });
    }

    public InstantCommand disengageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(false);
        });
    }

    public InstantCommand toggleBrake() {
        return new InstantCommand(() -> {
            if (isBraked == false) {
                elevatorBrake.set(true);
            } else
                elevatorBrake.set(false);

        });
    }

    public CommandBase goToHeight(liftHeights iPoint) {

        return new CommandBase() {
            @Override
            public void initialize() {
                disengageBrake();

            }

            @Override
            public boolean isFinished() {
                return getPosition == iPoint.value();
            }

            @Override
            public void execute() {
                if (getPosition > 2) {
                    elevatorMotor.set(elevatorMotorSpeed);
                } else {
                    elevatorMotor.set(-elevatorMotorSpeed);
                }
            }

            @Override
            public void end(boolean interrupted) {
                elevatorMotor.set(0);
                engageBrake();
            }
        };
    }
}