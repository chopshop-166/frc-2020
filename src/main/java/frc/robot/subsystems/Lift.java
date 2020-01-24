package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

//Lifts the robot to be able to climb the bar in the last 30 seconds of the game
//It should have a mode to extend the lift, hook onto the bar, and retract the robot to lift it up
//It doesn't interact with any other subsystems other than the space it shares
//It is triggered through motors and a wynch to be able to extend the lift and lift up the robot (and a d-clip to lock onto the bar)

public class Lift extends SubsystemBase {

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private SendableSpeedController elevatorMotor;
    private Solenoid elevatorBrake;

    private static final double elevatorMotorSpeed = 1;

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

    public InstantCommand engageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(true);
        });
    }

    public InstantCommand diengageBrake() {
        return new InstantCommand(() -> {
            elevatorBrake.set(false);
        });
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

    public CommandBase goToPoint(liftHeights iPoint) {

        return new CommandBase() {
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
            }
        };
    }
}