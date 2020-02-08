package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.maps.RobotMap.IndexMap;

/**
 * The indexer lines up and transports the balls to the Shooter The indexer has
 * two modes, On(motors on) and Off(motors off) The indexer relates to the
 * Shooter in that it should only put the balls in when the Shooter is ready The
 * indexer is triggered with motors and uses belts to move the balls(controller)
 * The indexer will use IR sensors so we can see the presence of a ball The
 * distance that the balls are pushed between the belts needs to be coded so
 * that they can move fluidly When a ball is at the top of the Pierre, the ball
 * at the entrance of Pierre needs to be moved forward at the same pace of the
 * balls being intaked
 * 
 */

public class Indexer extends SubsystemBase {
    final SendableSpeedController singulator;
    private SendableSpeedController pierreMotor;
    AnalogInput irSensor1;
    AnalogInput irSensor2;
    AnalogInput irSensor3;

    private static final double singulatorMotorSpeed = 0.85;
    private static final double pierreIndexSpeed = 0.85;

    public Indexer(final IndexMap map) {
        super();
        singulator = map.singulator();
        irSensor1 = map.irSensor1();
        irSensor2 = map.irSensor2();
        irSensor3 = map.irSensor3();
        pierreMotor = map.pierreMotor();
        // singulatorMotor = map.singulator();
    }

    public SequentialCommandGroup intakeToPierre() {
        return new SequentialCommandGroup(singulatorPossesion(), pierrePossesion(), runToClear());
    }
   


    public CommandBase indexMotor(final double motorSpeed) {
        return new StartEndCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
            singulator.set(0);
        });
    }

    public CommandBase runPierre() {
        return new StartEndCommand(() -> {
            pierreMotor.set(.75);
        }, () -> {
            pierreMotor.stopMotor();
        }, this);
    }

    // private CommandBase pierreMotor(final double motorSpeed) {
    // return new StartEndCommand(() -> {
    // pierreMotor1.set(motorSpeed);
    // }, () -> {
    // pierreMotor1.set(0);
    // });
    // }

    public CommandBase quicklyPush() {
        return indexMotor(singulatorMotorSpeed);
    }

    public CommandBase reversePush() {
        return indexMotor(singulatorMotorSpeed / 2);
    }

    public CommandBase quicklyOutput() {
        return indexMotor(pierreIndexSpeed);
    }

    public CommandBase reverseOutput() {
        return indexMotor(pierreIndexSpeed / 2);
    }

    /*
     * Command to run the singlator motor until the IR sensor 1 (analog 0) Command
     * to run the singulator and pierre motor until the IR senor 1 is empty and IR
     * senor 2 is covered Command to run the pierre motor until the IR sensor is
     * uncovered
     */
    public CommandBase singulatorPossesion() {
        return new CommandBase() {

            @Override
            public boolean isFinished() {
                return irSensor1.getVoltage() > 1.5;
                // values of .8 when open, 1.7 when closed

            }

            @Override
            public void execute() {
                singulator.set(singulatorMotorSpeed);

            }

            @Override
            public void end(boolean interrupted) {
                singulator.set(0);
            }

        };

    }

    public CommandBase pierrePossesion() {
        return new CommandBase() {

            @Override
            public boolean isFinished() {
                return irSensor2.getVoltage() > 2.0;
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {
                pierreMotor.set(pierreIndexSpeed);
                singulator.set(singulatorMotorSpeed);

            }

            @Override
            public void end(boolean interrupted) {
                pierreMotor.set(0);
                singulator.set(0);

            }

        };

    }

    public CommandBase loadBallToTop() {
        return new CommandBase() {

            @Override
            public boolean isFinished() {
                return irSensor3.getVoltage() > 2.0;
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {
                pierreMotor.set(pierreIndexSpeed);

            }

            @Override
            public void end(boolean interrupted) {
                pierreMotor.set(0);

            }

        };

    }

    public CommandBase runToClear() {
        return new CommandBase() {

            @Override
            public void initialize() {
                if (irSensor2.getVoltage() > 2.0) {
                    pierreMotor.set(pierreIndexSpeed);
                }
            }

            @Override
            public boolean isFinished() {
                return irSensor2.getVoltage() < 2.0;
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {
                // pierreMotor.set(0);

            }

            @Override
            public void end(boolean interrupted) {
                pierreMotor.set(0);

            }

        };

    }

}
