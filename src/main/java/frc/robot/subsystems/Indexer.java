package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
    private final SendableSpeedController pierreMotor;
    final BooleanSupplier frontIntakeIR;
    final BooleanSupplier bottomPierreIR;
    final BooleanSupplier topPierreIR;
    final BooleanSupplier backIntakeIR;
    public double ballCounting;

    private static final double singulatorMotorSpeed = 0.95;
    private static final double pierreIndexSpeed = 0.85;

    public Indexer(final IndexMap map) {
        super();
        singulator = map.singulator();
        frontIntakeIR = map.frontIntakeIR();
        bottomPierreIR = map.bottomPierreIR();
        topPierreIR = map.topPierreIR();
        backIntakeIR = map.backIntakeIR();
        pierreMotor = map.pierreMotor();

    }

    public SequentialCommandGroup intakeToPierre() {
        return new SequentialCommandGroup(pierrePossesion(), runToClearBottomSensor());
    }

    public SequentialCommandGroup shootOneBall() {
        return new SequentialCommandGroup(loadBallToTop(), unLoadBall());
    }

    public ParallelDeadlineGroup topSensorTriggered() {
        final ParallelDeadlineGroup topIRStop = new ParallelDeadlineGroup(
                new SequentialCommandGroup(pierrePossesion(), runToClearBottomSensor()));
        topIRStop.setDeadline(stopWhenBallsAtTop());
        return topIRStop;
    }
    // This will stop all commands when the top IR sensor on Pierre is triggered

    public CommandBase indexMotor(final double motorSpeed) {
        return new StartEndCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
            singulator.set(0);
        }, this);
    }

    public CommandBase runPierre() {
        return new StartEndCommand(() -> {
            pierreMotor.set(.75);
        }, () -> {
            pierreMotor.stopMotor();
        }, this);
    }

    public CommandBase quicklyPush() {
        return indexMotor(singulatorMotorSpeed);
    }

    public CommandBase reversePush() {
        return indexMotor(-singulatorMotorSpeed);
    }

    public CommandBase quicklyOutput() {
        return indexMotor(pierreIndexSpeed);
    }

    public CommandBase reverseOutput() {
        return indexMotor(-pierreIndexSpeed);
    }

    /*
     * Command to run the singlator motor until the IR sensor 1 (analog 0) Command
     * to run the singulator and pierre motor until the IR senor 1 is empty and IR
     * senor 2 is covered Command to run the pierre motor until the IR sensor is
     * uncovered
     */
    public CommandBase singulatorPossesion() {
        return new CommandBase() {
            {
                addRequirements(Indexer.this);
            }

            @Override
            public boolean isFinished() {
                return frontIntakeIR.getAsBoolean();
            }

            @Override
            public void execute() {

            }

            @Override
            public void end(final boolean interrupted) {
                singulator.set(0);
            }

        };

    }
    // This command will make sure that the singulator has possesion of the ball

    public CommandBase pierrePossesion() {
        return new CommandBase() {

            @Override
            public void initialize() {
                if (frontIntakeIR.getAsBoolean()) {

                    singulator.set(singulatorMotorSpeed);
                }
                if (backIntakeIR.getAsBoolean()) {
                    pierreMotor.set(pierreIndexSpeed);
                    singulator.set(singulatorMotorSpeed);
                }
            }

            public boolean isFinished() {
                return bottomPierreIR.getAsBoolean();
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {
                if (frontIntakeIR.getAsBoolean()) {

                    singulator.set(singulatorMotorSpeed);
                }
                if (backIntakeIR.getAsBoolean()) {
                    pierreMotor.set(pierreIndexSpeed);
                    singulator.set(singulatorMotorSpeed);
                }
            }

            @Override
            public void end(final boolean interrupted) {
                pierreMotor.set(0);
                singulator.set(0);
                SmartDashboard.putNumber("Ball Count", ballCounting);

            }

        };

    }
    // This command will make sure that pierre has possesion of the ball. It will be
    // at the bottom

    public CommandBase loadBallToTop() {
        return new FunctionalCommand(() -> {

        }, () -> {

            pierreMotor.set(pierreIndexSpeed);

        }, (interrupted) -> {

            pierreMotor.set(0);

        }, () -> {

            return topPierreIR.getAsBoolean();

        }, this);

    }

    // this will bring the ball to the top
    public CommandBase unLoadBall() {
        return new CommandBase() {

            {
                addRequirements(Indexer.this);
            }

            @Override
            public boolean isFinished() {
                return !topPierreIR.getAsBoolean();
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {
                pierreMotor.set(pierreIndexSpeed);

            }

            @Override
            public void end(final boolean interrupted) {
                pierreMotor.set(0);
                ballCounting--;
                SmartDashboard.putNumber("Ball Count", ballCounting);

            }

        };

    }
    // this will bring the ball to the shooter, it must already be at the top

    public CommandBase runToClearBottomSensor() {
        return new CommandBase() {

            {
                addRequirements(Indexer.this);
            }

            @Override
            public void initialize() {
                if (bottomPierreIR.getAsBoolean()) {
                    pierreMotor.set(pierreIndexSpeed);
                }
            }

            @Override
            public boolean isFinished() {
                return !bottomPierreIR.getAsBoolean();
                // values of .8 when open, 2.4 when closed

            }

            @Override
            public void execute() {

            }

            @Override
            public void end(final boolean interrupted) {
                pierreMotor.set(0);
                if (interrupted == false) {
                    ballCounting++;
                }
                SmartDashboard.putNumber("Ball Count", ballCounting);

            }

        };

    }

    // this will make space for another ball
    public CommandBase stopWhenBallsAtTop() {
        return new CommandBase() {
            @Override
            public boolean isFinished() {
                return topPierreIR.getAsBoolean();
            }
        };
    }
    // The balls will not go past the top sensor unless called by the specific
    // function
}
