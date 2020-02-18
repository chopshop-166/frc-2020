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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

    // This will stop all commands when the top IR sensor on Pierre is triggered

    public CommandBase indexMotor(final double motorSpeed) {
        return new StartEndCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
            singulator.set(0);
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

    // This command will make sure that the singulator has possesion of the ball

    public CommandBase pierrePossesion() {
        return new FunctionalCommand(() -> {

        }, () -> {

            if (frontIntakeIR.getAsBoolean()) {

                singulator.set(singulatorMotorSpeed);
            }
            if (backIntakeIR.getAsBoolean()) {
                pierreMotor.set(pierreIndexSpeed);
                singulator.set(singulatorMotorSpeed);
            }
        }, (interrupted) -> {

            pierreMotor.set(0);
            singulator.set(0);
            SmartDashboard.putNumber("Ball Count", ballCounting);

        }, () -> {

            return bottomPierreIR.getAsBoolean() || topPierreIR.getAsBoolean();

        }, this);

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

    public CommandBase unLoadBall() {
        return new FunctionalCommand(() -> {

        }, () -> {

            pierreMotor.set(pierreIndexSpeed);

        }, (interrupted) -> {

            pierreMotor.set(0);
            ballCounting--;
            SmartDashboard.putNumber("Ball Count", ballCounting);

        }, () -> {

            return !topPierreIR.getAsBoolean();

        }, this);

    }
    // this will bring the ball to the shooter, it must already be at the top

    public CommandBase runToClearBottomSensor() {
        return new FunctionalCommand(() -> {
            if (bottomPierreIR.getAsBoolean()) {
                pierreMotor.set(pierreIndexSpeed);
            }

        }, () -> {

        }, (interrupted) -> {

            pierreMotor.set(0);
            if (interrupted == false) {
                ballCounting++;
            }
            SmartDashboard.putNumber("Ball Count", ballCounting);

        }, () -> {

            return !bottomPierreIR.getAsBoolean() || topPierreIR.getAsBoolean();

        }, this);

    }

    // this will make space for another ball
    public WaitUntilCommand stopWhenBallsAtTop() {
        return new WaitUntilCommand(topPierreIR::getAsBoolean);
    }
    // The balls will not go past the top sensor unless called by the specific
    // function
}
