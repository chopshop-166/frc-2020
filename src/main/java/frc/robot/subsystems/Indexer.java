package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.commands.CommandUtils;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.IndexMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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

public class Indexer extends SubsystemBase implements Loggable {
    @Log.SpeedController
    final SmartSpeedController singulator;
    @Log.SpeedController
    final SmartSpeedController pierreMotor;

    final BooleanSupplier bottomPierreIR;
    final BooleanSupplier topPierreIR;
    // final BooleanSupplier frontIntakeIR;
    // final BooleanSupplier backIntakeIR;

    @Log
    public double ballCounting;

    private static final double SINGULATOR_MOTOR_SPEED = 1.0;
    // Previous value was 0.6
    private static final double PIERRE_INDEX_SPEED = 0.4;
    private static final double PIERRE_SHOOT_SPEED = 1.0;

    public Indexer(final IndexMap map) {
        super();
        singulator = map.singulator();
        topPierreIR = map.topPierreIR();
        bottomPierreIR = map.bottomPierreIR();
        // frontIntakeIR = map.frontIntakeIR();
        // backIntakeIR = map.backIntakeIR();
        pierreMotor = map.pierreMotor();

    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Indexer Cancel");
        return cmd;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Top Pierre", topPierreIR.getAsBoolean());
        SmartDashboard.putBoolean("bOTTOM Pierre", bottomPierreIR.getAsBoolean());
        // SmartDashboard.putBoolean("BackIntake", backIntakeIR.getAsBoolean());
        // SmartDashboard.putBoolean("Front intake", frontIntakeIR.getAsBoolean());
    }

    // Will shoot all the balls. the only thing missing to this is the command to
    // spin up the shooter. that happens in robot
    public CommandBase shootBall() {
        CommandBase cmd = new SequentialCommandGroup(loadBallToTop(), unloadBall());
        cmd.setName("Shoot Ball");
        return cmd;
    }

    // runs the singulator
    public CommandBase singulator(final double motorSpeed) {
        CommandBase cmd = new FunctionalCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
        }, (interrupted) -> {
            singulator.set(0);
        }, () -> {
            return bottomPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Run Singulator");
        return cmd;
    }

    public CommandBase discharge() {
        return singulator(-SINGULATOR_MOTOR_SPEED);
    }

    // this will bring the ball to the top of pierre
    public CommandBase loadBallToTop() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            if (!topPierreIR.getAsBoolean()) {
                pierreMotor.set(PIERRE_SHOOT_SPEED);
            }
        }, (interrupted) -> {
            pierreMotor.set(0);
        }, () -> {
            return topPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Load Ball to Top");
        return cmd;
    }

    // this will bring the ball to the shooter, it must already be at the top
    public CommandBase unloadBall() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            pierreMotor.set(PIERRE_SHOOT_SPEED);
        }, (interrupted) -> {
            pierreMotor.set(0);
            ballCounting--;
        }, () -> {
            return !topPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Unload Ball");
        return cmd;
    }

    // this will shoot the balls until there are none left in pierre.
    public CommandBase shootAllBalls(int ballAmount) {
        CommandBase cmd = CommandUtils.repeat(ballAmount, this::shootBall);
        cmd.setName("Shoot All Balls");
        return cmd;
    }

    // this will make space for another ball
    public CommandBase indexBall() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            if (bottomPierreIR.getAsBoolean() && !topPierreIR.getAsBoolean()) {
                pierreMotor.set(PIERRE_INDEX_SPEED);
            } else {
                pierreMotor.set(0);
            }
        }, (interrupted) -> {
        }, () -> {
            return false;
        }, this);
        cmd.setName("Clear Bottom Sensor");
        return cmd;
    }

}
