package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
    private static final double pierreIndexSpeed = 1;

    public Indexer(final IndexMap map) {
        super();
        singulator = map.singulator();
        frontIntakeIR = map.frontIntakeIR();
        bottomPierreIR = map.bottomPierreIR();
        topPierreIR = map.topPierreIR();
        backIntakeIR = map.backIntakeIR();
        pierreMotor = map.pierreMotor();

    }

    public CommandBase intakeToPierre() {
        CommandBase cmd = new SequentialCommandGroup(pierrePossesion(), runToClearBottomSensor());
        cmd.setName("Intake to Pierre");
        return cmd;
    }

    public CommandBase shootingBalls() {
        CommandBase cmd = new SequentialCommandGroup(loadBallToTop(), unLoadBall());
        cmd.setName("Shoot Ball");
        return cmd;
    }
    // Will shoot all the balls. the only thing missing to this is the command to
    // spin up the shooter. that happens in robot

    public CommandBase indexMotor(final double motorSpeed) {
        CommandBase cmd = new StartEndCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
            singulator.set(0);
        }, this);
        cmd.setName("Run Singulator");
        return cmd;
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
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            if (frontIntakeIR.getAsBoolean()) {
                singulator.set(singulatorMotorSpeed);
            }
            // This checks to see if a ball is at the top of Pierre and doesn't not run
            // because sometimes it will
            if (backIntakeIR.getAsBoolean() && !topPierreIR.getAsBoolean()) {
                pierreMotor.set(pierreIndexSpeed);
                singulator.set(singulatorMotorSpeed);
            }
        }, (interrupted) -> {
            pierreMotor.set(0);
            singulator.set(0);
            SmartDashboard.putNumber("Ball Count", ballCounting);
        }, () -> {
            return (bottomPierreIR.getAsBoolean() && !backIntakeIR.getAsBoolean()) || topPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Pierre Possession");
        return cmd;
    }
    // This command will make sure that pierre has possesion of the ball. It will be
    // at the bottom

    public CommandBase loadBallToTop() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            if (!topPierreIR.getAsBoolean()) {
                pierreMotor.set(pierreIndexSpeed);
            }
        }, (interrupted) -> {
            pierreMotor.set(0);
        }, () -> {
            return topPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Load Ball to Top");
        return cmd;
    }

    public CommandBase unLoadBall() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            pierreMotor.set(pierreIndexSpeed);
        }, (interrupted) -> {
            pierreMotor.set(0);
            ballCounting--;
            SmartDashboard.putNumber("Ball Count", ballCounting);
        }, () -> {
            return !topPierreIR.getAsBoolean();
        }, this);
        cmd.setName("Unload Ball");
        return cmd;
    }
    // this will bring the ball to the shooter, it must already be at the top

    public CommandBase shootAllBalls() {
        CommandBase cmd = new SequentialCommandGroup(shootingBalls(), shootingBalls(), shootingBalls(), shootingBalls(),
                shootingBalls());
        cmd.setName("Shoot All Balls");
        return cmd;
    }

    // this will shoot the balls until there are none left in pierre.
    public CommandBase runToClearBottomSensor() {
        CommandBase cmd = new FunctionalCommand(() -> {
            if (bottomPierreIR.getAsBoolean() && !topPierreIR.getAsBoolean()) {
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
        cmd.setName("Clear Bottom Sensor");
        return cmd;
    }

    // this will make space for another ball
}
