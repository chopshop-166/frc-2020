package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;
import com.chopshop166.chopshoplib.ThresholdCheck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * What does it do? Given a target and shooter height, it calculates velocity
 * needed to hit said target, and spins the motors at a corresponding RPM. What
 * modes does it have? Semi-Auto and DUMP. what interactions does it have with
 * other subsystems? Asks the Indexer if there is a ball, and get's information
 * about the target from vision. How is it triggered/OI? A semi-auto button A-
 * or X for shoot all. Does it store any state? No. Sensors? No.
 */

public class Shooter extends SubsystemBase implements Loggable {
    @Log.SpeedController
    private final PIDSpeedController shooterWheelMotor;
    @Log.Encoder
    private IEncoder shooterEncoder;
    @Log
    private static double velocity;
    public static double distanceToTarget;
    public final double shooterHeight;
    public static double verticalDistance;
    public static double horizontalDistance;
    @Log
    public double output;
    @Log
    public double shooterSpeed = 4300;

    // inches/second/second
    public final static double GRAVITY = 386.2205;
    // inches
    public final static double TARGET_HEIGHT = 98.25;
    public final static double THETA = Math.toRadians(60);
    // RPM equal to 1ft/s
    public final static double BALL_SPEED_RATIO = 27.358;

    private final static double MAX_SHOOTER_SPEED = 5400;
    // y = 2878.7x^(0.3094)

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        shooterEncoder = shooterWheelMotor.getEncoder();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
    }

    @Config
    public void setshooterSpeed(double speed) {
        shooterSpeed = speed;
    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Shooter Cancel");
        return cmd;
    }

    @Override
    public void periodic() {
        distanceToTarget = SmartDashboard.getNumber("Distance To Target", 160);
        horizontalDistance = Math.sqrt((verticalDistance * verticalDistance) - (distanceToTarget * distanceToTarget));
        super.periodic();
    }

    public CommandBase spinUp(final double speed) {
        final CommandBase cmd = new FunctionalCommand(() -> {
            // TODO incorperate calculations
            // shooterWheelMotor.set(calculateRPM() / MAX_RPM);
            shooterWheelMotor.setSetpoint(shooterSpeed);
        }, () -> {

        }, (interrupted) -> {

        }, () -> {
            return shooterEncoder.getRate() >= shooterSpeed;
        }, this);
        cmd.setName("spinUp");
        return cmd;
    }

    public CommandBase linearSpeedUp() {
        final CommandBase cmd = new FunctionalCommand(() -> {
            double dist = SmartDashboard.getNumber("Distance To Target", 3.8);
            // If we don't see the target then default to the Initiation line speed
            if (!SmartDashboard.getBoolean("Sees Target", false)) {
                output = 4700;
            }
            output = ((119.1 * (Math.pow(dist, 2))) - (1000 * dist) + 6300);

            shooterWheelMotor.setSetpoint(output);
        }, () -> {

        }, (interrupted) -> {

        }, () -> {
            return shooterEncoder.getRate() >= output;
        }, this);
        cmd.setName("spinUp");
        return cmd;
    }

    public CommandBase linearSpinUp() {
        CommandBase cmd = new CommandBase() {
            {
                addRequirements(Shooter.this);
            }
            ThresholdCheck check = new ThresholdCheck(25, () -> {
                return (shooterEncoder.getRate() >= output * .95) && (shooterEncoder.getRate() <= output * 1.05);
            });

            @Override
            public void initialize() {
                double dist = SmartDashboard.getNumber("Distance To Target", 3.8);
                output = 2800.7 * (Math.pow(dist, 0.3094));
                output = Math.max(output, MAX_SHOOTER_SPEED);
                shooterWheelMotor.setSetpoint(output);
            }

            @Override
            public void execute() {

            }

            @Override
            public boolean isFinished() {
                return check.getAsBoolean();
            }

            @Override
            public void end(boolean interrupted) {

            }
        };
        cmd.setName("Turn Degrees");
        return cmd;
    }

    public CommandBase spinDown() {
        final CommandBase cmd = new InstantCommand(shooterWheelMotor::stopMotor, this);
        cmd.setName("spinDown");
        return cmd;
    }

    public CommandBase mandatoryEvacuation() {
        final CommandBase cmd = new StartEndCommand(() -> {
            shooterWheelMotor.set(0.3);
        }, () -> {
            spinDown();
        }, this);
        cmd.setName("mandatoryEvacuation");
        return cmd;
    }

    /*
     * Calculates RPM with some gear ratio mathematics. (returns inches/second) Also
     * applies a 15% increase.
     */
    public CommandBase calculatedShoot() {
        final double rpmSpeed;

        // If it doesn't see the target, it will just shoot at the last speed.
        if (SmartDashboard.getBoolean("Sees Target", false)) {
            rpmSpeed = SmartDashboard.getNumber("Last RPM", 0);
        } else {
            final double rpm = calculateVelocity() * BALL_SPEED_RATIO * 1.15;
            SmartDashboard.putNumber("Last RPM", rpm);
            rpmSpeed = rpm;
        }
        final CommandBase cmd = new InstantCommand(() -> {
            shooterWheelMotor.setSetpoint(rpmSpeed);
        }, this);
        cmd.setName("calculatedShoot");
        return cmd;
    }

    /*
     * Finds the needed velocity to reach a target (x, y) or (horizontalDistance,
     * verticalDistance). The formula takes takes theta or launch angle, target and
     * gravity.
     */
    public static double calculateVelocity() {
        // Checks if the target is within reach, plus a 12.5% leniency rate- incase lift
        // gets it there or something.
        if ((horizontalDistance * Math.tan(THETA)) * 1.125 >= verticalDistance) {
            final double gravitySide = GRAVITY * horizontalDistance * horizontalDistance;
            final double tanSide = horizontalDistance * Math.tan(THETA) - verticalDistance;
            final double cosSide = Math.cos(THETA) * Math.cos(THETA);

            velocity = Math.sqrt(gravitySide / tanSide / cosSide / 2);
        } else {
            velocity = 0;
        }
        return velocity;
    }
}