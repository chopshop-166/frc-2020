package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    public double testSpeed = INITIATION_LINE_SPEED;
    private final static double INITIATION_LINE_SPEED = 4400;
    public final static double TRENCH_SPEED = 5000;
    private final static double TESTING_SPEED = 2500;
    public final static double SHOOTER_SPEED = INITIATION_LINE_SPEED;

    // inches/second/second
    public final static double GRAVITY = 386.2205;
    // inches
    public final static double TARGET_HEIGHT = 98.25;
    public final static double THETA = Math.toRadians(60);
    // RPM equal to 1ft/s
    public final static double BALL_SPEED_RATIO = 27.358;

    // Original Value: 5400
    public final static double MAX_SPEED = 5400;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        shooterEncoder = shooterWheelMotor.getEncoder();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
    }

    @Config
    public void setshooterSpeed(double speed) {
        testSpeed = speed;
    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Shooter Cancel");
        return cmd;
    }

    @Override
    public void periodic() {
        // Distance is measured in meters
        distanceToTarget = SmartDashboard.getNumber("Distance To Target", 3.8);
        horizontalDistance = Math.sqrt((verticalDistance * verticalDistance) - (distanceToTarget * distanceToTarget));
        super.periodic();
    }

    public CommandBase spinUpForDistance() {
        return linearSpinUp(() -> {
            // Distance is measured in meters
            double dist = SmartDashboard.getNumber("Distance To Target", 3.8);
            if (!SmartDashboard.getBoolean("Sees Target", false)) {
                output = INITIATION_LINE_SPEED;
            } else {
                // calculated through linear regression in an excel spreadsheet
                output = 2800.7 * (Math.pow(dist, 0.3094));
                SmartDashboard.putNumber("VisionShooterTarget", output);
            }
            // Caps speed to ensure we can achieve the speed
            return Math.min(MAX_SPEED, output);
        });

    }

    public CommandBase testShoot() {
        return linearSpinUp(() -> testSpeed);
    }

    public CommandBase spinUp(double speed) {
        return linearSpinUp(() -> speed);
    }

    public CommandBase linearSpinUp(DoubleSupplier speed) {
        CommandBase cmd = new CommandBase() {
            {
                addRequirements(Shooter.this);
            }
            PersistenceCheck check = new PersistenceCheck(3, () -> {
                return (Math.abs(shooterEncoder.getRate() - Math.min(speed.getAsDouble(), MAX_SPEED)) <= 200.0);
            });

            @Override
            public void initialize() {
                output = Math.min(speed.getAsDouble(), MAX_SPEED);
                shooterWheelMotor.setSetpoint(Math.min(speed.getAsDouble(), MAX_SPEED));
                check.reset();
            }

            @Override
            public boolean isFinished() {
                return check.getAsBoolean();
            }
        };
        cmd.setName("Linear Spin Up");
        return cmd;
    }

    public CommandBase slowSpin() {
        final CommandBase cmd = new InstantCommand(() -> shooterWheelMotor.setSetpoint(2000), this);
        cmd.setName("Slow Spin");
        return cmd;
    }

    public CommandBase stopShooter() {
        final CommandBase cmd = new InstantCommand(shooterWheelMotor::stopMotor, this);
        cmd.setName("Stop Shooter");
        return cmd;
    }

    public CommandBase mandatoryEvacuation() {
        final CommandBase cmd = new StartEndCommand(() -> {
            shooterWheelMotor.set(0.3);
        }, () -> {
            slowSpin();
        }, this);
        cmd.setName("Mandatory Evacuation");
        return cmd;
    }

    /*
     * Calculates RPM with some gear ratio mathematics. (returns inches/second) Also
     * applies a 15% increase.
     */
    /*
     * public CommandBase calculatedShoot() { final double rpmSpeed;
     * 
     * // If it doesn't see the target, it will just shoot at the last speed. if
     * (SmartDashboard.getBoolean("Sees Target", false)) { rpmSpeed =
     * SmartDashboard.getNumber("Last RPM", 0); } else { final double rpm =
     * calculateVelocity() * BALL_SPEED_RATIO * 1.15;
     * SmartDashboard.putNumber("Last RPM", rpm); rpmSpeed = rpm; } final
     * CommandBase cmd = new InstantCommand(() -> {
     * shooterWheelMotor.setSetpoint(rpmSpeed); }, this);
     * cmd.setName("calculatedShoot"); return cmd; }
     *
     * Finds the needed velocity to reach a target (x, y) or (horizontalDistance,
     * verticalDistance). The formula takes takes theta or launch angle, target and
     * gravity.
     * 
     * public static double calculateVelocity() { // Checks if the target is within
     * reach, plus a 12.5% leniency rate- incase lift // gets it there or something.
     * if ((horizontalDistance * Math.tan(THETA)) * 1.125 >= verticalDistance) {
     * final double gravitySide = GRAVITY * horizontalDistance * horizontalDistance;
     * final double tanSide = horizontalDistance * Math.tan(THETA) -
     * verticalDistance; final double cosSide = Math.cos(THETA) * Math.cos(THETA);
     * 
     * velocity = Math.sqrt(gravitySide / tanSide / cosSide / 2); } else { velocity
     * = 0; } return velocity; }
     */
}