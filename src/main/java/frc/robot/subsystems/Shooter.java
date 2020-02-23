package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.PIDSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

/**
 * What does it do? Given a target and shooter height, it calculates velocity
 * needed to hit said target, and spins the motors at a corresponding RPM. What
 * modes does it have? Semi-Auto and DUMP. what interactions does it have with
 * other subsystems? Asks the Indexer if there is a ball, and get's information
 * about the target from vision. How is it triggered/OI? A semi-auto button A-
 * or X for shoot all. Does it store any state? No. Sensors? No.
 */

public class Shooter extends SubsystemBase {
    private final PIDSpeedController shooterWheelMotor;
    public static double distanceToTarget;
    public final double shooterHeight;
    public static double verticalDistance;
    public static double horizontalDistance;

    // inches/second/second
    public final static double GRAVITY = 386.2205;
    // inches
    public final static double TARGET_HEIGHT = 98.25;
    private final static double MAX_RPM = 5200;
    public final static double THETA = Math.toRadians(60);
    // RPM equal to 1ft/s
    public final static double BALL_SPEED_RATIO = 27.358;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
        SendableRegistry.add(shooterWheelMotor.getEncoder(), "Shooter");
        SendableRegistry.enableLiveWindow(shooterWheelMotor.getEncoder());
    }

    @Override
    public void periodic() {
        distanceToTarget = SmartDashboard.getNumber("Distance To Target", 160);
        horizontalDistance = Math.sqrt((verticalDistance * verticalDistance) - (distanceToTarget * distanceToTarget));
        super.periodic();
    }

    public CommandBase spinUp(final double speed) {
        CommandBase cmd = new InstantCommand(() -> {
            // TODO incorperate calculations
            // shooterWheelMotor.set(calculateRPM() / MAX_RPM);
            shooterWheelMotor.setSetpoint(speed);
        }, this);
        cmd.setName("spinUp");
        return cmd;
    }

    public CommandBase spinDown() {
        CommandBase cmd = new InstantCommand(shooterWheelMotor::stopMotor, this);
        cmd.setName("spinDown");
        return cmd;
    }

    public CommandBase mandatoryEvacuation() {
        CommandBase cmd = new StartEndCommand(() -> {
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
        CommandBase cmd = new InstantCommand(() -> {
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

            return Math.sqrt(gravitySide / tanSide / cosSide / 2);
        } else {
            return 0;
        }

    }
}