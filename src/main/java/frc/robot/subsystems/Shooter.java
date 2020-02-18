package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.PIDSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * What does it do? When the A button is pressed- it shoots a ball. What modes
 * does it have? Semi-Auto and DUMP. what interactions does it have with other
 * subsystems? Asks the Indexer and 'vision' if it's ready to shoot. How is it
 * triggered/OI? A semi-auto button A- or X for shoot all. Does it store any
 * state? No. Sensors? No.
 */

public class Shooter extends SubsystemBase {

    @Config(tabName = "Configurable Values")
    public final static double THETA = Math.toRadians(37);
    public final static double DIAMETER = 4;

    private final double MAX_RPM = 5200;
    private final PIDSpeedController shooterWheelMotor;
    public static double distanceToTarget;
    public final double shooterHeight;
    public static double verticalDistance;

    // inches/second
    public final static double GRAVITY = 386.2205;
    // inches
    public final static double TARGET_HEIGHT = 98.25;
    public final static double CIRCUMFERENCE = DIAMETER * Math.PI * Math.PI;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        distanceToTarget = SmartDashboard.getNumber("Distance To Target", 0);
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
    }

    public CommandBase spinUp() {
        return new InstantCommand(() -> {
            shooterWheelMotor.set(calculateRPM() / MAX_RPM);
        }, this);
    }

    public CommandBase spinDown() {
        return new InstantCommand(shooterWheelMotor::stopMotor, this);
    }

    /*
     * Calculates RPM -> velocity / circumference is how many rotation needed in a
     * second, so times 60 gives us how many RPM we need. (returns inches/second)
     * Also applies a 20% loss.
     */
    public static double calculateRPM() {
        return 60 * calculateVelocity() / CIRCUMFERENCE * 0.8;
    }

    /*
     * Finds the needed velocity to reach a target (x, y) or (distanceToTarget,
     * verticalDistance). The formula takes takes theta or launch angle, target and
     * gravity.
     */

    public static double calculateVelocity() {
        if (distanceToTarget * Math.tan(THETA) >= verticalDistance) {
            final double gravitySide = GRAVITY * distanceToTarget * distanceToTarget;
            final double tanSide = distanceToTarget * Math.tan(THETA) - verticalDistance;
            final double cosSide = Math.cos(THETA) * Math.cos(THETA);

            return Math.sqrt(gravitySide / tanSide / cosSide / 2);
        }
        return 0;
    }
}
