package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

/**
 * What does it do? When the A button is pressed- it shoots a ball. What modes
 * does it have? Semi-Auto and DUMP. what interactions does it have with other
 * subsystems? Asks the Indexer and 'vision' if it's ready to shoot. How is it
 * triggered/OI? A semi-auto button A- or X for shoot all. Does it store any
 * state? No. Sensors? No.
 */

public class Shooter extends SubsystemBase {

    private final SendableSpeedController shooterWheelMotor;
    public final double shooterHeight;
    public static double verticalDistance;
    public final static double GRAVITY = 386.09;
    public final static double THETA = Math.toRadians(37);
    public final static double TARGET_HEIGHT = 98.25;
    public static double distanceToTarget;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        distanceToTarget = SmartDashboard.getNumber("Distance To Target", 0);
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
    }

    public CommandBase spinUp() {
        return new InstantCommand(() -> {
            shooterWheelMotor.set(.85); // Make a calculateRPM() function or something to get the speed
        }, this);
    }

    public CommandBase spinDown() {
        return new InstantCommand(shooterWheelMotor::stopMotor, this);
    }

    /**
     * Finds the needed velocity to reach a target (x, y) or (distanceToTarget,
     * verticalDistance). The formula takes takes theta or launch angle, target and
     * gravity.
     */

    public static double calculateVelocity(final double distanceToTarget) {
        if (distanceToTarget * Math.tan(THETA) >= verticalDistance) {
            final double gravitySide = GRAVITY * distanceToTarget * distanceToTarget;
            final double tanSide = distanceToTarget * Math.tan(THETA) - verticalDistance;
            final double cosSide = Math.cos(THETA) * Math.cos(THETA);

            return Math.sqrt(gravitySide / tanSide / cosSide / 2);
        } else {
            return 0;
        }
    }
}