package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
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
    public final static double THETA = 37;
    public final static double TARGET_HEIGHT = 98.25;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        shooterHeight = map.shooterHeight();
        shooterWheelMotor = map.shooterWheel();
        verticalDistance = TARGET_HEIGHT - shooterHeight;
    }

    public CommandBase spinUp() {
        return new InstantCommand(() -> {
            shooterWheelMotor.set(8.5); // Make a calculateRPM() function or something to get the speed
        }, this);
    }

    public CommandBase spinDown() {
        return new InstantCommand(() -> {
            shooterWheelMotor.stopMotor();
        }, this);
    }

    /**
     * Finds the needed velocity to hit target (inches) (x,y) with a given launch
     * angle (THETA).
     */

    public static double calculateVelocity(final double distanceToTarget) {
        /**
         * xtanθ has to be greater than y- if it isn't then it is impossible to reach
         * that point with given launch angle. Eg. trying to hit a target at (1, 100)
         * with a launch angle of 1 degree.
         */
        if (distanceToTarget * Math.tan(Math.toRadians(THETA)) >= verticalDistance) {
            final double GRAVITY_SIDE = GRAVITY * distanceToTarget * distanceToTarget;
            final double TAN_SIDE = distanceToTarget * Math.tan(Math.toRadians(THETA)) - verticalDistance;
            final double COS_SIDE = Math.cos(Math.toRadians(THETA)) * Math.cos(Math.toRadians(THETA));

            final double velocity = Math.sqrt(GRAVITY_SIDE / TAN_SIDE / COS_SIDE / 2);
            return velocity;
        } else {
            return 0;
        }
    }
}