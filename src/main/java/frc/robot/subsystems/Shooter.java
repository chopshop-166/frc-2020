package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static double speed;
    public static double gravity;
    public static double theta;
    public static double y;

    public Shooter(final RobotMap.ShooterMap map) {
        super();
        gravity = 386.09;
        theta = 37;
        y = 98.25;
        speed = SmartDashboard.getNumber("YeetSpeed", speed);
        shooterWheelMotor = map.shooterWheel();
    }

    public CommandBase spinUp() {
        return new InstantCommand(() -> {
            shooterWheelMotor.set(speed);
        }, this);
    }

    public CommandBase spinDown() {
        return new InstantCommand(() -> {
            shooterWheelMotor.stopMotor();
        }, this);
    }

    /**
     * Finds the needed velocity to hit target (inches) (x,y) with a given launch
     * angle (theta).
     */

    public static double calculateVelocity(double x) {
        /**
         * xtanθ has to be greater than y- if it isn't then it is impossible to reach
         * that point with given launch angle. Eg. trying to hit a target at (1, 100)
         * with a launch angle of 1 degree.
         */
        if (x * Math.tan(Math.toRadians(theta)) >= y) {
            double gravitySide = gravity * x * x;
            double tanSide = x * Math.tan(Math.toRadians(theta)) - y;
            double cosSide = Math.cos(Math.toRadians(theta)) * Math.cos(Math.toRadians(theta));

            double velocity = Math.sqrt(gravitySide / tanSide / cosSide / 2);
            return velocity;
        } else {
            return 0;
        }
    }
}