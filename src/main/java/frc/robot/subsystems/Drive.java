package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 1) What does it do? Makes motors turn a certain amount depending on how much
 * an axis is triggered
 * 
 * 2) What modes does it have? Not at the moment
 * 
 * 3) What interactions does it have with other subsystems? Does not have any
 * interactions with other systems other than the space it shares on the robot
 * 
 * 4) How is it triggered -> OI? No triggers runs by default
 * 
 * 5) Does it store any state? Right and Left motor groups.
 * 
 * 6) Sensors? Encoders, Gyro
 */
public class Drive extends SubsystemBase {

    private final EncodedSpeedController rightMotorGroup;
    private final EncodedSpeedController leftMotorGroup;
    private final GyroBase gyro;
    private final DifferentialDrive driveTrain;

    /**
     * Gets the left and right motor(s) from robot map and then puts them into a
     * differential drive
     * 
     * @param map represents the drive map
     */
    public Drive(DifferentialDriveMap map) {
        super();
        rightMotorGroup = map.getRight();
        leftMotorGroup = map.getLeft();
        gyro = map.getGyro();
        driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    /**
     * Maps the drive axises
     * 
     * @param forward defines the forward direction
     * @param turn    defines the direction to turn
     * @return returns a run command so drive will stay running as long as drive is
     *         being called
     */
    public CommandBase drive(DoubleSupplier forward, DoubleSupplier turn) {
        return new RunCommand(() -> {
            double yAxis = forward.getAsDouble();
            double xAxis = turn.getAsDouble();
            driveTrain.arcadeDrive(yAxis, xAxis);
        }, this);
    }

    public CommandBase driveDistance(double distance, double speed) {
        return new FunctionalCommand(() -> {
            leftMotorGroup.reset();
            rightMotorGroup.reset();
        }, () -> {
            driveTrain.arcadeDrive(speed, 0);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            double avg = (leftMotorGroup.getDistance() + rightMotorGroup.getDistance()) / 2;
            return (avg >= distance);
        }, this);
    }

    public CommandBase turnDegrees(double degrees, double speed) {
        return new FunctionalCommand(() -> {
            gyro.reset();
        }, () -> {
            driveTrain.arcadeDrive(0, speed);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            return Math.abs(gyro.getAngle()) >= Math.abs(degrees);
        }, this);
    }
}
