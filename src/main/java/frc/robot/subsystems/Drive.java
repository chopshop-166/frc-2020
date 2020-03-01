package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private final SendableSpeedController rightMotorGroup;
    private final SendableSpeedController leftMotorGroup;
    private final GyroBase gyro;
    private final DifferentialDrive driveTrain;
    private double ratioOffset = SmartDashboard.getNumber("Ratio Offset", 0);

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
        driveTrain.setRightSideInverted(false);
    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Drive Cancel");
        return cmd;
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
        CommandBase cmd = new RunCommand(() -> {
            double yAxis = forward.getAsDouble();
            double xAxis = turn.getAsDouble();
            driveTrain.arcadeDrive(yAxis, xAxis);
            SmartDashboard.putNumber("Left Drive Encoder", leftMotorGroup.getEncoder().getDistance());
            SmartDashboard.putNumber("Right Drive Encoder", rightMotorGroup.getEncoder().getDistance());
        }, this);
        cmd.setName("Drive");
        return cmd;
    }

    public CommandBase arcadeTurning() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            driveTrain.arcadeDrive(0, (SmartDashboard.getNumber("Ratio Offset", 0) * .5));
        }, (interrupted) -> {

        }, () -> {
            return Math.abs(SmartDashboard.getNumber("Ratio Offset", 0)) <= 0.03;
        }, this);
        cmd.setName("Arcade Drive turning");
        return cmd;
    }

    public CommandBase driveDistance(double distance, double speed) {
        CommandBase cmd = new FunctionalCommand(() -> {
            leftMotorGroup.getEncoder().reset();
            rightMotorGroup.getEncoder().reset();
        }, () -> {
            driveTrain.arcadeDrive(speed, 0);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            double avg = (leftMotorGroup.getEncoder().getDistance() + rightMotorGroup.getEncoder().getDistance()) / 2;
            return (avg >= distance);
        }, this);
        cmd.setName("Drive Distance");
        return cmd;
    }

    public CommandBase turnDegrees(double degrees, double speed) {
        CommandBase cmd = new FunctionalCommand(() -> {
            gyro.reset();
        }, () -> {
            double realSpeed = speed;
            if (degrees < 0 && speed > 0) {
                realSpeed *= -1;
            }
            driveTrain.arcadeDrive(0, realSpeed);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            return Math.abs(gyro.getAngle()) >= Math.abs(degrees);
        }, this);
        cmd.setName("Turn Degrees");
        return cmd;
    }

    // public CommandBase visionAdjust() {

    // CommandBase cmd = new FunctionalCommand(() -> {
    // if (ratioOffset >= 0) {
    // speed = .5;
    // } else {
    // speed = -.5;
    // ratioOffset *= -1;
    // }
    // gyro.reset();
    // }, () -> {

    // if (gyro.getAngle() - ratioOffset != 0) {
    // turnDegrees(ratioOffset, speed);
    // // driveTrain.arcadeDrive(0, speed);
    // }
    // }, (interrupted) -> {
    // driveTrain.stopMotor();
    // }, () -> {
    // return Math.abs(gyro.getAngle()) >= Math.abs(ratioOffset);
    // }, this);
    // cmd.setName("Turn Vision good");
    // return cmd;
    // }
}
