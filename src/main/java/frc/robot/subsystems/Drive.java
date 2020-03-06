package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

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
public class Drive extends SubsystemBase implements Loggable {

    @Log.SpeedController
    private final SendableSpeedController rightMotorGroup;
    @Log.SpeedController
    private final SendableSpeedController leftMotorGroup;
    @Log.Gyro
    private final GyroBase gyro;

    private final DifferentialDrive driveTrain;
    @Log.Encoder
    private final IEncoder driveRightEncoder;
    @Log.Encoder
    private final IEncoder driveLeftEncoder;
    @Log
    private final PIDController pid;
    static int i;

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
        pid = new PIDController(0, 0, 0);
        driveRightEncoder = rightMotorGroup.getEncoder();
        driveLeftEncoder = leftMotorGroup.getEncoder();
    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Drive Cancel");
        return cmd;
    }

    public CommandBase drivePastLine() {
        return driveDistance(40, 0.5);
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
        }, this);
        cmd.setName("Drive");
        return cmd;
    }

    public CommandBase arcadeTurning() {
        CommandBase cmd = new FunctionalCommand(() -> {
        }, () -> {
            driveTrain.arcadeDrive(0, (SmartDashboard.getNumber("Ratio Offset", 0) * 0.5));
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            return Math.abs(SmartDashboard.getNumber("Ratio Offset", 0)) <= 0.1;
        }, this);
        cmd.setName("Arcade Drive turning");
        return cmd;
    }

    public CommandBase driveDistance(double distance, double speed) {
        CommandBase cmd = new FunctionalCommand(() -> {
            driveLeftEncoder.reset();
            driveRightEncoder.reset();
        }, () -> {
            driveTrain.arcadeDrive(speed, 0);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            double avg = (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2;
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

    public CommandBase slowTurn(boolean direction) {
        CommandBase cmd = new RunCommand(() -> {
            if (direction == true) {
                driveTrain.arcadeDrive(0, 0.4);
            } else if (direction == false) {
                driveTrain.arcadeDrive(0, -0.4);

            }
        }, this);
        cmd.setName("turing");
        return cmd;
    }

    public CommandBase visionAlignDegrees() {
        CommandBase cmd = new CommandBase() {
            {
                addRequirements(Drive.this);
            }

            @Override
            public void initialize() {
                gyro.reset();
                pid.setSetpoint((SmartDashboard.getNumber("Angle Offset", 0)));
                pid.setTolerance(1.5);
                pid.setPID(0.025, 0.015, 0);

            }

            @Override
            public void execute() {
                if (i % 20 == 0) {
                    gyro.reset();
                    pid.setSetpoint((SmartDashboard.getNumber("Angle Offset", 0)));
                    i = 0;
                }
                double turning = pid.calculate(-gyro.getAngle());
                SmartDashboard.putNumber("pid Out", turning);
                driveTrain.arcadeDrive(0, turning);
                i++;
            }

            @Override
            public boolean isFinished() {
                return pid.atSetpoint() || !SmartDashboard.getBoolean("Sees Target", false);
            }

            @Override
            public void end(boolean interrupted) {
                driveTrain.stopMotor();

            }
        };
        cmd.setName("Turn Degrees");
        return cmd;
    }

    public CommandBase vision() {
        CommandBase cmd = new FunctionalCommand(() -> {
            // System.out.println("first");
            gyro.reset();
            pid.setSetpoint((SmartDashboard.getNumber("Angle Offset", 0)));
            pid.setTolerance(2.5);
            pid.setPID(0.025, 0.015, 0);
            // System.out.println("second");
        }, () -> {

            double turning = pid.calculate(-gyro.getAngle());
            SmartDashboard.putNumber("pid Out", turning);
            driveTrain.arcadeDrive(0, turning);

        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            return pid.atSetpoint() || !SmartDashboard.getBoolean("Sees Target", false);

        }, this);
        cmd.setName("Turn Degrees");
        return cmd;
    }
}
