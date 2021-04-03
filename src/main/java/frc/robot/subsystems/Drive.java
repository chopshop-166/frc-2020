package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.outputs.SmartSpeedController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.DriveKinematics;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;


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
    private final SmartSpeedController rightMotorGroup;
    @Log.SpeedController
    private final SmartSpeedController leftMotorGroup;
    @Log.Gyro
    private final GyroBase gyro;

    private final DifferentialDrive driveTrain;

    private final DifferentialDriveKinematics trajectoryKinematics;

    private final DifferentialDriveOdometry odometry;
    @Log.Encoder
    private final IEncoder rightEncoder;
    @Log.Encoder
    private final IEncoder leftEncoder;
    @Log
    private final PIDController pid;

    private final PhotonCamera camera;

    // Distance gain of the trajectory controller; 2.0 should work for most robots
    private final static double RAMSETE_B = 1.7;

    // Temporal gain of the trajectory controller; 0.7 should work for most robots
    private final static double RAMSETE_ZETA = 0.7;

    // Ramsete Controller used to control the robot during auto
    RamseteController trajectoryController = new RamseteController(RAMSETE_B, RAMSETE_ZETA);

    private final double ALIGN_PID_FEED = 0.2;

    public final double MAX_SPEED_MPS = 2;

    public double MAX_ACCELERATION = 1.2;

    public final static double KS_VOLTS = 0.294;
    public final static double KV_VOLT_SPM = 3.06;
    public final static double KA_VOLT_SSPM = 0.291;

    public final static double DRIVE_VEL_P = 0.56;
    public final static double DRIVE_VEL_D = 0.0;

    Field2d field = new Field2d();

    private final double[] yawMin = {1,2};
    private final double[] yawMax = {1,2};
    private final double[] pitchMin = {1,2};
    private final double[] pitchMax = {1,2};

    private enum Quadrant {
        UNKNOWN("unknown"),
        REDA("redA"),
        REDB("redB"),
        BLUEA("blueA"),
        BLUEB("blueB");

        public final String pathName;

        private Quadrant(String pathName) {
            this.pathName = pathName;
        }

        public String getPath() {
            return pathName;
        }
    }

    /**
     * Gets the left and right motor(s) from robot map and then puts them into a
     * differential drive
     * 
     * @param map represents the drive map
     */
    public Drive(final DriveKinematics map) {
        super();
        rightMotorGroup = map.getRight();
        leftMotorGroup = map.getLeft();
        gyro = map.getGyro();
        driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        driveTrain.setRightSideInverted(false);
        trajectoryKinematics = map.getKinematics();
        pid = new PIDController(0.0106, 0.0004, 0.008);
        rightEncoder = rightMotorGroup.getEncoder();
        leftEncoder = leftMotorGroup.getEncoder();
        odometry = new DifferentialDriveOdometry(getRotation());
        SmartDashboard.putData("Field", field);
        camera = new PhotonCamera("gloworm");
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(getRotation(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, getRotation());
    }

    private double encoderAvg() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getTurnRate() {
        return gyro.getRate();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void tankDriveVolts(double left, double right) {
        leftMotorGroup.setVoltage(left);
        rightMotorGroup.setVoltage(right);
        driveTrain.feed();
    }

    public CommandBase cancel() {
        final CommandBase cmd = new InstantCommand(() -> {

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
    public CommandBase drive(final DoubleSupplier forward, final DoubleSupplier turn) {
        final CommandBase cmd = new RunCommand(() -> {
            final double yAxis = forward.getAsDouble();
            final double xAxis = turn.getAsDouble();
            driveTrain.arcadeDrive(yAxis, xAxis);
        }, this);
        cmd.setName("Drive");
        return cmd;
    }

    public CommandBase arcadeTurning() {
        final CommandBase cmd = new FunctionalCommand(() -> {
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

    public CommandBase driveDistance(final double distance, final double speed) {
        final CommandBase cmd = new FunctionalCommand(() -> {
            resetEncoders();
        }, () -> {
            driveTrain.arcadeDrive(speed, 0);
        }, (interrupted) -> {
            driveTrain.stopMotor();
        }, () -> {
            return (encoderAvg() >= distance);
        }, this);
        cmd.setName("Drive Distance");
        return cmd;
    }

    public CommandBase turnDegrees(final double degrees, final double speed) {
        final CommandBase cmd = new FunctionalCommand(() -> {
            resetGyro();
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

    public CommandBase slowTurn(final boolean isTurningRight) {
        final CommandBase cmd = new RunCommand(() -> {
            if (isTurningRight) {
                driveTrain.arcadeDrive(0, 0.4);
            } else {
                driveTrain.arcadeDrive(0, -0.4);
            }
        }, this);
        cmd.setName("Turning");
        return cmd;
    }

    public Quadrant visionField() {
        var result = camera.getLatestResult();
        var bestTarget = result.getBestTarget();

        double yaw = bestTarget.getYaw();
        double pitch = bestTarget.getPitch();

        if (result.hasTargets()) {
            for (int i=1; i<Quadrant.values().length; i++) {
                if ((yaw > yawMin[i] && yaw < yawMax[i]) && (pitch > pitchMin[i] && pitch < pitchMax[i])) {
                    return Quadrant.values()[i];
                }
            }
        }
        return Quadrant.UNKNOWN;
    }
    
    public CommandBase galacticSearch() {
        Quadrant pathName = visionField();
        return autonomousCommand(pathName.getPath());
    }

    public CommandBase visionAlignDegrees() {
        final CommandBase cmd = new CommandBase() {
            {
                addRequirements(Drive.this);
            }
            int i;
            PersistenceCheck check = new PersistenceCheck(25, () -> {
                return (pid.atSetpoint() || !SmartDashboard.getBoolean("Sees Target", false));

            });

            @Override
            public void initialize() {
                resetGyro();
                pid.setSetpoint(SmartDashboard.getNumber("Angle Offset", 0));
                pid.setTolerance(0.75);
            }

            @Override
            public void execute() {

                if (pid.getPositionError() <= 5 && (i % 50 == 0)) {
                    resetGyro();
                    pid.setSetpoint((SmartDashboard.getNumber("Angle Offset", 0)));
                    i = 0;
                }

                double turning = pid.calculate(-gyro.getAngle());
                turning += (turning < 0) ? -ALIGN_PID_FEED : ALIGN_PID_FEED;
                SmartDashboard.putNumber("pid Out", turning);
                driveTrain.arcadeDrive(0, turning);
                i++;
            }

            @Override
            public boolean isFinished() {
                return check.getAsBoolean();
            }

            @Override
            public void end(final boolean interrupted) {
                driveTrain.stopMotor();
            }
        };

        cmd.setName("Turn Degrees");

        return cmd;

    }

    public CommandBase autonomousCommand(String trajectoryName) {
        return autonomousCommand(trajectoryName, true);
    }

    public CommandBase autonomousCommand(String trajectoryName, Boolean resetPose) {

        String trajectoryJSON = "paths/" + trajectoryName + ".wpilib.json";
        Trajectory autoTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        final Trajectory finalAutoTrajectory = autoTrajectory;

        RamseteCommand ramseteCommand = new RamseteCommand(autoTrajectory,
                // Gets pose
                this::getPose,
                // Creates our ramsete controller
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                // Allows for robot control
                new SimpleMotorFeedforward(KS_VOLTS, KV_VOLT_SPM, KA_VOLT_SSPM),
                // Describes how the drivetrain is influenced by motor speed
                trajectoryKinematics,
                // Gets the speed of the wheels
                this::getWheelSpeeds,
                // Left Controller
                new PIDController(DRIVE_VEL_P, 0, DRIVE_VEL_D),
                // Right Controller
                new PIDController(DRIVE_VEL_P, 0, DRIVE_VEL_D),
                // Sends voltages to motors
                this::tankDriveVolts, this);

        CommandBase cmd;
        if (resetPose) {
            cmd = (new InstantCommand(() -> resetOdometry(finalAutoTrajectory.getInitialPose())))
                    .andThen(ramseteCommand).andThen(() -> tankDriveVolts(0, 0));
        } else {
            cmd = ramseteCommand.andThen(() -> tankDriveVolts(0, 0));
        }

        cmd.setName(trajectoryName);
        return cmd;
    }
}
