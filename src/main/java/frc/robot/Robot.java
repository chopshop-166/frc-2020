/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.CommandUtils;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.triggers.DpadTrigger;
import frc.robot.triggers.DpadTrigger.DpadDirection;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends CommandRobot {

    private Command autonomousCommand;
    final private ButtonXboxController driveController = new ButtonXboxController(1);

    final private NetworkTableEntry nameEntry = Shuffleboard.getTab("RobotData").addPersistent("RobotName", "Unknown")
            .getEntry();
    final private String robotName = nameEntry.getString("Unknown");

    final private RobotMap map = getMapForName(robotName, RobotMap.class, "frc.robot.maps", new RobotMap());

    final private Drive drive = new Drive(map.getDriveMap());
    final private Intake intake = new Intake(map.getIntakeMap());
    final private Shooter shooter = new Shooter(map.getShooterMap());
    final private Indexer indexer = new Indexer(map.getIndexerMap());
    final private Led led = new Led(map.getLEDMap());

    final private SendableChooser<Command> autoChooser = new SendableChooser<>();

    UsbCamera camera0;
    VideoSink videoSink;
    boolean camera0Active = true;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit();
        Logger.configureLoggingAndConfig(this, false);
        configureButtonBindings();
        nameEntry.setPersistent();
        SmartDashboard.putData("loadtotop", indexer.loadBallToTop());
        SmartDashboard.putData("unloadBallToShooter", indexer.unloadBall());
        SmartDashboard.putData("ShootNoSpinup", indexer.shootBall());
        SmartDashboard.putData("index ball", indexer.indexBall());
        SmartDashboard.putData("Deploy intake", intake.deployIntake());
        SmartDashboard.putData("Retract intake", intake.retractIntake());

        SmartDashboard.putData("cam toggle", camToggle());
        SmartDashboard.putData("vision align only", drive.visionAlignDegrees());
        SmartDashboard.putData("vision align", visionAlignment());
        SmartDashboard.putData("ring light on", led.ringLightOn());
        SmartDashboard.putData("ring light off", led.ringLightOff());

        autoChooser.setDefaultOption("Nothing", new InstantCommand());
        autoChooser.addOption("Pass the Line", drive.drivePastLine());
        autoChooser.addOption("Shoot 3 Balls and Pass Line", shootAuto());

        Shuffleboard.getTab("Shuffleboard").add("Autonomous", autoChooser);

        drive.setDefaultCommand(drive.drive(driveController::getTriggers, () -> driveController.getX(Hand.kLeft)));
        indexer.setDefaultCommand(indexer.indexBall());

        // protovision
        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera0.setResolution(320, 240);
        camera0.setFPS(20);
        // videoSink.getProperty("compression").set(70);
        videoSink = CameraServer.getInstance().getServer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        Logger.updateEntries();

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        RobotUtils.resetAll(this);
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    public CommandBase regurgitate() {
        final CommandBase cmd = new ParallelCommandGroup(intake.discharge(), indexer.discharge());
        cmd.setName("Regurgitate");
        return cmd;
    }

    public CommandBase shootAuto() {
        final CommandBase cmd = new SequentialCommandGroup(shooter.spinUp(4500), shootNBalls(3), shooter.slowSpin(),
                drive.drivePastLine());
        cmd.setName("Shoot Auto");
        return cmd;
    }

    public CommandBase shootNBalls(final int ballAmount) {
        final CommandBase cmd = CommandUtils.repeat(ballAmount,
                new SequentialCommandGroup(shooter.spinUpForDistance(), indexer.shootBall()));
        cmd.setName("Shoot All Balls");
        return cmd;
    }

    public CommandBase maxSpeedNBalls() {
        final CommandBase cmd = new SequentialCommandGroup(shooter.linearSpinUp(() -> Shooter.MAX_SPEED),
                indexer.loadBallToTop(), indexer.unloadBall());
        cmd.setName("Shoot All Balls");
        return cmd;
    }

    public CommandBase systemsCheck() {
        final CommandBase cmd = new SequentialCommandGroup(intake.intake(), shooter.spinUp(1000),
                indexer.shootAllBalls(1));
        cmd.setName("SYSTEMS CHECK");
        return cmd;
    }

    public CommandBase cancelAll() {
        final CommandBase cmd = new ParallelCommandGroup(drive.cancel(), indexer.cancel(), intake.cancel(),
                shooter.cancel());
        cmd.setName("Cancel All");
        return cmd;
    }

    public CommandBase camToggle() {
        final CommandBase camTogglecmd = new StartEndCommand(() -> {
            SmartDashboard.putBoolean("Is Shooting", true);
        }, () -> {
            SmartDashboard.putBoolean("Is Shooting", false);
        });
        camTogglecmd.setName("camToggle");
        return camTogglecmd;
    }

    public CommandBase enableTargeting() {
        final CommandBase cmd = new InstantCommand(() -> {
            SmartDashboard.putBoolean("Is Shooting", true);

        });
        cmd.setName("Targeting On");
        return cmd;
    }

    public CommandBase disableTargeting() {
        final CommandBase cmd = new InstantCommand(() -> {
            SmartDashboard.putBoolean("Is Shooting", false);
        });
        cmd.setName("Targeting Off");
        return cmd;
    }

    public CommandBase visionAlignment() {
        final CommandBase cmd = new SequentialCommandGroup(enableTargeting(), led.ringLightOn(),
                drive.visionAlignDegrees());
        cmd.setName("Vision Alignment");
        return cmd;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveController.getButton(Button.kA).whenHeld(intake.intake());
        driveController.getButton(Button.kB).whileHeld(shootNBalls(5)).whenReleased(shooter.slowSpin());
        driveController.getButton(Button.kX).whileHeld(maxSpeedNBalls()).whenReleased(shooter.slowSpin());
        driveController.getButton(Button.kY).toggleWhenActive(
                drive.drive(() -> -driveController.getTriggers(), () -> driveController.getX(Hand.kLeft)));
        driveController.getButton(Button.kBack).whenPressed(cancelAll());
        driveController.getButton(Button.kStart).whenPressed(visionAlignment());
        driveController.getButton(Button.kBumperRight).whenHeld(drive.slowTurn(true));
        driveController.getButton(Button.kBumperLeft).whenHeld(drive.slowTurn(false));
        new DpadTrigger(driveController, DpadDirection.Up).whenActive(shooter.slowSpin());
        new DpadTrigger(driveController, DpadDirection.Down).whenActive(shooter.stopShooter());
        new DpadTrigger(driveController, DpadDirection.Left).whenActive(intake.deployIntake());
        new DpadTrigger(driveController, DpadDirection.Right).whenActive(intake.retractIntake());
    }
}
