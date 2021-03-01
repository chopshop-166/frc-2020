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
import com.chopshop166.chopshoplib.triggers.XboxTrigger;

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
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
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
    final private ButtonXboxController copilotController = new ButtonXboxController(5);

    final private NetworkTableEntry nameEntry = Shuffleboard.getTab("RobotData").addPersistent("RobotName", "Unknown")
            .getEntry();
    final private String robotName = nameEntry.getString("Unknown");

    final private RobotMap map = getMapForName(robotName, RobotMap.class, "frc.robot.maps", new RobotMap());

    final private Drive drive = new Drive(map.getDriveMap());
    final private Intake intake = new Intake(map.getIntakeMap());
    final private Shooter shooter = new Shooter(map.getShooterMap());
    // final private ControlPanel controlPanel = new
    // ControlPanel(map.getControlPanelMap());
    final private Lift lift = new Lift(map.getLiftMap());
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
        SmartDashboard.putData("lift brake toggle", lift.toggleBrake());
        SmartDashboard.putData("Deploy intake", intake.deployIntake());
        SmartDashboard.putData("Retract intake", intake.retractIntake());

        SmartDashboard.putData("cam toggle", camToggle());
        SmartDashboard.putData("After Match Lift Sequence", afterMatchPit());
        SmartDashboard.putData("vision align only", drive.visionAlignDegrees());
        SmartDashboard.putData("vision align", visionAlignment());
        SmartDashboard.putData("ring light on", led.ringLightOn());
        SmartDashboard.putData("ring light off", led.ringLightOff());

        autoChooser.setDefaultOption("Nothing", new InstantCommand());
        autoChooser.addOption("Pass the Line", drive.drivePastLine());
        autoChooser.addOption("Shoot 3 Balls and Pass Line", shootAuto());

        Shuffleboard.getTab("Shuffleboard").add("Autonomous", autoChooser);

        drive.setDefaultCommand(drive.drive(driveController::getTriggers, () -> driveController.getX(Hand.kLeft)));
        lift.setDefaultCommand(lift.moveLift(() -> -copilotController.getTriggers()));
        // controlPanel.setDefaultCommand(controlPanel.spinControlPanel(() ->
        // copilotController.getX(Hand.kLeft)));
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
        final CommandBase cmd = new SequentialCommandGroup(shooter.spinUp(4500), shootNBalls(3), shooter.spinDown(),
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

    public CommandBase afterMatchPit() {
        final CommandBase cmd = new SequentialCommandGroup(lift.resetLift(), intake.retractIntake());
        cmd.setName("After Match Pit");
        return cmd;
    }

    public CommandBase systemsCheck() {
        final CommandBase cmd = new SequentialCommandGroup(intake.intake(), shooter.spinUp(1000),
                indexer.shootAllBalls(1));
        cmd.setName("SYSTEMS CHECK");
        return cmd;
    }

    // in the future we will add the vision lining up command to this.
    public CommandBase endGame() {
        final CommandBase endGameCmd = new SequentialCommandGroup(intake.deployIntake(), lift.disengageRatchet());
        endGameCmd.setName("End Game Lift");
        return endGameCmd;
    }

    public CommandBase cancelAll() {
        final CommandBase cmd = new ParallelCommandGroup(drive.cancel(), indexer.cancel(), intake.cancel(),
                lift.cancel(), shooter.cancel());
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
        driveController.getButton(Button.kB).whileHeld(shootNBalls(5)).whenReleased(shooter.spinDown());
        driveController.getButton(Button.kX).whileHeld(maxSpeedNBalls()).whenReleased(shooter.spinDown());
        driveController.getButton(Button.kY).toggleWhenActive(
                drive.drive(() -> -driveController.getTriggers(), () -> driveController.getX(Hand.kLeft)));
        driveController.getButton(Button.kBack).whenPressed(cancelAll());
        driveController.getButton(Button.kStart).whenPressed(visionAlignment());
        driveController.getButton(Button.kBumperRight).whenHeld(drive.slowTurn(true));
        driveController.getButton(Button.kBumperLeft).whenHeld(drive.slowTurn(false));

        // copilotController.getButton(Button.kB).whenPressed(controlPanel.stageTwoRotation());
        // copilotController.getButton(Button.kX).whenPressed(controlPanel.stageThreeRotation());
        copilotController.getButton(Button.kA).whenHeld(intake.intake());
        copilotController.getButton(Button.kBumperRight).whenPressed(shooter.spinUp(4400));
        copilotController.getButton(Button.kBumperLeft).whenHeld(shooter.spinDown());
        final XboxTrigger endTrigger = new XboxTrigger(copilotController, Hand.kRight);
        endTrigger.whenActive(endGame());
        copilotController.getButton(Button.kY).whenHeld(regurgitate());
        copilotController.getButton(Button.kBack).whenPressed(cancelAll());
        // copilotController.getButton(Button.kStart).whenHeld(controlPanel.spinForwards());
    }
}
