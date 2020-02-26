/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.chopshop166.chopshoplib.DashboardUtils;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.triggers.XboxTrigger;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.triggers.EndGameTrigger;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    final private ButtonXboxController driveController = new ButtonXboxController(1);
    final private ButtonXboxController copilotController = new ButtonXboxController(5);

    final private NetworkTableEntry nameEntry = Shuffleboard.getTab("RobotData").addPersistent("RobotName", "Unknown")
            .getEntry();
    final private String robotName = nameEntry.getString("Unknown");

    final private RobotMap map = RobotUtils.getMapForName(robotName, RobotMap.class, "frc.robot.maps", new RobotMap());

    final private Drive drive = new Drive(map.getDriveMap());
    final private Intake intake = new Intake(map.getIntakeMap());
    final private Shooter shooter = new Shooter(map.getShooterMap());
    final private ControlPanel controlPanel = new ControlPanel(map.getControlPanelMap());
    final private Lift lift = new Lift(map.getLiftMap());
    final private Indexer indexer = new Indexer(map.getIndexerMap());

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
        Logger.configureLoggingAndConfig(this, true);
        configureButtonBindings();
        nameEntry.setPersistent();
        SmartDashboard.putData("bottom pierre", indexer.pierrePossesion());
        SmartDashboard.putData("loadtotop", indexer.loadBallToTop());
        SmartDashboard.putData("runtoclear", indexer.runToClearBottomSensor());
        SmartDashboard.putData("lift brake toggle", lift.toggleBrake());
        SmartDashboard.putData("Deploy intake", intake.deployIntake());

        // SmartDashboard.putNumber("Ball Count", indexer.ballCounting);

        autoChooser.setDefaultOption("Nothing", new InstantCommand());
        autoChooser.addOption("Pass the Line", drive.driveDistance(20, .5));
        autoChooser.addOption("Shoot 3 Balls and Pass Line", shootAuto());

        Shuffleboard.getTab("Shuffleboard").add("Autonomous", autoChooser);

        DashboardUtils.logTelemetry();

        drive.setDefaultCommand(drive.drive(driveController::getTriggers, () -> driveController.getX(Hand.kLeft)));
        lift.setDefaultCommand(lift.moveLift(() -> -copilotController.getTriggers()));
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

    public CommandBase regurgitateFC() {
        CommandBase cmd = new ParallelCommandGroup(intake.discharge(), indexer.reversePush());
        cmd.setName("Regurgitate FC");
        return cmd;
    }

    public CommandBase shootAuto() {
        CommandBase cmd = new SequentialCommandGroup(releaseBalls(3), shooter.spinDown(), drive.driveDistance(20, .5));
        cmd.setName("Shoot Auto");
        return cmd;
    }

    // will spin the shooter then shoot all the balls and then turn the shooter off.
    public CommandBase releaseBalls(int ballCount) {
        CommandBase cmd = new SequentialCommandGroup(shooter.spinUp(4500), indexer.shootAllBalls(ballCount));
        cmd.setName("Shoot all Balls");
        return cmd;
    }

    // in the future we will add the vision lining up command to this.
    public CommandBase endGame() {
        CommandBase endGameCmd = new SequentialCommandGroup(intake.deployIntake(),
                lift.disengageRatchet(() -> -copilotController.getTriggers()));
        endGameCmd.setName("End Game Lift");
        return endGameCmd;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveController.getButton(Button.kA).whenHeld(intake.intake()).whileHeld(indexer.intakeToPierre());
        driveController.getButton(Button.kB).whenHeld(releaseBalls(5)).whenReleased(shooter.spinDown());
        driveController.getButton(Button.kY).toggleWhenActive(
                drive.drive(() -> -driveController.getTriggers(), () -> driveController.getX(Hand.kLeft)));

        copilotController.getButton(Button.kA).whenHeld(intake.intake()).whileHeld(indexer.intakeToPierre());
        copilotController.getButton(Button.kBumperRight).whenPressed(shooter.spinUp(5000));
        copilotController.getButton(Button.kBumperLeft).whenHeld(shooter.spinDown());
        XboxTrigger endTrigger = new XboxTrigger(copilotController, Hand.kRight);
        endTrigger.and(new EndGameTrigger(120)).whenActive(endGame());
        copilotController.getButton(Button.kB).whenHeld(controlPanel.spinForwards());

    }
}