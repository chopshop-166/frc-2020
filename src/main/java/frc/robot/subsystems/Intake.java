package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

// The Intake takes in the balls from the ground and it is raised and retracted by pneumatic pistons 
// There is only one speed for the intake motor (Member: Motor) 
// The intake interacts with the Indexer It uses motors, wheels, and pistons connected by belts and bands(The controller) 
// The intake needs to make sure the balls pass through it and don't get jammed(Member: Motor)

public class Intake extends SubsystemBase implements Loggable {
    @Log.SpeedController
    private final SendableSpeedController intakeMotor;
    private final IDSolenoid deployPiston;

    private static final double INTAKE_MOTOR_SPEED = 0.85;
    private static final double INTAKE_DISCHARGE = -0.85;

    public Intake(final RobotMap.IntakeMap map) {
        super();
        intakeMotor = map.intake();
        deployPiston = map.deployIntake();
    }

    public CommandBase cancel() {
        CommandBase cmd = new InstantCommand(() -> {

        }, this);
        cmd.setName("Intake Cancel");
        return cmd;
    }

    public CommandBase intake() {
        return new StartEndCommand(() -> {
            intakeMotor.set(INTAKE_MOTOR_SPEED);
            deployPiston.set(Value.kForward);
        }, () -> {
            intakeMotor.stopMotor();
            deployPiston.set(Value.kReverse);
        }, this);
    }

    public CommandBase pushBalls() {
        return new StartEndCommand(() -> {
            intakeMotor.set(INTAKE_DISCHARGE);
            deployPiston.set(Value.kForward);
        }, () -> {
            intakeMotor.stopMotor();
            deployPiston.set(Value.kReverse);
        }, this);
    }

    public StartEndCommand discharge() {
        return new StartEndCommand(() -> {
            intakeMotor.set(INTAKE_DISCHARGE);
        }, () -> {
            intakeMotor.stopMotor();
        }, this);
    }

    public CommandBase deployIntake() {
        return new InstantCommand(() -> {
            deployPiston.set(Value.kForward);
        }, this);
    }
}