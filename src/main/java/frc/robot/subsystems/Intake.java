package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

//The intake takes in the balls from the ground
//There are two modes which are: Extended(Motors are on) and Retracted(Motors are off)
//It is extended and retracted with pistons and there are motorized belts to control everything

public class Intake extends SubsystemBase {
    private final SendableSpeedController rollerMotor;
    private final IDSolenoid deployPiston;

    private static final double rollerMotorSpeed = 0.85;

    public Intake(final RobotMap.IntakeMap map) {
        super();
        rollerMotor = map.roller();
        deployPiston = map.deployPiston();
    }

    private CommandBase runRoller(final double motorSpeed) {
        return new StartEndCommand(() -> {
            rollerMotor.set(motorSpeed);
        }, () -> {
            rollerMotor.stopMotor();
        }, this);
    }

    public CommandBase intake() {
        return runRoller(rollerMotorSpeed);
    }

    public CommandBase discharge() {
        return runRoller(-rollerMotorSpeed);
    }

    public InstantCommand setPiston(final Value direction) {
        return new InstantCommand(() -> {
            deployPiston.set(direction);
        }, this);
    }

    // pneumatic pistons to raise the intake up so that balls can get underneath

    public InstantCommand deployPiston() {
        return setPiston(Value.kForward);
    }

    public InstantCommand retractPiston() {
        return setPiston(Value.kReverse);
    }
}