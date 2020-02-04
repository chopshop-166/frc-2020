package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

// The Intake takes in the balls from the ground and it is raised and retracted by pneumatic pistons 
// There is only one speed for the intake motor (Member: Motor) 
// The intake interacts with the Indexer It uses motors, wheels, and pistons connected by belts and bands(The controller) 
// The intake needs to make sure the balls pass through it and don't get jammed(Member: Motor)

public class Intake extends SubsystemBase {
    private final SendableSpeedController intakeMotor;
    private final IDSolenoid rollerPlacement;

    private static final double intakeMotorSpeed = 0.85;
    private static final double intakeDischarge = -0.85;

    public Intake(final RobotMap.IntakeMap map) {
        super();
        intakeMotor = map.intake();
        rollerPlacement = map.deployIntake();
    }

    // pneumatic pistons to raise the intake up so that balls can get underneath

    public CommandBase intake() {
        return new StartEndCommand(() -> {
            intakeMotor.set(intakeMotorSpeed);
            rollerPlacement.set(Value.kForward);
        }, () -> {
            intakeMotor.stopMotor();
            rollerPlacement.set(Value.kReverse);
        }, this);
    }

    public StartEndCommand discharge() {
        return new StartEndCommand(() -> {
            intakeMotor.set(intakeDischarge);
        }, () -> {
            intakeMotor.stopMotor();
        }, this);
    }

}