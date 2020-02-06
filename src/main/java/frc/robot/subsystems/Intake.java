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
    private SendableSpeedController intakeMotor;
    private SendableSpeedController singulatorMotor;
    private SendableSpeedController pierreMotor;

    private static final double motorSpeed = 0.5;

    public Intake(final RobotMap.IntakeMap map) {
        super();
        intakeMotor = map.intake();
        singulatorMotor = map.singulator();

    }

    public CommandBase runRoller() {
        return new StartEndCommand(() -> {
            intakeMotor.set(motorSpeed);
            singulatorMotor.set(motorSpeed);
            pierreMotor.set(.75);
        }, () -> {
            intakeMotor.stopMotor();
            singulatorMotor.stopMotor();
            pierreMotor.stopMotor();
        }, this);
    }

    public CommandBase runIntakeReverse() {
        return new StartEndCommand(() -> {
            intakeMotor.set(-motorSpeed);
        }, () -> {
            intakeMotor.stopMotor();

        }, this);
    }

}
