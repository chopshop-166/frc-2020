package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class Intake extends SubsystemBase {
    private SendableSpeedController intakeMotor;
    private SendableSpeedController singulatorMotor;
    private SendableSpeedController pierreMotor;

    private static final double motorSpeed = 0.5;

    public Intake(RobotMap.IntakeMap map) {
        super();
        intakeMotor = map.intake();
        singulatorMotor = map.singulator();
        pierreMotor = map.pierre();
    }

    public CommandBase runRoller() {
        return new StartEndCommand(() -> {
            intakeMotor.set(motorSpeed);
            singulatorMotor.set(motorSpeed);
            pierreMotor.set(motorSpeed);
        }, () -> {
            intakeMotor.stopMotor();
            singulatorMotor.stopMotor();
            pierreMotor.stopMotor();
        }, this);
    }
}