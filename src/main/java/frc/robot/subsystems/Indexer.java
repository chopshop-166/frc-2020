package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.IndexMap;

// The indexer lines up and transports the balls to the Shooter
// The indexer has two modes, On(motors on) and Off(motors off)
// The indexer relates to the Shooter in that it should only put the balls in when the Shooter is ready
// The indexer is triggered with motors and uses polycords to move the balls(controller)
// The indexer will use IR sensors so we can see the presence of a ball
// The distance that the balls are pushed between the sensors needs to be coded so that their is enough space in between the balls at each point of the tunnel

public class Indexer extends SubsystemBase {
    final SendableSpeedController singulator;

    private static final double indexMotorSpeed = 0.85;

    public Indexer(final IndexMap map) {
        super();
        singulator = map.indexMotor();
    }

    private CommandBase indexMotor(final double motorSpeed) {
        return new StartEndCommand(() -> {
            singulator.set(motorSpeed);
        }, () -> {
        });
    }

    public CommandBase fast() {
        return indexMotor(indexMotorSpeed);
    }

    public CommandBase slow() {
        return indexMotor(-indexMotorSpeed);
    }

}
