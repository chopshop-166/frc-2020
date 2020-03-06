package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap.LEDMap;

public class Led extends SubsystemBase {
    AddressableLED visionLED;
    AddressableLEDBuffer visionBuffer;
    final private int VisionGreen = 120;

    public Led(LEDMap map) {
        visionBuffer = map.visionBuffer();
        visionLED = map.visionLED();
        visionLED.setLength(visionBuffer.getLength());
    }

    public CommandBase visionLedOn() {
        CommandBase cmd = new InstantCommand(() -> {
            for (var i = 0; i < visionBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for green
                visionBuffer.setHSV(i, VisionGreen, 100, 100);
            }

            visionLED.setData(visionBuffer);
            visionLED.start();
        }, this);
        cmd.setName("Vision Green");
        return cmd;
    }

    public CommandBase visionLedOff() {
        CommandBase cmd = new InstantCommand(() -> {
            for (var i = 0; i < visionBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for green
                visionBuffer.setRGB(i, 0, 0, 0);
            }

            visionLED.setData(visionBuffer);
            visionLED.start();
        }, this);
        cmd.setName("LED Off");
        return cmd;
    }
}