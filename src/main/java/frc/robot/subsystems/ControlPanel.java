package frc.robot.subsystems;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;

public class ControlPanel extends SubsystemBase {

    private String gameData;
    private ColorStates color;

    private SendableSpeedController spinnerMotor;

    private static final double spinnerMotorSpeed = 1;

    public ControlPanel(RobotMap.ControlPanelMap map) {
        super();
        spinnerMotor = map.spinner();
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public enum ColorStates {

        RED, BLUE, GREEN, YELLOW, OTHER

    }

    public ColorStates getTargetColor() {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                color = ColorStates.BLUE;
                break;
            case 'G':
                color = ColorStates.GREEN;
                break;
            case 'R':
                color = ColorStates.RED;
                break;
            case 'Y':
                color = ColorStates.YELLOW;
                break;
            default:
                color = ColorStates.OTHER;
                break;
            }
        } else {
            // Code for no data received yet
        }
        return color;
    }

    public CommandBase spinForwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(-spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }

    public CommandBase spinBackwards() {
        return new StartEndCommand(() -> {
            spinnerMotor.set(spinnerMotorSpeed);
        }, () -> {
            spinnerMotor.stopMotor();
        }, this);
    }

    public final I2C.Port i2cPort = I2C.Port.kOnboard;

    public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public final ColorMatch m_colorMatcher = new ColorMatch();
    public static String colorString1 = "";

    public ColorStates detectColor() {

        Color detectedColor = m_colorSensor.getColor();

        ColorStates colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = ColorStates.BLUE;
        } else if (match.color == kRedTarget) {
            colorString = ColorStates.RED;
        } else if (match.color == kGreenTarget) {
            colorString = ColorStates.GREEN;
        } else if (match.color == kYellowTarget) {
            colorString = ColorStates.YELLOW;
        } else {
            colorString = ColorStates.OTHER;
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString.name());
        return colorString;

    }

    public CommandBase stageTwoRotation() {
        return new CommandBase() {
            int i = 0;
            ColorStates firstColor = detectColor();

            @Override
            public void initialize() {

                ColorStates firstColor = detectColor();
                spinForwards();
                super.initialize();
            }

            @Override
            public boolean isFinished() {

                return 40 < i && i < 50;

            }

            @Override
            public void execute() {

                ColorStates secondColor = detectColor();
                if (firstColor != secondColor) {

                    i++;
                }

            }

            @Override
            public void end(boolean interrupted) {
                // TODO Auto-generated method stub
                spinnerMotor.stopMotor();
                super.end(interrupted);

            }

        };

    }

};
