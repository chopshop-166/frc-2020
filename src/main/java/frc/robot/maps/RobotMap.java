package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.DifferentialDriveMap;
import com.chopshop166.chopshoplib.outputs.EncodedSpeedController;
import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockSpeedController;
import com.chopshop166.chopshoplib.outputs.PIDSpeedController;
import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.MockEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class RobotMap {

    public DifferentialDriveMap getDriveMap() {
        return new DifferentialDriveMap() {

            @Override
            public EncodedSpeedController getRight() {
                return getMock();
            }

            @Override
            public EncodedSpeedController getLeft() {
                return getMock();
            }

            private EncodedSpeedController getMock() {
                return EncodedSpeedController.join(new MockSpeedController(), new MockEncoder());
            }
        };
    }

    public static class IntakeMap {
        public SendableSpeedController intake() {
            return new MockSpeedController();
        }

        public IDSolenoid deployIntake() {
            return new MockDSolenoid();
        }
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class ShooterMap {
        public PIDSpeedController shooterWheel() {
            return new PIDSpeedController() {

                @Override
                public void initSendable(SendableBuilder builder) {
                    // TODO Auto-generated method stub

                }

                @Override
                public void set(double speed) {
                    // TODO Auto-generated method stub

                }

                @Override
                public double get() {
                    // TODO Auto-generated method stub
                    return 0;
                }

                @Override
                public void setInverted(boolean isInverted) {
                    // TODO Auto-generated method stub

                }

                @Override
                public boolean getInverted() {
                    // TODO Auto-generated method stub
                    return false;
                }

                @Override
                public void disable() {
                    // TODO Auto-generated method stub

                }

                @Override
                public void stopMotor() {
                    // TODO Auto-generated method stub

                }

                @Override
                public void pidWrite(double output) {
                    // TODO Auto-generated method stub

                }

                @Override
                public void setP(double kp) {
                    // TODO Auto-generated method stub

                }

                @Override
                public void setI(double ki) {
                    // TODO Auto-generated method stub

                }

                @Override
                public void setD(double kd) {
                    // TODO Auto-generated method stub

                }

                @Override
                public void setSetpoint(double setPoint) {
                    // TODO Auto-generated method stub

                }
            };
        }

        public double shooterHeight() {
            return 0;
        }
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();

    }

    public static class ControlPanelMap {
        public SendableSpeedController spinner() {
            return new MockSpeedController();
        }
    }

    public ControlPanelMap getControlPanelMap() {
        return new ControlPanelMap();
    }

    public static class LiftMap {
        public SendableSpeedController elevator() {
            return new MockSpeedController();
        }
    }

    public LiftMap getLiftMap() {
        return new LiftMap();
    }

}