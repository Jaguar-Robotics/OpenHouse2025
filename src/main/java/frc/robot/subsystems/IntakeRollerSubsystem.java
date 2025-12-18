package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final MotorController rollerMotor = new Talon(Constants.IntakeConstants.IntakeRollerID);

    public IntakeRollerSubsystem() {}

    /** Spin rollers [-1,1] */
    public void set(double speed) {
        rollerMotor.set(speed);
    }

    /** Stop rollers */
    public void stop() {
        rollerMotor.set(0);
    }
}
