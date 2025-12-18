package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements StateSubsystem {

    public enum IntakeState implements State {
        OFF,
        INTAKING,
        OUTTAKING
    }

    private static Intake instance;
    private final IntakeRollerSubsystem intake = new IntakeRollerSubsystem();

    private IntakeState desiredState = IntakeState.OFF;
    private IntakeState currentState = IntakeState.OFF;

    private Intake() {}

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    @Override
    public void setDesiredState(State state) {
        if (state instanceof IntakeState intakeState && desiredState != intakeState) {
            desiredState = intakeState;
        }
    }

    @Override
    public void handleStateTransition() {
        // Optional: delegate to update
        update();
    }

    @Override
    public void update() {
        switch (desiredState) {
            case INTAKING:
                intake.set(Constants.IntakeConstants.In);
                break;
            case OUTTAKING:
                intake.set(Constants.IntakeConstants.Out);
                break;
            case OFF:
            default:
                intake.stop();
                break;
        }
        currentState = desiredState;
    }

    public IntakeState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
        update();
    }
}
