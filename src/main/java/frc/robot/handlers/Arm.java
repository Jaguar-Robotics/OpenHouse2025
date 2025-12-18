package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem; // your pivot
import frc.robot.Constants;

public class Arm extends SubsystemBase implements StateSubsystem {

  public enum PivotState implements State {
    IDLE,
    STOW,
    INTAKE
  }

  private static Arm instance;
  private final ArmSubsystem pivot = ArmSubsystem.getInstance(); // renamed for clarity

  private PivotState desiredState = PivotState.IDLE;
  private PivotState currentState = PivotState.IDLE;

  private Arm() {}

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state) {
    if (state instanceof PivotState pivotState && desiredState != pivotState) {
      desiredState = pivotState;
    }
  }

  @Override
  public void handleStateTransition() {
    // In 2910-style, we often just call update() continuously,
    // so this can delegate to update() or handle immediate logic if needed
    update();
  }

  @Override
  public void periodic() {
    update();
  }

  /** Called periodically to apply the correct target to the pivot subsystem. */
  public void update() {
    switch (desiredState) {
      case STOW:
        pivot.setAngle(Constants.IntakeConstants.STOW);
        break;

      case INTAKE:
        pivot.setAngle(Constants.IntakeConstants.INTAKE);
        break;

      case IDLE:
      default:
        pivot.stop();
        break;
    }

    currentState = desiredState;
  }

  public PivotState getCurrentState() {
    return currentState;
  }
}
