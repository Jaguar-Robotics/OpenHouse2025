package frc.robot.handlers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.units.Units.Degrees;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    IDLE,
    STOW,
    INTAKE
  }

  private static Superstructure instance;
  private final ArmSubsystem arm = new ArmSubsystem();
  private final Arm pivot = Arm.getInstance();
  private final Intake intake = Intake.getInstance(); 

  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;
  private Angle targetAngle = Degrees.of(0);

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  private Superstructure() {}

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperstructureState newState) {
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case IDLE:
        pivot.setDesiredState(Arm.PivotState.IDLE);
        intake.setDesiredState(Intake.IntakeState.OFF);
        break;
      case STOW:
        pivot.setDesiredState(Arm.PivotState.STOW);
        intake.setDesiredState(Intake.IntakeState.OFF);
        break;
      case INTAKE:
        pivot.setDesiredState(Arm.PivotState.INTAKE);
        intake.setDesiredState(Intake.IntakeState.INTAKING);
        break;
      default:
        pivot.setDesiredState(Arm.PivotState.IDLE);
        intake.setDesiredState(Intake.IntakeState.OFF);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {
    // Enforce current state continuously (just like 2910)
    CommandScheduler.getInstance().schedule(arm.setAngle(targetAngle));
  }

  /** Optional: check if arm reached goal */
  public boolean isAtTarget() {
    return arm.atSetpoint(targetAngle);
  }

  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}
