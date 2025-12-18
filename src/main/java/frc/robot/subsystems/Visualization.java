package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
import yams.mechanisms.positional.Arm;
//import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.Logger;

public class Visualization extends SubsystemBase {
  private final ArmSubsystem elevator;

  private static final double IN_TO_M = 0.0254;

  private static final double CARRIAGE_TRAVEL_M = 23.0 * IN_TO_M; // stage 3 hardstop dist
  private static final double STAGE_TRAVEL_M = 28.5 * IN_TO_M; // stage 2 and 1 hardstop dist

  public Visualization(ArmSubsystem elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("3D/RobotZeroed", new Pose2d());

    // zeroed component poses
    // [stage1, stage2, carriage, wrist]
    Logger.recordOutput(
        "3D/ComponentPosesZeroed",
        new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});

    // real component poses
    Pose3d elevatorPose = new Pose3d();
    Pose3d intakePose = new Pose3d();
    Pose3d stage2Pose = new Pose3d();
    Pose3d stage3Pose = new Pose3d();
    if (elevator != null) {
      // elevator offset
      final double baseX = 0;
      final double baseY = 0;
      final double baseZ = 0;
      double armA = elevator.getAngle();
      
      intakePose = new Pose3d(new Translation3d(baseX, baseY, baseZ + armA), new Rotation3d());

      // Zeroed and real arrays for moving stages
      Logger.recordOutput(
          "3D/IntakeZeroed", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
      Logger.recordOutput("3D/Intake", new Pose3d[] {intakePose});
    }


    // [stage1, stage2, carriage, wrist]
    Logger.recordOutput(
        "3D/ComponentPoses", new Pose3d[] {intakePose});
  }

  private static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }
}