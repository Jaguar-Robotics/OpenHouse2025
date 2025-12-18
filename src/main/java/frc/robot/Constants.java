// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class Constants {
  public static abstract class IntakeConstants {
    public static final int PivotMotorID = 20;
    public static final int IntakeRollerID = 19;
    public static final Angle STOW = Units.Degrees.of(0);
    public static final Angle INTAKE = Units.Degrees.of(-90);
    public static final double In = -6.0;
    public static final double Out = 8.0;
    public static final double Off = 0.0;
  }
}
