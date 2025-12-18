// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance = new IntakeSubsystem();
  public TalonFX IntakeMotor = new TalonFX(Constants.IntakeConstants.IntakeRollerID);

  /** Creates a new Intake. */
  public IntakeSubsystem() {}

  public static IntakeSubsystem getInstance(){
    if (instance == null){
      return new IntakeSubsystem();
    }else{
      return instance;
    }
  }

  public void setOutput(double val){
    IntakeMotor.set(val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
