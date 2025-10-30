// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import  frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

  private static IntakePivot instance = new IntakePivot();
  public TalonFX pivotMotor = new TalonFX(Constants.IntakePivotConstants.PivotMotorID);
  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  private TrapezoidProfile m_Profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private PositionDutyCycle m_request = new PositionDutyCycle(0).withSlot(0);

  /** Creates a new IntakePivot. */
  public IntakePivot() {

    pivotMotor.setControl(m_request);

    m_Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.01, 0.00001));

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = 0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    pivotMotor.getConfigurator().apply(slot0Configs);
  }

  public static IntakePivot getInstance(){
    if (instance == null){
      return new IntakePivot();
    }else{
      return instance;
    }
  }

  public void setSetpoint(double set) {
    m_goal.position = (17.5*9)*set;
  }

  public double getPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint() {
    if(Math.abs(pivotMotor.getPosition().getValueAsDouble() - (m_goal.position)) < .1){
      return true;
    }else{
      return false;
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_request.Position = (17.5*9)*setpoint;
    //pivotMotor.setControl(m_request);
    if(m_Profile != null){
      m_setpoint = m_Profile.calculate(0.020, m_setpoint, m_goal);
      m_request.Position = m_setpoint.position;
      m_request.Velocity = m_setpoint.velocity;
      }
      pivotMotor.setControl(m_request);
  }
}
