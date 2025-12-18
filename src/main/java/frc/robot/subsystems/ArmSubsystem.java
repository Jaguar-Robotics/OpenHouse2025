// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.IntakeRollerID);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(270))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(157.5,1)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.5))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private TalonFX talon = new TalonFX(Constants.IntakeConstants.PivotMotorID);

  // Create our SmartMotorController from our Talon and config with the Kraken.
  private SmartMotorController talonSmartMotorController = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

  private ArmConfig armCfg = new ArmConfig(talonSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-100), Degrees.of(100))
  // Hard limit is applied to the simulation.
  .withHardLimit(Degrees.of(-100), Degrees.of(100))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(0))
  // Length and mass of your arm for sim.
  .withLength(Feet.of(3))
  .withMass(Pounds.of(1))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

    /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) { return arm.setAngle(angle);}

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}

  public double getAngle() {
    return Math.abs(arm.getAngle().in(Degrees));
  }

  public boolean atSetpoint(Angle target) {
    return Math.abs(arm.getAngle().in(Degrees) - target.in(Degrees)) < 1.0;
  }

  private static ArmSubsystem m_Instance;

  /** Stop the pivot by holding current angle */
  public void stop() {
    arm.setAngle(arm.getAngle()).schedule();
  }


  public static ArmSubsystem getInstance() {
    if (m_Instance == null) {
      m_Instance = new ArmSubsystem();
    }
    return m_Instance;
  }

  public void rollerSpeed(double dutycycle) {  
    rollerMotor.set(dutycycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}
