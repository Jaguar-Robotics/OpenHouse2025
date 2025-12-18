// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Intake;
import frc.robot.commands.L1;
import frc.robot.commands.Outtake;
import frc.robot.commands.PivotDown;
import frc.robot.commands.PivotUp;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.Superstructure;
import frc.robot.handlers.Superstructure.SuperstructureState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {
    
    /* 
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    */

    private double MaxSpeed = 2;
    private double MaxAngularRate = 5;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

    private final Superstructure superstructure = Superstructure.getInstance();

    public RobotContainer() {
        configureBindings();

    // Set the default command to force the arm to go to 0.
    m_armSubsystem.setDefaultCommand(m_armSubsystem.setAngle(Degrees.of(0)));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //TESTING CONTROLS FOR YAMS
        /* 
        joystick.a().onTrue(Commands.run(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE)));
        joystick.b().onTrue(Commands.run(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.STOW)));


        // Schedule `setAngle` when the Xbox controller's B button is pressed,
        // cancelling on release.
        joystick.x().whileTrue(m_armSubsystem.setAngle(Degrees.of(-5)));
        joystick.y().whileTrue(m_armSubsystem.setAngle(Degrees.of(15)));
        */
        /*
        // Schedule `set` when the Xbox controller's B button is pressed,
        // cancelling on release.
        joystick.x().whileTrue(m_armSubsystem.set(0.3));
        joystick.y().whileTrue(m_armSubsystem.set(-0.3));*/
        

        //WORKING CONTROLS FOR OPEN HOUSE
        
        joystick.leftTrigger().whileTrue(new PivotDown());
        //joystick.leftTrigger().whileTrue(new Outtake());
        //joystick.leftTrigger().whileFalse(new PivotUp());

        /* joystick.rightTrigger().whileTrue(
            new SequentialCommandGroup(
                new L1(),
                new Intake()
            )
        );*/
        //joystick.rightTrigger().whileTrue(new L1());
        //joystick.rightTrigger().whileFalse(new PivotUp());
        //joystick.rightTrigger().whileFalse(
           // new SequentialCommandGroup(
               // new Intake()
                //new PivotUp()
            //)
        //);

        //joystick.b().whileTrue(new PivotDown());
        //joystick.b().whileFalse(new PivotUp());

        //joystick.a().whileTrue(new Intake());


        

        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
