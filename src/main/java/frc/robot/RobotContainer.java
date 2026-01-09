// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final double m_maxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double m_maxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1)
          .withRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry m_logger = new Telemetry(m_maxSpeed);

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_autoChooser = new SendableChooser<>();
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(
                        -m_joystick.getLeftY()
                            * m_maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_joystick.getLeftX() * m_maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_joystick.getRightX()
                            * m_maxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_joystick.back().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_joystick.back().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_joystick
        .start()
        .and(m_joystick.y())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_joystick
        .start()
        .and(m_joystick.x())
        .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_joystick.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return m_autoChooser.getSelected();
  }
}
