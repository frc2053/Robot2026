// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter;

/**
 * Minimal robot program for the shooter test bed. Only initializes the Shooter and Vision
 * subsystems, avoiding CAN errors from missing drivetrain/intake/kicker/spindexer motors.
 *
 * <p>To switch to this mode, change Main.java to: RobotBase.startRobot(TestBedRobot::new)
 *
 * <p>Joystick bindings:
 *
 * <ul>
 *   <li>Tuning mode: enable via Shooter/Tuning/Enabled in NetworkTables, set RPMs there.
 *   <li>Right Bumper + Y: main shooter SysId quasistatic forward
 *   <li>Right Bumper + B: main shooter SysId quasistatic reverse
 *   <li>Right Bumper + X: main shooter SysId dynamic forward
 *   <li>Right Bumper + A: main shooter SysId dynamic reverse
 *   <li>Left Bumper + Y: roller SysId quasistatic forward
 *   <li>Left Bumper + B: roller SysId quasistatic reverse
 *   <li>Left Bumper + X: roller SysId dynamic forward
 *   <li>Left Bumper + A: roller SysId dynamic reverse
 * </ul>
 */
public class TestBedRobot extends TimedRobot {

  private final Shooter m_shooter = new Shooter();

  // No-op consumer — no drivetrain to push pose estimates into.
  private final Vision m_vision = new Vision((pose, timestamp, stdDevs) -> {});

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  /** Creates a new TestBedRobot. */
  public TestBedRobot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    SignalLogger.start();

    configureBindings();
  }

  private void configureBindings() {
    // PID tuning mode — controlled via NetworkTables Shooter/Tuning/Enabled entry.
    // Set MainShooterRPM and RollerRPM there, along with gain entries under
    // Shooter/Tuning/MainShooterGains/* and Shooter/Tuning/RollerGains/*.
    m_shooter.tuningEnabledTrigger().whileTrue(m_shooter.tuningCommand());

    // Main shooter SysId — hold right bumper, then press face button.
    m_joystick
        .rightBumper()
        .and(m_joystick.y())
        .whileTrue(m_shooter.mainShooterSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_joystick
        .rightBumper()
        .and(m_joystick.b())
        .whileTrue(m_shooter.mainShooterSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_joystick
        .rightBumper()
        .and(m_joystick.x())
        .whileTrue(m_shooter.mainShooterSysIdDynamic(SysIdRoutine.Direction.kForward));
    m_joystick
        .rightBumper()
        .and(m_joystick.a())
        .whileTrue(m_shooter.mainShooterSysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Roller SysId — hold left bumper, then press face button.
    m_joystick
        .leftBumper()
        .and(m_joystick.y())
        .whileTrue(m_shooter.rollerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_joystick
        .leftBumper()
        .and(m_joystick.b())
        .whileTrue(m_shooter.rollerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_joystick
        .leftBumper()
        .and(m_joystick.x())
        .whileTrue(m_shooter.rollerSysIdDynamic(SysIdRoutine.Direction.kForward));
    m_joystick
        .leftBumper()
        .and(m_joystick.a())
        .whileTrue(m_shooter.rollerSysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // Pass a static pose — the test bed does not move.
    m_vision.periodic(new Pose3d());
  }

  @Override
  public void disabledInit() {
    // Stop the signal logger when disabled so SysId .hoot files are flushed and closed.
    SignalLogger.stop();
  }

  @Override
  public void teleopInit() {
    // Restart signal logger when enabled in case it was stopped on disable.
    SignalLogger.start();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    SignalLogger.start();
  }
}
