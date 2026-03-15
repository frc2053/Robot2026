// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.BuildConstants;
import frc.robot.util.FuelVisualizer;
import frc.robot.util.GameState;
import frc.robot.util.LogUtil;
import frc.robot.util.MechanismVisualizer;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final GameState m_gameState;

  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    LogUtil.recordMetadata("Java Vendor", System.getProperty("java.vendor"));
    LogUtil.recordMetadata("Java Version", System.getProperty("java.version"));
    LogUtil.recordMetadata("WPILib Version", WPILibVersion.Version);

    LogUtil.recordMetadata("Runtime Type", getRuntimeType().toString());

    // Git and build information
    LogUtil.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    LogUtil.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
    LogUtil.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    LogUtil.recordMetadata("Git Date", BuildConstants.GIT_DATE);
    LogUtil.recordMetadata("Git Revision", BuildConstants.GIT_REVISION);
    LogUtil.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        LogUtil.recordMetadata("Git Dirty", "All changes committed");
        break;
      case 1:
        LogUtil.recordMetadata("Git Dirty", "Uncommitted changes");
        break;
      default:
        LogUtil.recordMetadata("Git Dirty", "Unknown");
        break;
    }

    m_robotContainer = new RobotContainer();
    m_gameState = new GameState(() -> m_robotContainer.m_drivetrain.getState().Pose);
  }

  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    m_robotContainer.m_vision.periodic(new Pose3d(m_robotContainer.m_drivetrain.getState().Pose));
    m_gameState.periodic();
    FuelVisualizer.update();
    MechanismVisualizer.publish();

    gcStatsCollector.update();

    SmartDashboard.putNumber("RoboRIO/CPU Temperature", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RoboRIO/RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("RoboRIO/Input Current", RobotController.getInputCurrent());

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    double codeRuntime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
    SmartDashboard.putNumber("Code Runtime (ms)", codeRuntime);
  }

  @Override
  public void disabledInit() {
    m_gameState.onDisabled();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_gameState.startAuto();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
    m_gameState.startTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_gameState.startTest();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.m_vision.simulationPeriodic(m_robotContainer.m_drivetrain.getState().Pose);
  }

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      SmartDashboard.putNumber("GC Stats/GC Time MS", (double) accumTime);
      SmartDashboard.putNumber("GC Stats/GC Counts", (double) accumCounts);
    }
  }
}
