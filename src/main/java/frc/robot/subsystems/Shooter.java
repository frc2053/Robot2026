// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShootingOnTheFly;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

  private final TalonFX m_shooterMotorLeft;
  private final TalonFX m_shooterMotorRight;
  private final TalonFX m_shooterMotorTopRoller;

  private final StatusSignal<AngularVelocity> m_leftMotorVel;
  private final StatusSignal<AngularVelocity> m_rightMotorVel;
  private final StatusSignal<AngularVelocity> m_rollerVel;

  private final StatusSignal<Angle> m_leftMotorPos;
  private final StatusSignal<Angle> m_rightMotorPos;
  private final StatusSignal<Angle> m_rollerPos;

  private final StatusSignal<Voltage> m_leftMotorVoltage;
  private final StatusSignal<Voltage> m_rightMotorVoltage;
  private final StatusSignal<Voltage> m_rollerVoltage;

  private final StatusSignal<Current> m_leftMotorStatorCurrent;
  private final StatusSignal<Current> m_rightMotorStatorCurrent;
  private final StatusSignal<Current> m_rollerStatorCurrent;
  private final StatusSignal<Current> m_leftMotorSupplyCurrent;
  private final StatusSignal<Current> m_rightMotorSupplyCurrent;
  private final StatusSignal<Current> m_rollerSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_leftMotorSimState;
  private final TalonFXSimState m_rightMotorSimState;
  private final TalonFXSimState m_rollerSimState;
  private final DCMotorSim m_mainShooterSim;
  private final DCMotorSim m_rollerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_leftMotorVelPub;
  private final DoublePublisher m_rightMotorVelPub;
  private final DoublePublisher m_rollerVelPub;
  private final DoublePublisher m_leftMotorVoltagePub;
  private final DoublePublisher m_rightMotorVoltagePub;
  private final DoublePublisher m_rollerVoltagePub;
  private final DoublePublisher m_leftMotorStatorCurrentPub;
  private final DoublePublisher m_rightMotorStatorCurrentPub;
  private final DoublePublisher m_rollerStatorCurrentPub;
  private final DoublePublisher m_leftMotorSupplyCurrentPub;
  private final DoublePublisher m_rightMotorSupplyCurrentPub;
  private final DoublePublisher m_rollerSupplyCurrentPub;
  private final StringPublisher m_currentCommandPub;
  private final BooleanPublisher m_atSpeedPub;
  private final DoublePublisher m_mainShooterSetpointPub;
  private final DoublePublisher m_rollerSetpointPub;

  // SOTF debug publishers
  private final StructPublisher<Pose2d> m_sotfVirtualTargetPub;
  private final StructPublisher<Pose2d> m_sotfGoalPub;
  private final DoublePublisher m_sotfStaticDistancePub;
  private final DoublePublisher m_sotfVirtualDistancePub;
  private final DoublePublisher m_sotfTimeOfFlightPub;
  private final IntegerPublisher m_sotfIterationsPub;
  private final BooleanPublisher m_sotfConvergedPub;
  private final DoublePublisher m_sotfContractionFactorPub;
  private final DoublePublisher m_sotfFirstOrderMissPub;
  private final DoublePublisher m_sotfRobotSpeedPub;
  private final DoublePublisher m_sotfAimingAngleDegPub;

  // Target velocities for at-speed detection
  private double m_targetMainShooterRps;
  private double m_targetRollerRps;

  // Public trigger for when shooter is at speed
  private final Trigger m_atSpeedTrigger;

  // Control requests
  private final VelocityTorqueCurrentFOC m_mainShooterVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VelocityTorqueCurrentFOC m_rollerVelocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Tuning mode NetworkTables controls
  private final BooleanSubscriber m_tuningEnabledSub;
  private final BooleanPublisher m_tuningEnabledPub;
  private final Trigger m_tuningEnabledTrigger;
  private final DoubleSubscriber m_tuningMainShooterRpsSub;
  private final DoublePublisher m_tuningMainShooterRpsPub;
  private final DoubleSubscriber m_tuningRollerRpsSub;
  private final DoublePublisher m_tuningRollerRpsPub;
  private final DoubleSubscriber m_velocityToleranceRpsSub;
  private final DoublePublisher m_velocityToleranceRpsPub;

  // Tunable gains for main shooter
  private final DoubleSubscriber m_mainShooterKSSub;
  private final DoubleSubscriber m_mainShooterKVSub;
  private final DoubleSubscriber m_mainShooterKASub;
  private final DoubleSubscriber m_mainShooterKPSub;
  private final DoubleSubscriber m_mainShooterKISub;
  private final DoubleSubscriber m_mainShooterKDSub;
  private final DoublePublisher m_mainShooterKSPub;
  private final DoublePublisher m_mainShooterKVPub;
  private final DoublePublisher m_mainShooterKAPub;
  private final DoublePublisher m_mainShooterKPPub;
  private final DoublePublisher m_mainShooterKIPub;
  private final DoublePublisher m_mainShooterKDPub;

  // Tunable gains for roller
  private final DoubleSubscriber m_rollerKSSub;
  private final DoubleSubscriber m_rollerKVSub;
  private final DoubleSubscriber m_rollerKASub;
  private final DoubleSubscriber m_rollerKPSub;
  private final DoubleSubscriber m_rollerKISub;
  private final DoubleSubscriber m_rollerKDSub;
  private final DoublePublisher m_rollerKSPub;
  private final DoublePublisher m_rollerKVPub;
  private final DoublePublisher m_rollerKAPub;
  private final DoublePublisher m_rollerKPPub;
  private final DoublePublisher m_rollerKIPub;
  private final DoublePublisher m_rollerKDPub;

  // Track last known gain values to detect changes
  private double m_lastMainShooterKS;
  private double m_lastMainShooterKV;
  private double m_lastMainShooterKA;
  private double m_lastMainShooterKP;
  private double m_lastMainShooterKI;
  private double m_lastMainShooterKD;
  private double m_lastRollerKS;
  private double m_lastRollerKV;
  private double m_lastRollerKA;
  private double m_lastRollerKP;
  private double m_lastRollerKI;
  private double m_lastRollerKD;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
    m_shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
    m_shooterMotorTopRoller = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ROLLER_ID);

    TalonFXConfiguration mainShooterConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ShooterConstants.SHOOTER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.SHOOTER_SUPPLY_LIMIT))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(ShooterConstants.SHOOTER_STATOR_LIMIT)
                    .withPeakReverseTorqueCurrent(0))
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kMainShooterKS)
                    .withKV(ShooterConstants.kMainShooterKV)
                    .withKA(ShooterConstants.kMainShooterKA)
                    .withKP(ShooterConstants.kMainShooterKP)
                    .withKI(ShooterConstants.kMainShooterKI)
                    .withKD(ShooterConstants.kMainShooterKD));

    // Roller config with different PID gains and inverted direction (clockwise positive)
    TalonFXConfiguration rollerConfig =
        mainShooterConfig
            .clone()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ShooterConstants.SHOOTER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.SHOOTER_SUPPLY_LIMIT))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(ShooterConstants.SHOOTER_STATOR_LIMIT)
                    .withPeakReverseTorqueCurrent(0))
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kRollerKS)
                    .withKV(ShooterConstants.kRollerKV)
                    .withKA(ShooterConstants.kRollerKA)
                    .withKP(ShooterConstants.kRollerKP)
                    .withKI(ShooterConstants.kRollerKI)
                    .withKD(ShooterConstants.kRollerKD));

    StatusCode shooterLeftConfigResult =
        m_shooterMotorLeft.getConfigurator().apply(mainShooterConfig);
    if (!shooterLeftConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to left shooter motor!");
    }
    StatusCode shooterRightConfigResult =
        m_shooterMotorRight.getConfigurator().apply(mainShooterConfig);
    if (!shooterRightConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to right shooter motor!");
    }
    StatusCode rollerConfigResult = m_shooterMotorTopRoller.getConfigurator().apply(rollerConfig);
    if (!rollerConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to roller shooter motor!");
    }

    m_leftMotorVel = m_shooterMotorLeft.getVelocity();
    m_rightMotorVel = m_shooterMotorRight.getVelocity();
    m_rollerVel = m_shooterMotorTopRoller.getVelocity();

    m_leftMotorPos = m_shooterMotorLeft.getPosition();
    m_rightMotorPos = m_shooterMotorRight.getPosition();
    m_rollerPos = m_shooterMotorTopRoller.getPosition();

    m_leftMotorVoltage = m_shooterMotorLeft.getMotorVoltage();
    m_rightMotorVoltage = m_shooterMotorRight.getMotorVoltage();
    m_rollerVoltage = m_shooterMotorTopRoller.getMotorVoltage();

    m_leftMotorStatorCurrent = m_shooterMotorLeft.getStatorCurrent();
    m_rightMotorStatorCurrent = m_shooterMotorRight.getStatorCurrent();
    m_rollerStatorCurrent = m_shooterMotorTopRoller.getStatorCurrent();
    m_leftMotorSupplyCurrent = m_shooterMotorLeft.getSupplyCurrent();
    m_rightMotorSupplyCurrent = m_shooterMotorRight.getSupplyCurrent();
    m_rollerSupplyCurrent = m_shooterMotorTopRoller.getSupplyCurrent();

    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_leftMotorVel,
            m_rightMotorVel,
            m_rollerVel,
            m_leftMotorPos,
            m_rightMotorPos,
            m_rollerPos,
            m_leftMotorVoltage,
            m_rightMotorVoltage,
            m_rollerVoltage,
            m_leftMotorStatorCurrent,
            m_rightMotorStatorCurrent,
            m_rollerStatorCurrent,
            m_leftMotorSupplyCurrent,
            m_rightMotorSupplyCurrent,
            m_rollerSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for shooter subsystem!");
    }
    StatusCode optiResult =
        ParentDevice.optimizeBusUtilizationForAll(
            m_shooterMotorLeft, m_shooterMotorRight, m_shooterMotorTopRoller);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for shooter subsystem!");
    }

    // Set left motor to follow right motor (they are mechanically coupled)
    // MotorAlignmentValue.Aligned because they spin in the same direction
    m_shooterMotorLeft.setControl(
        new Follower(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, MotorAlignmentValue.Aligned));

    // Initialize NetworkTables publishers for logging
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    m_leftMotorVelPub = shooterTable.getDoubleTopic("LeftMotorVelocityRPM").publish();
    m_rightMotorVelPub = shooterTable.getDoubleTopic("RightMotorVelocityRPM").publish();
    m_rollerVelPub = shooterTable.getDoubleTopic("RollerVelocityRPM").publish();
    m_leftMotorVoltagePub = shooterTable.getDoubleTopic("LeftMotorVoltage").publish();
    m_rightMotorVoltagePub = shooterTable.getDoubleTopic("RightMotorVoltage").publish();
    m_rollerVoltagePub = shooterTable.getDoubleTopic("RollerVoltage").publish();
    m_leftMotorStatorCurrentPub = shooterTable.getDoubleTopic("LeftMotorStatorCurrent").publish();
    m_rightMotorStatorCurrentPub = shooterTable.getDoubleTopic("RightMotorStatorCurrent").publish();
    m_rollerStatorCurrentPub = shooterTable.getDoubleTopic("RollerStatorCurrent").publish();
    m_leftMotorSupplyCurrentPub = shooterTable.getDoubleTopic("LeftMotorSupplyCurrent").publish();
    m_rightMotorSupplyCurrentPub = shooterTable.getDoubleTopic("RightMotorSupplyCurrent").publish();
    m_rollerSupplyCurrentPub = shooterTable.getDoubleTopic("RollerSupplyCurrent").publish();
    m_currentCommandPub = shooterTable.getStringTopic("CurrentCommand").publish();
    m_atSpeedPub = shooterTable.getBooleanTopic("AtSpeed").publish();
    m_mainShooterSetpointPub = shooterTable.getDoubleTopic("MainShooterSetpointRPM").publish();
    m_rollerSetpointPub = shooterTable.getDoubleTopic("RollerSetpointRPM").publish();

    // Initialize at-speed trigger
    m_atSpeedTrigger = new Trigger(this::isAtSpeed);

    // Initialize simulation
    m_leftMotorSimState = m_shooterMotorLeft.getSimState();
    m_rightMotorSimState = m_shooterMotorRight.getSimState();
    m_rollerSimState = m_shooterMotorTopRoller.getSimState();

    // Set motor types for accurate simulation
    m_leftMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rightMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rollerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Main shooter flywheel sim (powered by left and right motors)
    // Using 2 Kraken X60 motors
    m_mainShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2),
                ShooterConstants.MAIN_SHOOTER_MOI,
                ShooterConstants.MAIN_SHOOTER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    // Roller flywheel sim (powered by single motor)
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500Foc(1),
                ShooterConstants.ROLLER_MOI,
                ShooterConstants.ROLLER_GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));

    // Initialize SOTF debug publishers
    NetworkTable sotfTable = shooterTable.getSubTable("SOTF");
    m_sotfVirtualTargetPub = sotfTable.getStructTopic("VirtualTarget", Pose2d.struct).publish();
    m_sotfGoalPub = sotfTable.getStructTopic("Goal", Pose2d.struct).publish();
    m_sotfStaticDistancePub = sotfTable.getDoubleTopic("StaticDistanceM").publish();
    m_sotfVirtualDistancePub = sotfTable.getDoubleTopic("VirtualDistanceM").publish();
    m_sotfTimeOfFlightPub = sotfTable.getDoubleTopic("TimeOfFlightS").publish();
    m_sotfIterationsPub = sotfTable.getIntegerTopic("Iterations").publish();
    m_sotfConvergedPub = sotfTable.getBooleanTopic("Converged").publish();
    m_sotfContractionFactorPub = sotfTable.getDoubleTopic("ContractionFactor").publish();
    m_sotfFirstOrderMissPub = sotfTable.getDoubleTopic("FirstOrderMissM").publish();
    m_sotfRobotSpeedPub = sotfTable.getDoubleTopic("RobotSpeedMps").publish();
    m_sotfAimingAngleDegPub = sotfTable.getDoubleTopic("AimingAngleDeg").publish();

    // Initialize tuning mode NetworkTables controls
    NetworkTable tuningTable = shooterTable.getSubTable("Tuning");
    m_tuningEnabledPub = tuningTable.getBooleanTopic("Enabled").publish();
    m_tuningEnabledSub = tuningTable.getBooleanTopic("Enabled").subscribe(false);
    m_tuningEnabledPub.set(false);
    m_tuningEnabledTrigger = new Trigger(m_tuningEnabledSub::get);
    m_tuningMainShooterRpsPub = tuningTable.getDoubleTopic("MainShooterRPM").publish();
    m_tuningMainShooterRpsSub = tuningTable.getDoubleTopic("MainShooterRPM").subscribe(0.0);
    m_tuningMainShooterRpsPub.set(0.0);
    m_tuningRollerRpsPub = tuningTable.getDoubleTopic("RollerRPM").publish();
    m_tuningRollerRpsSub = tuningTable.getDoubleTopic("RollerRPM").subscribe(0.0);
    m_tuningRollerRpsPub.set(0.0);
    m_velocityToleranceRpsPub = tuningTable.getDoubleTopic("VelocityToleranceRps").publish();
    m_velocityToleranceRpsSub =
        tuningTable
            .getDoubleTopic("VelocityToleranceRps")
            .subscribe(ShooterConstants.kVelocityToleranceRps);
    m_velocityToleranceRpsPub.set(ShooterConstants.kVelocityToleranceRps);

    // Initialize tunable gains for main shooter
    NetworkTable mainShooterGains = tuningTable.getSubTable("MainShooterGains");

    m_mainShooterKSPub = mainShooterGains.getDoubleTopic("kS").publish();
    m_mainShooterKSSub =
        mainShooterGains.getDoubleTopic("kS").subscribe(ShooterConstants.kMainShooterKS);
    m_mainShooterKSPub.set(ShooterConstants.kMainShooterKS);
    m_lastMainShooterKS = ShooterConstants.kMainShooterKS;

    m_mainShooterKVPub = mainShooterGains.getDoubleTopic("kV").publish();
    m_mainShooterKVSub =
        mainShooterGains.getDoubleTopic("kV").subscribe(ShooterConstants.kMainShooterKV);
    m_mainShooterKVPub.set(ShooterConstants.kMainShooterKV);
    m_lastMainShooterKV = ShooterConstants.kMainShooterKV;

    m_mainShooterKAPub = mainShooterGains.getDoubleTopic("kA").publish();
    m_mainShooterKASub =
        mainShooterGains.getDoubleTopic("kA").subscribe(ShooterConstants.kMainShooterKA);
    m_mainShooterKAPub.set(ShooterConstants.kMainShooterKA);
    m_lastMainShooterKA = ShooterConstants.kMainShooterKA;

    m_mainShooterKPPub = mainShooterGains.getDoubleTopic("kP").publish();
    m_mainShooterKPSub =
        mainShooterGains.getDoubleTopic("kP").subscribe(ShooterConstants.kMainShooterKP);
    m_mainShooterKPPub.set(ShooterConstants.kMainShooterKP);
    m_lastMainShooterKP = ShooterConstants.kMainShooterKP;

    m_mainShooterKIPub = mainShooterGains.getDoubleTopic("kI").publish();
    m_mainShooterKISub =
        mainShooterGains.getDoubleTopic("kI").subscribe(ShooterConstants.kMainShooterKI);
    m_mainShooterKIPub.set(ShooterConstants.kMainShooterKI);
    m_lastMainShooterKI = ShooterConstants.kMainShooterKI;

    m_mainShooterKDPub = mainShooterGains.getDoubleTopic("kD").publish();
    m_mainShooterKDSub =
        mainShooterGains.getDoubleTopic("kD").subscribe(ShooterConstants.kMainShooterKD);
    m_mainShooterKDPub.set(ShooterConstants.kMainShooterKD);
    m_lastMainShooterKD = ShooterConstants.kMainShooterKD;

    // Initialize tunable gains for roller
    NetworkTable rollerGains = tuningTable.getSubTable("RollerGains");

    m_rollerKSPub = rollerGains.getDoubleTopic("kS").publish();
    m_rollerKSSub = rollerGains.getDoubleTopic("kS").subscribe(ShooterConstants.kRollerKS);
    m_rollerKSPub.set(ShooterConstants.kRollerKS);
    m_lastRollerKS = ShooterConstants.kRollerKS;

    m_rollerKVPub = rollerGains.getDoubleTopic("kV").publish();
    m_rollerKVSub = rollerGains.getDoubleTopic("kV").subscribe(ShooterConstants.kRollerKV);
    m_rollerKVPub.set(ShooterConstants.kRollerKV);
    m_lastRollerKV = ShooterConstants.kRollerKV;

    m_rollerKAPub = rollerGains.getDoubleTopic("kA").publish();
    m_rollerKASub = rollerGains.getDoubleTopic("kA").subscribe(ShooterConstants.kRollerKA);
    m_rollerKAPub.set(ShooterConstants.kRollerKA);
    m_lastRollerKA = ShooterConstants.kRollerKA;

    m_rollerKPPub = rollerGains.getDoubleTopic("kP").publish();
    m_rollerKPSub = rollerGains.getDoubleTopic("kP").subscribe(ShooterConstants.kRollerKP);
    m_rollerKPPub.set(ShooterConstants.kRollerKP);
    m_lastRollerKP = ShooterConstants.kRollerKP;

    m_rollerKIPub = rollerGains.getDoubleTopic("kI").publish();
    m_rollerKISub = rollerGains.getDoubleTopic("kI").subscribe(ShooterConstants.kRollerKI);
    m_rollerKIPub.set(ShooterConstants.kRollerKI);
    m_lastRollerKI = ShooterConstants.kRollerKI;

    m_rollerKDPub = rollerGains.getDoubleTopic("kD").publish();
    m_rollerKDSub = rollerGains.getDoubleTopic("kD").subscribe(ShooterConstants.kRollerKD);
    m_rollerKDPub.set(ShooterConstants.kRollerKD);
    m_lastRollerKD = ShooterConstants.kRollerKD;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_leftMotorVel,
        m_rightMotorVel,
        m_rollerVel,
        m_leftMotorVoltage,
        m_rightMotorVoltage,
        m_rollerVoltage,
        m_leftMotorStatorCurrent,
        m_rightMotorStatorCurrent,
        m_rollerStatorCurrent,
        m_leftMotorSupplyCurrent,
        m_rightMotorSupplyCurrent,
        m_rollerSupplyCurrent);

    // Publish velocity signals to NetworkTables in RPM
    m_leftMotorVelPub.set(m_leftMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rightMotorVelPub.set(m_rightMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerVelPub.set(m_rollerVel.getValue().in(RotationsPerSecond) * 60.0);

    // Publish voltage signals to NetworkTables
    m_leftMotorVoltagePub.set(m_leftMotorVoltage.getValue().in(Volts));
    m_rightMotorVoltagePub.set(m_rightMotorVoltage.getValue().in(Volts));
    m_rollerVoltagePub.set(m_rollerVoltage.getValue().in(Volts));

    // Publish current signals to NetworkTables
    m_leftMotorStatorCurrentPub.set(m_leftMotorStatorCurrent.getValue().in(Amps));
    m_rightMotorStatorCurrentPub.set(m_rightMotorStatorCurrent.getValue().in(Amps));
    m_rollerStatorCurrentPub.set(m_rollerStatorCurrent.getValue().in(Amps));
    m_leftMotorSupplyCurrentPub.set(m_leftMotorSupplyCurrent.getValue().in(Amps));
    m_rightMotorSupplyCurrentPub.set(m_rightMotorSupplyCurrent.getValue().in(Amps));
    m_rollerSupplyCurrentPub.set(m_rollerSupplyCurrent.getValue().in(Amps));

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Publish at-speed status
    m_atSpeedPub.set(isAtSpeed());

    // Publish velocity setpoints in RPM
    m_mainShooterSetpointPub.set(m_targetMainShooterRps * 60.0);
    m_rollerSetpointPub.set(m_targetRollerRps * 60.0);

    // Check for tuning updates and apply if changed
    updateTunableGains();
  }

  /** Checks for tunable gain updates from NetworkTables and applies them to motors. */
  private void updateTunableGains() {
    // Read current values from NetworkTables
    double mainKS = m_mainShooterKSSub.get();
    double mainKV = m_mainShooterKVSub.get();
    double mainKA = m_mainShooterKASub.get();
    double mainKP = m_mainShooterKPSub.get();
    double mainKI = m_mainShooterKISub.get();
    double mainKD = m_mainShooterKDSub.get();

    // Check if main shooter gains have changed
    boolean mainGainsChanged =
        mainKS != m_lastMainShooterKS
            || mainKV != m_lastMainShooterKV
            || mainKA != m_lastMainShooterKA
            || mainKP != m_lastMainShooterKP
            || mainKI != m_lastMainShooterKI
            || mainKD != m_lastMainShooterKD;

    if (mainGainsChanged) {
      Slot0Configs mainSlot0 =
          new Slot0Configs()
              .withKS(mainKS)
              .withKV(mainKV)
              .withKA(mainKA)
              .withKP(mainKP)
              .withKI(mainKI)
              .withKD(mainKD);

      m_shooterMotorLeft.getConfigurator().apply(mainSlot0);
      m_shooterMotorRight.getConfigurator().apply(mainSlot0);

      // Update last known values
      m_lastMainShooterKS = mainKS;
      m_lastMainShooterKV = mainKV;
      m_lastMainShooterKA = mainKA;
      m_lastMainShooterKP = mainKP;
      m_lastMainShooterKI = mainKI;
      m_lastMainShooterKD = mainKD;
    }

    // Read roller values from NetworkTables
    double rollerKS = m_rollerKSSub.get();
    double rollerKV = m_rollerKVSub.get();
    double rollerKA = m_rollerKASub.get();
    double rollerKP = m_rollerKPSub.get();
    double rollerKI = m_rollerKISub.get();
    double rollerKD = m_rollerKDSub.get();

    // Check if roller gains have changed
    boolean rollerGainsChanged =
        rollerKS != m_lastRollerKS
            || rollerKV != m_lastRollerKV
            || rollerKA != m_lastRollerKA
            || rollerKP != m_lastRollerKP
            || rollerKI != m_lastRollerKI
            || rollerKD != m_lastRollerKD;

    if (rollerGainsChanged) {
      Slot0Configs rollerSlot0 =
          new Slot0Configs()
              .withKS(rollerKS)
              .withKV(rollerKV)
              .withKA(rollerKA)
              .withKP(rollerKP)
              .withKI(rollerKI)
              .withKD(rollerKD);

      m_shooterMotorTopRoller.getConfigurator().apply(rollerSlot0);

      // Update last known values
      m_lastRollerKS = rollerKS;
      m_lastRollerKV = rollerKV;
      m_lastRollerKA = rollerKA;
      m_lastRollerKP = rollerKP;
      m_lastRollerKI = rollerKI;
      m_lastRollerKD = rollerKD;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_leftMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rightMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Main shooter flywheel: average voltage from left and right motors
    double mainShooterVoltage =
        (m_leftMotorSimState.getMotorVoltageMeasure().in(Volts)
                + m_rightMotorSimState.getMotorVoltageMeasure().in(Volts))
            / 2.0;
    m_mainShooterSim.setInputVoltage(mainShooterVoltage);
    m_mainShooterSim.update(0.020);

    // Update left and right motor sim states with rotor position and velocity
    // Note: DCMotorSim returns mechanism values, multiply by gear ratio for rotor values
    m_leftMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_leftMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_rightMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_rightMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));

    // Roller flywheel
    m_rollerSim.setInputVoltage(m_rollerSimState.getMotorVoltageMeasure().in(Volts));
    m_rollerSim.update(0.020);

    // Update roller motor sim state with rotor position and velocity
    m_rollerSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(ShooterConstants.ROLLER_GEAR_RATIO));
    m_rollerSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(ShooterConstants.ROLLER_GEAR_RATIO));
  }

  /**
   * Returns whether the shooter is at the target speed.
   *
   * @return true if both main shooter and roller are within tolerance of target speed
   */
  public boolean isAtSpeed() {
    double mainShooterVel = m_rightMotorVel.getValue().in(RotationsPerSecond);
    double rollerVel = m_rollerVel.getValue().in(RotationsPerSecond);
    double tolerance = m_velocityToleranceRpsSub.get();
    boolean mainAtSpeed = Math.abs(mainShooterVel - m_targetMainShooterRps) < tolerance;
    boolean rollerAtSpeed = Math.abs(rollerVel - m_targetRollerRps) < tolerance;
    return mainAtSpeed && rollerAtSpeed;
  }

  /**
   * Returns a trigger that is true when the shooter is at the target speed.
   *
   * @return the at-speed trigger
   */
  public Trigger atSpeedTrigger() {
    return m_atSpeedTrigger;
  }

  /**
   * Creates a command that spins up the shooter wheels based on distance to goal. Uses
   * interpolating tree maps to determine the appropriate speeds for the bottom shooter and top
   * roller based on the supplied distance.
   *
   * @param distanceSupplier A supplier that provides the distance to goal in meters
   * @return A command that runs the shooter at the interpolated speeds
   */
  public Command spinUpForDistanceCommand(DoubleSupplier distanceSupplier) {
    return this.run(
            () -> {
              double distance = distanceSupplier.getAsDouble();

              // Look up speeds from interpolating tree maps
              double bottomSpeedRps =
                  ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(distance) / 60.0;
              double topRollerSpeedRps = ShooterConstants.TOP_ROLLER_SPEED_MAP.get(distance) / 60.0;

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (right motor is leader, left follows)
              m_shooterMotorRight.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorTopRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForDistance");
  }

  public Command idleVoltage(double idleVoltage) {
    return this.run(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;

              // Set bottom shooter motor (leader)
              m_shooterMotorRight.setControl(m_voltageRequest.withOutput(idleVoltage));

              // Set top roller motor
              m_shooterMotorTopRoller.setControl(m_voltageRequest.withOutput(idleVoltage));
            })
        .withName("IdleShooterLowerVoltage");
  }

  /**
   * Creates a command that spins up the shooter using iterative TOF recursion for Shooting On The
   * Fly (SOTF). This compensates for robot velocity by iteratively computing a virtual target that
   * accounts for the projectile's time-of-flight.
   *
   * @param robotPoseSupplier Supplier for current robot pose.
   * @param robotSpeedsSupplier Supplier for current robot velocity (field-relative).
   * @param goalPositionSupplier Supplier for the goal position to shoot at.
   * @return A command that runs the shooter at SOTF-adjusted speeds.
   */
  public Command spinUpForSOTFCommand(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotSpeedsSupplier,
      Supplier<Translation2d> goalPositionSupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = robotPoseSupplier.get();
              ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();
              Translation2d goalPosition = goalPositionSupplier.get();

              // Calculate SOTF via iterative TOF recursion
              ShootingOnTheFly.SOTFResult result =
                  ShootingOnTheFly.calculate(
                      robotPose,
                      robotSpeeds,
                      goalPosition,
                      ShooterConstants.kSOTFLatencyCompensation,
                      ShooterConstants.TIME_OF_FLIGHT_MAP,
                      ShooterConstants.kVelocityUncertainty);

              // Publish SOTF debug data to NetworkTables
              Translation2d virtualTarget = result.virtualTarget();
              m_sotfVirtualTargetPub.set(new Pose2d(virtualTarget, result.aimingAngle()));
              m_sotfGoalPub.set(new Pose2d(goalPosition, new Rotation2d()));
              double staticDistance = goalPosition.getDistance(robotPose.getTranslation());
              m_sotfStaticDistancePub.set(staticDistance);
              m_sotfVirtualDistancePub.set(result.virtualDistance());
              m_sotfTimeOfFlightPub.set(result.timeOfFlight());
              m_sotfIterationsPub.set(result.iterations());
              m_sotfConvergedPub.set(result.converged());
              m_sotfContractionFactorPub.set(result.contractionFactor());
              m_sotfFirstOrderMissPub.set(result.firstOrderMiss());
              double robotSpeed =
                  Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
              m_sotfRobotSpeedPub.set(robotSpeed);
              m_sotfAimingAngleDegPub.set(result.aimingAngle().getDegrees());

              // The virtual distance is the distance to the converged virtual target —
              // use it directly for RPM lookup from our tuned tables
              double virtualDistance = result.virtualDistance();

              double bottomSpeedRps =
                  ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(virtualDistance) / 60.0;
              double topRollerSpeedRps =
                  ShooterConstants.TOP_ROLLER_SPEED_MAP.get(virtualDistance) / 60.0;

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (right motor is leader, left follows)
              m_shooterMotorRight.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorTopRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForSOTF");
  }

  /**
   * Creates a command that stops all shooter motors.
   *
   * @return A command that stops the shooter
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;
              m_shooterMotorRight.setControl(m_neutralRequest);
              m_shooterMotorTopRoller.setControl(m_neutralRequest);
            })
        .withName("StopShooter");
  }

  /**
   * Returns a trigger that is true when tuning mode is enabled via NetworkTables.
   *
   * @return the tuning enabled trigger
   */
  public Trigger tuningEnabledTrigger() {
    return m_tuningEnabledTrigger;
  }

  /**
   * Creates a command for tuning mode. Runs the shooter at the RPM values from
   * Shooter/Tuning/MainShooterRPM and Shooter/Tuning/RollerRPM. Use with tuningEnabledTrigger() to
   * schedule based on the Enabled entry.
   *
   * @return A command that runs the shooter in tuning mode.
   */
  public Command tuningCommand() {
    return this.run(
            () -> {
              double mainRps = m_tuningMainShooterRpsSub.get() / 60.0;
              double rollerRps = m_tuningRollerRpsSub.get() / 60.0;
              m_targetMainShooterRps = mainRps;
              m_targetRollerRps = rollerRps;
              m_shooterMotorRight.setControl(m_mainShooterVelocityRequest.withVelocity(mainRps));
              m_shooterMotorTopRoller.setControl(m_rollerVelocityRequest.withVelocity(rollerRps));
            })
        .finallyDo(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;
              m_shooterMotorRight.setControl(m_neutralRequest);
              m_shooterMotorTopRoller.setControl(m_neutralRequest);
            })
        .withName("ShooterTuning");
  }
}
