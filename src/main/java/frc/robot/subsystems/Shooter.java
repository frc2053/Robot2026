// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ShootingOnTheFly;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

  private final TalonFX m_shooterMotorLeft;
  private final TalonFX m_shooterMotorRight;
  private final TalonFX m_shooterMotorLeftRoller;
  private final TalonFX m_shooterMotorRightRoller;

  private final StatusSignal<AngularVelocity> m_leftMotorVel;
  private final StatusSignal<AngularVelocity> m_rightMotorVel;
  private final StatusSignal<AngularVelocity> m_rollerLeftVel;
  private final StatusSignal<AngularVelocity> m_rollerRightVel;

  private final StatusSignal<Angle> m_leftMotorPos;
  private final StatusSignal<Angle> m_rightMotorPos;
  private final StatusSignal<Angle> m_rollerLeftPos;
  private final StatusSignal<Angle> m_rollerRightPos;

  private final StatusSignal<Voltage> m_leftMotorVoltage;
  private final StatusSignal<Voltage> m_rightMotorVoltage;
  private final StatusSignal<Voltage> m_rollerLeftVoltage;
  private final StatusSignal<Voltage> m_rollerRightVoltage;

  private final StatusSignal<Current> m_leftMotorStatorCurrent;
  private final StatusSignal<Current> m_rightMotorStatorCurrent;
  private final StatusSignal<Current> m_rollerLeftStatorCurrent;
  private final StatusSignal<Current> m_rollerRightStatorCurrent;
  private final StatusSignal<Current> m_leftMotorSupplyCurrent;
  private final StatusSignal<Current> m_rightMotorSupplyCurrent;
  private final StatusSignal<Current> m_rollerLeftSupplyCurrent;
  private final StatusSignal<Current> m_rollerRightSupplyCurrent;
  private final StatusSignal<Current> m_torqueCurrentMain;
  private final StatusSignal<Current> m_torqueCurrentRoller;

  // Simulation objects
  private final TalonFXSimState m_leftMotorSimState;
  private final TalonFXSimState m_rightMotorSimState;
  private final TalonFXSimState m_rollerLeftSimState;
  private final TalonFXSimState m_rollerRightSimState;

  private final DCMotorSim m_mainShooterSim;
  private final DCMotorSim m_rollerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_leftMotorVelPub;
  private final DoublePublisher m_rightMotorVelPub;
  private final DoublePublisher m_rollerLeftVelPub;
  private final DoublePublisher m_rollerRightVelPub; // new for right roller
  private final DoublePublisher m_leftMotorVoltagePub;
  private final DoublePublisher m_rightMotorVoltagePub;
  private final DoublePublisher m_rollerLeftVoltagePub;
  private final DoublePublisher m_rollerRightVoltagePub; // new for right roller
  private final DoublePublisher m_leftMotorStatorCurrentPub;
  private final DoublePublisher m_rightMotorStatorCurrentPub;
  private final DoublePublisher m_rollerLeftStatorCurrentPub;
  private final DoublePublisher m_rollerRightStatorCurrentPub; // new for right roller
  private final DoublePublisher m_leftMotorSupplyCurrentPub;
  private final DoublePublisher m_rightMotorSupplyCurrentPub;
  private final DoublePublisher m_rollerLeftSupplyCurrentPub;
  private final DoublePublisher m_rollerRightSupplyCurrentPub; // new for right roller
  private final StringPublisher m_currentCommandPub;
  private final BooleanPublisher m_atSpeedPub;
  private final BooleanPublisher m_atSpeedPubPassing;
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
  private final DoublePublisher m_sotfRobotSpeedPub;
  private final DoublePublisher m_sotfAimingAngleDegPub;

  // Target velocities for at-speed detection
  private double m_targetMainShooterRps;
  private double m_targetRollerRps;

  // Public trigger for when shooter is at speed
  private final Trigger m_atSpeedTrigger;
  private final Trigger m_atSpeedTriggerPassing;

  // Control requests - both use VelocityVoltage for better disturbance rejection
  private final VelocityVoltage m_mainShooterVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final VelocityVoltage m_rollerVelocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // SysId voltage request (separate from idle voltage to avoid conflicts)
  private final VoltageOut m_sysIdVoltageRequest = new VoltageOut(0).withEnableFOC(true);

  // SysId routines for characterization using Phoenix 6 Signal Logger
  private final SysIdRoutine m_mainShooterSysIdRoutine;
  private final SysIdRoutine m_rollerSysIdRoutine;

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
  private final DoublePublisher m_torqueCurrentMainPub;
  private final DoublePublisher m_torqueCurrentRollerPub;

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
    m_shooterMotorLeftRoller = new TalonFX(ShooterConstants.SHOOTER_MOTOR_LEFT_ROLLER_ID);
    m_shooterMotorRightRoller = new TalonFX(ShooterConstants.SHOOTER_MOTOR_RIGHT_ROLLER_ID);

    // Configure main shooter motors with VelocityVoltage control
    // No TorqueCurrentConfigs needed since we're using voltage-based velocity control
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
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kMainShooterKS)
                    .withKV(ShooterConstants.kMainShooterKV)
                    .withKA(ShooterConstants.kMainShooterKA)
                    .withKP(ShooterConstants.kMainShooterKP)
                    .withKI(ShooterConstants.kMainShooterKI)
                    .withKD(ShooterConstants.kMainShooterKD));

    StatusCode shooterLeftConfigResult =
        m_shooterMotorLeft.getConfigurator().apply(mainShooterConfig);
    if (!shooterLeftConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to left shooter motor!");
    }

    // New shooter has the motors mounted opposite faces
    StatusCode shooterRightConfigResult =
        m_shooterMotorRight.getConfigurator().apply(mainShooterConfig);
    if (!shooterRightConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to right shooter motor!");
    }

    // Raise the leader's DutyCycle broadcast rate so the follower mirrors output at 500 Hz
    // instead of the default 100 Hz. At 100 Hz the 19 Hz torsional mode between the two
    // coupled rotors gets a ~34° phase lag from the ZOH follower delay, which pumps the
    // mode. 500 Hz drops that to ~7°, below the threshold where it excites oscillation.
    m_shooterMotorLeft.getDutyCycle().setUpdateFrequency(500);

    // Run the right shooter motor as a strict follower of the left. A single PID loop on the
    // leader replaces the previous dueling dual-PID that excited the shared flywheel's
    // torsional mode.
    m_shooterMotorRight.setControl(new StrictFollower(m_shooterMotorLeft.getDeviceID()));

    // Roller config with VelocityVoltage control (allows both positive and negative voltage output)
    // No TorqueCurrentConfigs needed since we're using voltage-based velocity control
    TalonFXConfiguration rollerConfig =
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
            .withSlot0(
                new Slot0Configs()
                    .withKS(ShooterConstants.kRollerKS)
                    .withKV(ShooterConstants.kRollerKV)
                    .withKA(ShooterConstants.kRollerKA)
                    .withKP(ShooterConstants.kRollerKP)
                    .withKI(ShooterConstants.kRollerKI)
                    .withKD(ShooterConstants.kRollerKD));

    StatusCode rollerLeftConfigResult =
        m_shooterMotorLeftRoller.getConfigurator().apply(rollerConfig);
    if (!rollerLeftConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to roller shooter motor!");
    }

    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    StatusCode rollerRightConfigResult =
        m_shooterMotorRightRoller.getConfigurator().apply(rollerConfig);
    if (!rollerRightConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to roller shooter motor!");
    }
    m_leftMotorVel = m_shooterMotorLeft.getVelocity();
    m_rightMotorVel = m_shooterMotorRight.getVelocity();
    m_rollerLeftVel = m_shooterMotorLeftRoller.getVelocity();
    m_rollerRightVel = m_shooterMotorRightRoller.getVelocity();

    m_leftMotorPos = m_shooterMotorLeft.getPosition();
    m_rightMotorPos = m_shooterMotorRight.getPosition();
    m_rollerLeftPos = m_shooterMotorLeftRoller.getPosition();
    m_rollerRightPos = m_shooterMotorRightRoller.getPosition();

    m_leftMotorVoltage = m_shooterMotorLeft.getMotorVoltage();
    m_rightMotorVoltage = m_shooterMotorRight.getMotorVoltage();
    m_rollerLeftVoltage = m_shooterMotorLeftRoller.getMotorVoltage();
    m_rollerRightVoltage = m_shooterMotorRightRoller.getMotorVoltage();

    m_leftMotorStatorCurrent = m_shooterMotorLeft.getStatorCurrent();
    m_rightMotorStatorCurrent = m_shooterMotorRight.getStatorCurrent();
    m_rollerLeftStatorCurrent = m_shooterMotorLeftRoller.getStatorCurrent();
    m_rollerRightStatorCurrent = m_shooterMotorRightRoller.getStatorCurrent();

    m_leftMotorSupplyCurrent = m_shooterMotorLeft.getSupplyCurrent();
    m_rightMotorSupplyCurrent = m_shooterMotorRight.getSupplyCurrent();
    m_rollerLeftSupplyCurrent = m_shooterMotorLeftRoller.getSupplyCurrent();
    m_rollerRightSupplyCurrent = m_shooterMotorRightRoller.getSupplyCurrent();

    m_torqueCurrentMain = m_shooterMotorLeft.getTorqueCurrent();
    m_torqueCurrentRoller = m_shooterMotorLeftRoller.getTorqueCurrent();

    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_leftMotorVel,
            m_rightMotorVel,
            m_rollerLeftVel,
            m_rollerRightVel,
            m_leftMotorPos,
            m_rightMotorPos,
            m_rollerLeftPos,
            m_rollerRightPos,
            m_leftMotorVoltage,
            m_rightMotorVoltage,
            m_rollerLeftVoltage,
            m_rollerRightVoltage,
            m_leftMotorStatorCurrent,
            m_rightMotorStatorCurrent,
            m_rollerLeftStatorCurrent,
            m_rollerRightStatorCurrent,
            m_leftMotorSupplyCurrent,
            m_rightMotorSupplyCurrent,
            m_rollerLeftSupplyCurrent,
            m_rollerRightSupplyCurrent,
            m_torqueCurrentMain,
            m_torqueCurrentRoller);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for shooter subsystem!");
    }
    StatusCode optiResult =
        ParentDevice.optimizeBusUtilizationForAll(
            m_shooterMotorLeft,
            m_shooterMotorRight,
            m_shooterMotorLeftRoller,
            m_shooterMotorRightRoller);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for shooter subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    m_leftMotorVelPub = shooterTable.getDoubleTopic("LeftMotorVelocityRPM").publish();
    m_rightMotorVelPub = shooterTable.getDoubleTopic("RightMotorVelocityRPM").publish();
    m_rollerLeftVelPub = shooterTable.getDoubleTopic("RollerLeftVelocityRPM").publish();
    m_rollerRightVelPub = shooterTable.getDoubleTopic("RollerRightVelocityRPM").publish();

    m_leftMotorVoltagePub = shooterTable.getDoubleTopic("LeftMotorVoltage").publish();
    m_rightMotorVoltagePub = shooterTable.getDoubleTopic("RightMotorVoltage").publish();
    m_rollerLeftVoltagePub = shooterTable.getDoubleTopic("RollerLeftVoltage").publish();
    m_rollerRightVoltagePub = shooterTable.getDoubleTopic("RollerRightVoltage").publish();

    m_leftMotorStatorCurrentPub = shooterTable.getDoubleTopic("LeftMotorStatorCurrent").publish();
    m_rightMotorStatorCurrentPub = shooterTable.getDoubleTopic("RightMotorStatorCurrent").publish();
    m_rollerLeftStatorCurrentPub = shooterTable.getDoubleTopic("RollerLeftStatorCurrent").publish();
    m_rollerRightStatorCurrentPub =
        shooterTable.getDoubleTopic("RollerRightStatorCurrent").publish();

    m_leftMotorSupplyCurrentPub = shooterTable.getDoubleTopic("LeftMotorSupplyCurrent").publish();
    m_rightMotorSupplyCurrentPub = shooterTable.getDoubleTopic("RightMotorSupplyCurrent").publish();
    m_rollerLeftSupplyCurrentPub = shooterTable.getDoubleTopic("RollerLeftSupplyCurrent").publish();
    m_rollerRightSupplyCurrentPub =
        shooterTable.getDoubleTopic("RollerRightSupplyCurrent").publish();
    m_currentCommandPub = shooterTable.getStringTopic("CurrentCommand").publish();
    m_atSpeedPub = shooterTable.getBooleanTopic("AtSpeed").publish();
    m_atSpeedPubPassing = shooterTable.getBooleanTopic("AtSpeedPassing").publish();
    m_mainShooterSetpointPub = shooterTable.getDoubleTopic("MainShooterSetpointRPM").publish();
    m_rollerSetpointPub = shooterTable.getDoubleTopic("RollerSetpointRPM").publish();
    m_torqueCurrentMainPub = shooterTable.getDoubleTopic("TorqueCurrentMain").publish();
    m_torqueCurrentRollerPub = shooterTable.getDoubleTopic("TorqueCurrentRoller").publish();

    // Initialize at-speed trigger
    m_atSpeedTrigger = new Trigger(this::isAtSpeed);
    m_atSpeedTriggerPassing = new Trigger(this::isAtSpeedPassing);

    // Initialize simulation
    m_leftMotorSimState = m_shooterMotorLeft.getSimState();
    m_rightMotorSimState = m_shooterMotorRight.getSimState();
    m_rollerLeftSimState = m_shooterMotorLeftRoller.getSimState();
    m_rollerRightSimState = m_shooterMotorRightRoller.getSimState();
    // Set motor types for accurate simulation
    m_leftMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rightMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rollerLeftSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    m_rollerRightSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Main shooter flywheel sim (powered by left and right motors)
    // Using 2 Kraken X60 motors
    m_mainShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2),
                ShooterConstants.MAIN_SHOOTER_MOI,
                ShooterConstants.MAIN_SHOOTER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    // Roller flywheel sim (powered by 2 Kraken X60 motors)
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2),
                ShooterConstants.ROLLER_MOI,
                ShooterConstants.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    // Initialize SOTF debug publishers
    NetworkTable sotfTable = shooterTable.getSubTable("SOTF");
    m_sotfVirtualTargetPub = sotfTable.getStructTopic("VirtualTarget", Pose2d.struct).publish();
    m_sotfGoalPub = sotfTable.getStructTopic("Goal", Pose2d.struct).publish();
    m_sotfStaticDistancePub = sotfTable.getDoubleTopic("StaticDistanceM").publish();
    m_sotfVirtualDistancePub = sotfTable.getDoubleTopic("VirtualDistanceM").publish();
    m_sotfTimeOfFlightPub = sotfTable.getDoubleTopic("TimeOfFlightS").publish();
    m_sotfIterationsPub = sotfTable.getIntegerTopic("Iterations").publish();
    m_sotfConvergedPub = sotfTable.getBooleanTopic("Converged").publish();
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

    // Initialize SysId routine for main shooter characterization
    // Uses Phoenix 6 Signal Logger for hoot file logging
    m_mainShooterSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(10), // Dynamic step voltage (flywheels don't brownout easily)
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("sysid-main-shooter-state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  // Right motor follows the left via StrictFollower, so one request covers both.
                  m_shooterMotorLeft.setControl(m_sysIdVoltageRequest.withOutput(volts.in(Volts)));
                },
                null, // No log callback needed - Phoenix SignalLogger handles it
                this));

    // Initialize SysId routine for roller characterization
    // Uses Phoenix 6 Signal Logger for hoot file logging
    m_rollerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(10), // Dynamic step voltage (flywheels don't brownout easily)
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("sysid-roller-state", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> {
                  // Apply voltage to both roller motors independently
                  m_shooterMotorLeftRoller.setControl(
                      m_sysIdVoltageRequest.withOutput(volts.in(Volts)));
                  m_shooterMotorRightRoller.setControl(
                      m_sysIdVoltageRequest.withOutput(volts.in(Volts)));
                },
                null, // No log callback needed - Phoenix SignalLogger handles it
                this));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_leftMotorVel,
        m_rightMotorVel,
        m_rollerLeftVel,
        m_rollerRightVel,
        m_leftMotorVoltage,
        m_rightMotorVoltage,
        m_rollerLeftVoltage,
        m_rollerRightVoltage,
        m_leftMotorStatorCurrent,
        m_rightMotorStatorCurrent,
        m_rollerLeftStatorCurrent,
        m_rollerRightStatorCurrent,
        m_leftMotorSupplyCurrent,
        m_rightMotorSupplyCurrent,
        m_rollerLeftSupplyCurrent,
        m_rollerRightSupplyCurrent,
        m_torqueCurrentMain,
        m_torqueCurrentRoller);

    m_leftMotorVelPub.set(m_leftMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rightMotorVelPub.set(m_rightMotorVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerLeftVelPub.set(m_rollerLeftVel.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerRightVelPub.set(m_rollerRightVel.getValue().in(RotationsPerSecond) * 60.0);

    // Publish voltage signals to NetworkTables
    m_leftMotorVoltagePub.set(m_leftMotorVoltage.getValue().in(Volts));
    m_rightMotorVoltagePub.set(m_rightMotorVoltage.getValue().in(Volts));
    m_rollerLeftVoltagePub.set(m_rollerLeftVoltage.getValue().in(Volts));
    m_rollerRightVoltagePub.set(m_rollerRightVoltage.getValue().in(Volts));

    // Publish current signals to NetworkTables
    m_leftMotorStatorCurrentPub.set(m_leftMotorStatorCurrent.getValue().in(Amps));
    m_rightMotorStatorCurrentPub.set(m_rightMotorStatorCurrent.getValue().in(Amps));
    m_rollerLeftStatorCurrentPub.set(m_rollerLeftStatorCurrent.getValue().in(Amps));
    m_rollerRightStatorCurrentPub.set(m_rollerRightStatorCurrent.getValue().in(Amps));

    m_leftMotorSupplyCurrentPub.set(m_leftMotorSupplyCurrent.getValue().in(Amps));
    m_rightMotorSupplyCurrentPub.set(m_rightMotorSupplyCurrent.getValue().in(Amps));
    m_rollerLeftSupplyCurrentPub.set(m_rollerLeftSupplyCurrent.getValue().in(Amps));
    m_rollerRightSupplyCurrentPub.set(m_rollerRightSupplyCurrent.getValue().in(Amps));

    m_torqueCurrentMainPub.set(m_torqueCurrentMain.getValue().in(Amps));
    m_torqueCurrentRollerPub.set(m_torqueCurrentRoller.getValue().in(Amps));

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Publish at-speed status
    m_atSpeedPub.set(isAtSpeed());
    m_atSpeedPubPassing.set(isAtSpeedPassing());

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

      m_shooterMotorLeftRoller.getConfigurator().apply(rollerSlot0);
      m_shooterMotorRightRoller.getConfigurator().apply(rollerSlot0);

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
    m_rollerLeftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rollerRightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Main shooter flywheel: subtract voltages because right motor has opposite inversion
    // Left outputs +V, right outputs -V (inverted), so (left - right) / 2 gives true average
    double mainShooterVoltage =
        (m_leftMotorSimState.getMotorVoltageMeasure().in(Volts)
                - m_rightMotorSimState.getMotorVoltageMeasure().in(Volts))
            / 2.0;
    m_mainShooterSim.setInputVoltage(mainShooterVoltage);
    m_mainShooterSim.update(0.020);

    // Update left and right motor sim states with rotor position and velocity
    // Note: DCMotorSim returns mechanism values, multiply by gear ratio for rotor values
    // Right motor has opposite inversion so its raw position is negated
    m_leftMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_leftMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));

    m_rightMotorSimState.setRawRotorPosition(
        m_mainShooterSim.getAngularPosition().times(-ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));
    m_rightMotorSimState.setRotorVelocity(
        m_mainShooterSim.getAngularVelocity().times(-ShooterConstants.MAIN_SHOOTER_GEAR_RATIO));

    // Roller flywheel
    double mainRollerVoltage =
        (m_rollerLeftSimState.getMotorVoltageMeasure().in(Volts)
                - m_rollerRightSimState.getMotorVoltageMeasure().in(Volts))
            / 2.0;
    m_rollerSim.setInputVoltage(mainRollerVoltage);
    m_rollerSim.update(0.020);

    // Update roller motor sim state with rotor position and velocity
    m_rollerLeftSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(ShooterConstants.ROLLER_GEAR_RATIO));
    m_rollerLeftSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(ShooterConstants.ROLLER_GEAR_RATIO));

    m_rollerRightSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(-ShooterConstants.ROLLER_GEAR_RATIO));
    m_rollerRightSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(-ShooterConstants.ROLLER_GEAR_RATIO));
  }

  /**
   * Returns whether the shooter is at the target speed.
   *
   * @return true if both main shooter and roller are within tolerance of target speed
   */
  public boolean isAtSpeed() {
    double mainShooterVel = m_rightMotorVel.getValue().in(RotationsPerSecond);
    double rollerVel = m_rollerLeftVel.getValue().in(RotationsPerSecond);
    double rollerRightVel = m_rollerRightVel.getValue().in(RotationsPerSecond);
    double tolerance = m_velocityToleranceRpsSub.get();
    boolean mainAtSpeed = Math.abs(mainShooterVel - m_targetMainShooterRps) < tolerance;
    boolean rollerAtSpeed =
        Math.abs(rollerVel - m_targetRollerRps) < tolerance
            && Math.abs(rollerRightVel - m_targetRollerRps) < tolerance;
    return mainAtSpeed && rollerAtSpeed;
  }

  public boolean isAtSpeedPassing() {
    double mainShooterVel = m_rightMotorVel.getValue().in(RotationsPerSecond);
    double rollerVel = m_rollerLeftVel.getValue().in(RotationsPerSecond);
    double rollerRightVel = m_rollerRightVel.getValue().in(RotationsPerSecond);
    double tolerance = m_velocityToleranceRpsSub.get();
    boolean mainAtSpeed = Math.abs(mainShooterVel - m_targetMainShooterRps) < tolerance;
    boolean rollerAtSpeed =
        Math.abs(rollerVel - m_targetRollerRps) < 40.0
            && Math.abs(rollerRightVel - m_targetRollerRps) < 40.0;
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

  public Trigger atSpeedTriggerPassing() {
    return m_atSpeedTriggerPassing;
  }

  /**
   * Sets the target speeds of the shooter motors.
   *
   * @param mainSpeed the main shooter motor speed
   * @param topSpeed the speed of the top roller
   */
  public void setTargetSpeeds(double mainSpeed, double topSpeed) {
    m_targetMainShooterRps = mainSpeed;
    m_targetRollerRps = topSpeed;

    // Command the bottom shooter (left is leader, right follows via StrictFollower)
    m_shooterMotorLeft.setControl(m_mainShooterVelocityRequest.withVelocity(mainSpeed));

    // Command the top roller
    m_shooterMotorLeftRoller.setControl(m_rollerVelocityRequest.withVelocity(topSpeed));
    m_shooterMotorRightRoller.setControl(m_rollerVelocityRequest.withVelocity(topSpeed));
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

              m_shooterMotorLeft.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorLeftRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
              m_shooterMotorRightRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForDistance");
  }

  /**
   * Creates a command that spins up the shooter wheels to the predefined passing speed. Used for
   * passing game pieces to teammates across the field.
   *
   * @return A command that runs the shooter at the passing speeds
   */
  public Command spinUpForPassingCommand() {
    return this.run(
            () -> {
              double bottomSpeedRps = ShooterConstants.kPassingMainShooterRPM / 60.0;
              double topRollerSpeedRps = ShooterConstants.kPassingRollerRPM / 60.0;

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (left is leader, right follows via StrictFollower)
              m_shooterMotorLeft.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorLeftRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
              m_shooterMotorRightRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForPassing");
  }

  public Command idleVoltage(double idleVoltage) {
    return this.run(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;

              // Set bottom shooter motor (right follows via StrictFollower)
              m_shooterMotorLeft.setControl(m_voltageRequest.withOutput(idleVoltage));

              // Set top roller motor
              m_shooterMotorLeftRoller.setControl(m_voltageRequest.withOutput(idleVoltage));
              m_shooterMotorRightRoller.setControl(m_voltageRequest.withOutput(idleVoltage));
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
      Supplier<ChassisSpeeds> robotAccelSupplier,
      Supplier<Translation2d> goalPositionSupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = robotPoseSupplier.get();
              ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();
              ChassisSpeeds robotAccel = robotAccelSupplier.get();
              Translation2d goalPosition = goalPositionSupplier.get();

              // Calculate SOTF via iterative TOF recursion
              ShootingOnTheFly.SOTFResult result =
                  ShootingOnTheFly.calculate(
                      robotPose,
                      robotSpeeds,
                      robotAccel,
                      goalPosition,
                      ShooterConstants.kSOTFLatencyCompensation,
                      ShooterConstants.TIME_OF_FLIGHT_MAP);

              // Publish SOTF debug data to NetworkTables
              Translation2d virtualTarget = result.virtualTarget();
              m_sotfVirtualTargetPub.set(new Pose2d(virtualTarget, result.aimingAngle()));
              m_sotfGoalPub.set(new Pose2d(goalPosition, new Rotation2d()));
              double staticDistance =
                  goalPosition.getDistance(robotPose.getTranslation())
                      - FuelConstants.kLookupTableDistanceOffset
                      - FuelConstants.getAllianceLookupOffset();
              m_sotfStaticDistancePub.set(staticDistance);
              m_sotfVirtualDistancePub.set(result.virtualDistance());
              m_sotfTimeOfFlightPub.set(result.timeOfFlight());
              m_sotfIterationsPub.set(result.iterations());
              m_sotfConvergedPub.set(result.converged());
              double robotSpeed =
                  Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
              m_sotfRobotSpeedPub.set(robotSpeed);
              m_sotfAimingAngleDegPub.set(result.aimingAngle().getDegrees());

              // Convert virtual target distance to lookup table reference frame
              // (front-of-bumper to front-of-hub)
              double lookupDistance =
                  Math.max(
                      0,
                      result.virtualTarget().getDistance(robotPose.getTranslation())
                          - FuelConstants.kLookupTableDistanceOffset
                          - FuelConstants.getAllianceLookupOffset());

              double bottomSpeedRps =
                  ShooterConstants.BOTTOM_SHOOTER_SPEED_MAP.get(lookupDistance) / 60.0;
              double topRollerSpeedRps =
                  ShooterConstants.TOP_ROLLER_SPEED_MAP.get(lookupDistance) / 60.0;

              // Store target velocities for at-speed detection
              m_targetMainShooterRps = bottomSpeedRps;
              m_targetRollerRps = topRollerSpeedRps;

              // Command the bottom shooter (left is leader, right follows via StrictFollower)
              m_shooterMotorLeft.setControl(
                  m_mainShooterVelocityRequest.withVelocity(bottomSpeedRps));

              // Command the top roller
              m_shooterMotorLeftRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
              m_shooterMotorRightRoller.setControl(
                  m_rollerVelocityRequest.withVelocity(topRollerSpeedRps));
            })
        .withName("SpinUpForSOTF");
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
              m_shooterMotorLeft.setControl(m_neutralRequest);
              m_shooterMotorLeftRoller.setControl(m_neutralRequest);
              m_shooterMotorRightRoller.setControl(m_neutralRequest);
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
              m_shooterMotorLeft.setControl(m_mainShooterVelocityRequest.withVelocity(mainRps));
              m_shooterMotorLeftRoller.setControl(m_rollerVelocityRequest.withVelocity(rollerRps));
              m_shooterMotorRightRoller.setControl(m_rollerVelocityRequest.withVelocity(rollerRps));
            })
        .finallyDo(
            () -> {
              m_targetMainShooterRps = 0.0;
              m_targetRollerRps = 0.0;
              m_shooterMotorLeft.setControl(m_neutralRequest);
              m_shooterMotorLeftRoller.setControl(m_neutralRequest);
              m_shooterMotorRightRoller.setControl(m_neutralRequest);
            })
        .withName("ShooterTuning");
  }

  /**
   * Returns a command for roller SysId quasistatic characterization.
   *
   * @param direction the direction to run (forward or reverse)
   * @return the quasistatic SysId command
   */
  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_rollerSysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command for roller SysId dynamic characterization.
   *
   * @param direction the direction to run (forward or reverse)
   * @return the dynamic SysId command
   */
  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_rollerSysIdRoutine.dynamic(direction);
  }

  /**
   * Returns a command for main shooter SysId quasistatic characterization.
   *
   * @param direction the direction to run (forward or reverse).
   * @return the quasistatic SysId command.
   */
  public Command mainShooterSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_mainShooterSysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command for main shooter SysId dynamic characterization.
   *
   * @param direction the direction to run (forward or reverse).
   * @return the dynamic SysId command.
   */
  public Command mainShooterSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_mainShooterSysIdRoutine.dynamic(direction);
  }
}
