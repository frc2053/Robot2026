// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MechanismVisualizer;

/** Intake subsystem with a pivot arm and roller motor. */
public class Intake extends SubsystemBase {

  private final TalonFX m_pivotMotor;
  private final TalonFX m_rollerMotor;

  // Pivot status signals
  private final StatusSignal<Angle> m_pivotPosition;
  private final StatusSignal<AngularVelocity> m_pivotVelocity;
  private final StatusSignal<Voltage> m_pivotVoltage;

  // Roller status signals
  private final StatusSignal<AngularVelocity> m_rollerVelocity;
  private final StatusSignal<Voltage> m_rollerVoltage;

  // Current status signals
  private final StatusSignal<Current> m_pivotStatorCurrent;
  private final StatusSignal<Current> m_pivotSupplyCurrent;
  private final StatusSignal<Current> m_rollerStatorCurrent;
  private final StatusSignal<Current> m_rollerSupplyCurrent;

  // Simulation objects
  private final TalonFXSimState m_pivotSimState;
  private final TalonFXSimState m_rollerSimState;
  private final SingleJointedArmSim m_pivotSim;
  private final DCMotorSim m_rollerSim;

  // NetworkTables publishers for logging
  private final DoublePublisher m_pivotPositionPub;
  private final DoublePublisher m_pivotVelocityPub;
  private final DoublePublisher m_pivotVoltagePub;
  private final DoublePublisher m_pivotSetpointPub;
  private final DoublePublisher m_rollerVelocityPub;
  private final DoublePublisher m_rollerVoltagePub;
  private final DoublePublisher m_rollerVoltageSetpointPub;
  private final DoublePublisher m_pivotStatorCurrentPub;
  private final DoublePublisher m_pivotSupplyCurrentPub;
  private final DoublePublisher m_rollerStatorCurrentPub;
  private final DoublePublisher m_rollerSupplyCurrentPub;
  private final BooleanPublisher m_atPositionPub;
  private final StringPublisher m_currentCommandPub;

  // Control requests
  private final MotionMagicVoltage m_pivotPositionRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut m_rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Track setpoints for logging
  private double m_pivotPositionSetpoint;
  private double m_rollerVoltageSetpoint;

  // State variable for feedingWigglePivotCommand oscillation direction
  private boolean m_wiggleGoingToTop;

  // Tunable gains for pivot
  private final DoubleSubscriber m_pivotKSSub;
  private final DoubleSubscriber m_pivotKGSub;
  private final DoubleSubscriber m_pivotKVSub;
  private final DoubleSubscriber m_pivotKASub;
  private final DoubleSubscriber m_pivotKPSub;
  private final DoubleSubscriber m_pivotKISub;
  private final DoubleSubscriber m_pivotKDSub;
  private final DoublePublisher m_pivotKSPub;
  private final DoublePublisher m_pivotKGPub;
  private final DoublePublisher m_pivotKVPub;
  private final DoublePublisher m_pivotKAPub;
  private final DoublePublisher m_pivotKPPub;
  private final DoublePublisher m_pivotKIPub;
  private final DoublePublisher m_pivotKDPub;

  private final Timer m_wiggleTimer;

  // Track last known gain values to detect changes
  private double m_lastPivotKS;
  private double m_lastPivotKG;
  private double m_lastPivotKV;
  private double m_lastPivotKA;
  private double m_lastPivotKP;
  private double m_lastPivotKI;
  private double m_lastPivotKD;

  // Tuning mode NetworkTables controls
  private final BooleanSubscriber m_tuningEnabledSub;
  private final BooleanPublisher m_tuningEnabledPub;
  private final Trigger m_tuningEnabledTrigger;
  private final DoubleSubscriber m_tuningPivotPositionSub;
  private final DoublePublisher m_tuningPivotPositionPub;

  /** Creates a new Intake subsystem. */
  public Intake() {
    m_pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    m_rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

    configurePivotMotor();
    configureRollerMotor();

    // Get pivot status signals
    m_pivotPosition = m_pivotMotor.getPosition();
    m_pivotVelocity = m_pivotMotor.getVelocity();
    m_pivotVoltage = m_pivotMotor.getMotorVoltage();

    // Get roller status signals
    m_rollerVelocity = m_rollerMotor.getVelocity();
    m_rollerVoltage = m_rollerMotor.getMotorVoltage();

    // Get current status signals
    m_pivotStatorCurrent = m_pivotMotor.getStatorCurrent();
    m_pivotSupplyCurrent = m_pivotMotor.getSupplyCurrent();
    m_rollerStatorCurrent = m_rollerMotor.getStatorCurrent();
    m_rollerSupplyCurrent = m_rollerMotor.getSupplyCurrent();

    m_wiggleTimer = new Timer();

    // Set update frequencies
    StatusCode setUpdateFreqResult =
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            m_pivotPosition,
            m_pivotVelocity,
            m_pivotVoltage,
            m_rollerVelocity,
            m_rollerVoltage,
            m_pivotStatorCurrent,
            m_pivotSupplyCurrent,
            m_rollerStatorCurrent,
            m_rollerSupplyCurrent);
    if (!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for intake subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_pivotMotor, m_rollerMotor);
    if (!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for intake subsystem!");
    }

    // Initialize NetworkTables publishers for logging
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    m_pivotPositionPub = intakeTable.getDoubleTopic("PivotPositionRotations").publish();
    m_pivotVelocityPub = intakeTable.getDoubleTopic("PivotVelocityRPM").publish();
    m_pivotVoltagePub = intakeTable.getDoubleTopic("PivotVoltage").publish();
    m_pivotSetpointPub = intakeTable.getDoubleTopic("PivotSetpoint").publish();
    m_rollerVelocityPub = intakeTable.getDoubleTopic("RollerVelocityRPM").publish();
    m_rollerVoltagePub = intakeTable.getDoubleTopic("RollerVoltage").publish();
    m_rollerVoltageSetpointPub = intakeTable.getDoubleTopic("RollerVoltageSetpoint").publish();
    m_pivotStatorCurrentPub = intakeTable.getDoubleTopic("PivotStatorCurrent").publish();
    m_pivotSupplyCurrentPub = intakeTable.getDoubleTopic("PivotSupplyCurrent").publish();
    m_rollerStatorCurrentPub = intakeTable.getDoubleTopic("RollerStatorCurrent").publish();
    m_rollerSupplyCurrentPub = intakeTable.getDoubleTopic("RollerSupplyCurrent").publish();
    m_atPositionPub = intakeTable.getBooleanTopic("AtPosition").publish();
    m_currentCommandPub = intakeTable.getStringTopic("CurrentCommand").publish();

    // Initialize tunable gains for pivot
    NetworkTable tuningTable = intakeTable.getSubTable("Tuning").getSubTable("PivotGains");

    m_pivotKSPub = tuningTable.getDoubleTopic("kS").publish();
    m_pivotKSSub = tuningTable.getDoubleTopic("kS").subscribe(IntakeConstants.kPivotKS);
    m_pivotKSPub.set(IntakeConstants.kPivotKS);
    m_lastPivotKS = IntakeConstants.kPivotKS;

    m_pivotKGPub = tuningTable.getDoubleTopic("kG").publish();
    m_pivotKGSub = tuningTable.getDoubleTopic("kG").subscribe(IntakeConstants.kPivotKG);
    m_pivotKGPub.set(IntakeConstants.kPivotKG);
    m_lastPivotKG = IntakeConstants.kPivotKG;

    m_pivotKVPub = tuningTable.getDoubleTopic("kV").publish();
    m_pivotKVSub = tuningTable.getDoubleTopic("kV").subscribe(IntakeConstants.kPivotKV);
    m_pivotKVPub.set(IntakeConstants.kPivotKV);
    m_lastPivotKV = IntakeConstants.kPivotKV;

    m_pivotKAPub = tuningTable.getDoubleTopic("kA").publish();
    m_pivotKASub = tuningTable.getDoubleTopic("kA").subscribe(IntakeConstants.kPivotKA);
    m_pivotKAPub.set(IntakeConstants.kPivotKA);
    m_lastPivotKA = IntakeConstants.kPivotKA;

    m_pivotKPPub = tuningTable.getDoubleTopic("kP").publish();
    m_pivotKPSub = tuningTable.getDoubleTopic("kP").subscribe(IntakeConstants.kPivotKP);
    m_pivotKPPub.set(IntakeConstants.kPivotKP);
    m_lastPivotKP = IntakeConstants.kPivotKP;

    m_pivotKIPub = tuningTable.getDoubleTopic("kI").publish();
    m_pivotKISub = tuningTable.getDoubleTopic("kI").subscribe(IntakeConstants.kPivotKI);
    m_pivotKIPub.set(IntakeConstants.kPivotKI);
    m_lastPivotKI = IntakeConstants.kPivotKI;

    m_pivotKDPub = tuningTable.getDoubleTopic("kD").publish();
    m_pivotKDSub = tuningTable.getDoubleTopic("kD").subscribe(IntakeConstants.kPivotKD);
    m_pivotKDPub.set(IntakeConstants.kPivotKD);
    m_lastPivotKD = IntakeConstants.kPivotKD;

    // Initialize tuning mode controls (in parent Tuning table)
    NetworkTable tuningParent = intakeTable.getSubTable("Tuning");
    m_tuningEnabledPub = tuningParent.getBooleanTopic("Enabled").publish();
    m_tuningEnabledSub = tuningParent.getBooleanTopic("Enabled").subscribe(false);
    m_tuningEnabledPub.set(false);
    m_tuningEnabledTrigger = new Trigger(m_tuningEnabledSub::get);
    m_tuningPivotPositionPub = tuningParent.getDoubleTopic("PivotPositionRotations").publish();
    m_tuningPivotPositionSub =
        tuningParent
            .getDoubleTopic("PivotPositionRotations")
            .subscribe(IntakeConstants.kPivotStowedPosition);
    m_tuningPivotPositionPub.set(IntakeConstants.kPivotStowedPosition);

    // Initialize simulation
    m_pivotSimState = m_pivotMotor.getSimState();
    m_rollerSimState = m_rollerMotor.getSimState();

    // Set pivot sim orientation to match motor invert (Clockwise_Positive)
    m_pivotSimState.Orientation = ChassisReference.Clockwise_Positive;

    // Pivot arm simulation (single jointed arm)
    m_pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            IntakeConstants.PIVOT_GEAR_RATIO,
            IntakeConstants.PIVOT_MOI,
            IntakeConstants.PIVOT_ARM_LENGTH_METERS,
            Units.degreesToRadians(-10), // Min angle (slightly past stowed)
            Units.degreesToRadians(100), // Max angle (past deployed)
            true, // Simulate gravity
            Units.rotationsToRadians(0)); // Starting angle (stowed)

    // Roller flywheel simulation
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500Foc(1),
                IntakeConstants.ROLLER_MOI,
                IntakeConstants.ROLLER_GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));
  }

  private void configurePivotMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current limits
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(IntakeConstants.PIVOT_STATOR_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.PIVOT_SUPPLY_LIMIT);

    // Feedback configuration - use gear ratio for mechanism position
    config.Feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.PIVOT_GEAR_RATIO);

    // Slot 0 - Position control with gravity compensation
    config.Slot0 =
        new Slot0Configs()
            .withKS(IntakeConstants.kPivotKS)
            .withKG(IntakeConstants.kPivotKG)
            .withKV(IntakeConstants.kPivotKV)
            .withKA(IntakeConstants.kPivotKA)
            .withKP(IntakeConstants.kPivotKP)
            .withKI(IntakeConstants.kPivotKI)
            .withKD(IntakeConstants.kPivotKD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    // Motion Magic configuration
    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(IntakeConstants.kPivotMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(IntakeConstants.kPivotMotionMagicAcceleration);

    StatusCode configResult = m_pivotMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to intake pivot motor!");
    }

    // Set initial position to stowed
    // m_pivotMotor.setPosition(IntakeConstants.kPivotStowedPosition);
  }

  private void configureRollerMotor() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeConstants.ROLLER_STATOR_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IntakeConstants.ROLLER_SUPPLY_LIMIT));

    StatusCode configResult = m_rollerMotor.getConfigurator().apply(config);
    if (!configResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to intake roller motor!");
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_pivotPosition,
        m_pivotVelocity,
        m_pivotVoltage,
        m_rollerVelocity,
        m_rollerVoltage,
        m_pivotStatorCurrent,
        m_pivotSupplyCurrent,
        m_rollerStatorCurrent,
        m_rollerSupplyCurrent);

    // Publish signals to NetworkTables
    m_pivotPositionPub.set(m_pivotPosition.getValue().in(Rotations));
    m_pivotVelocityPub.set(m_pivotVelocity.getValue().in(RotationsPerSecond) * 60.0);
    m_pivotVoltagePub.set(m_pivotVoltage.getValue().in(Volts));
    m_pivotSetpointPub.set(m_pivotPositionSetpoint);
    m_rollerVelocityPub.set(m_rollerVelocity.getValue().in(RotationsPerSecond) * 60.0);
    m_rollerVoltagePub.set(m_rollerVoltage.getValue().in(Volts));
    m_rollerVoltageSetpointPub.set(m_rollerVoltageSetpoint);
    m_pivotStatorCurrentPub.set(m_pivotStatorCurrent.getValue().in(Amps));
    m_pivotSupplyCurrentPub.set(m_pivotSupplyCurrent.getValue().in(Amps));
    m_rollerStatorCurrentPub.set(m_rollerStatorCurrent.getValue().in(Amps));
    m_rollerSupplyCurrentPub.set(m_rollerSupplyCurrent.getValue().in(Amps));
    m_atPositionPub.set(atPosition());

    // Publish current command name
    Command currentCommand = getCurrentCommand();
    m_currentCommandPub.set(currentCommand != null ? currentCommand.getName() : "None");

    // Update mechanism pose for AdvantageScope 3D visualization
    double pivotAngleRad = Units.rotationsToRadians(m_pivotPosition.getValue().in(Rotations));
    MechanismVisualizer.setPose(
        MechanismVisualizer.INTAKE_INDEX,
        new Pose3d(
            Units.inchesToMeters(7.8296),
            0,
            Units.inchesToMeters(11.25),
            new Rotation3d(0, -pivotAngleRad + Units.degreesToRadians(102.2053), 0)));

    // Update hopper pose based on intake contact point position
    double contactDistanceMeters = Units.inchesToMeters(9.755401);
    double pivotXMeters = Units.inchesToMeters(7.8296);
    double actualAngle = -pivotAngleRad + Units.degreesToRadians(102.2053);
    double contactX = pivotXMeters + contactDistanceMeters * Math.cos(actualAngle);
    // Invert so hopper extends when intake deploys
    double hopperX = (pivotXMeters + contactDistanceMeters) - contactX;
    MechanismVisualizer.setPose(
        MechanismVisualizer.HOPPER_INDEX, new Pose3d(hopperX, 0, 0, new Rotation3d()));

    // Check for tuning updates and apply if changed
    updateTunableGains();
  }

  /** Checks for tunable gain updates from NetworkTables and applies them to the pivot motor. */
  private void updateTunableGains() {
    // Read current values from NetworkTables
    double kS = m_pivotKSSub.get();
    double kG = m_pivotKGSub.get();
    double kV = m_pivotKVSub.get();
    double kA = m_pivotKASub.get();
    double kP = m_pivotKPSub.get();
    double kI = m_pivotKISub.get();
    double kD = m_pivotKDSub.get();

    // Check if gains have changed
    boolean gainsChanged =
        kS != m_lastPivotKS
            || kG != m_lastPivotKG
            || kV != m_lastPivotKV
            || kA != m_lastPivotKA
            || kP != m_lastPivotKP
            || kI != m_lastPivotKI
            || kD != m_lastPivotKD;

    if (gainsChanged) {
      Slot0Configs slot0 =
          new Slot0Configs()
              .withKS(kS)
              .withKG(kG)
              .withKV(kV)
              .withKA(kA)
              .withKP(kP)
              .withKI(kI)
              .withKD(kD)
              .withGravityType(GravityTypeValue.Arm_Cosine);

      m_pivotMotor.getConfigurator().apply(slot0);

      // Update last known values
      m_lastPivotKS = kS;
      m_lastPivotKG = kG;
      m_lastPivotKV = kV;
      m_lastPivotKA = kA;
      m_lastPivotKP = kP;
      m_lastPivotKI = kI;
      m_lastPivotKD = kD;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage from battery
    m_pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update pivot arm simulation
    m_pivotSim.setInputVoltage(m_pivotSimState.getMotorVoltageMeasure().in(Volts));
    m_pivotSim.update(0.020);

    // Feed pivot simulation results back to motor simulation
    // Convert radians to rotations for the mechanism position
    double pivotMechanismRotations = Units.radiansToRotations(m_pivotSim.getAngleRads());
    m_pivotSimState.setRawRotorPosition(pivotMechanismRotations * IntakeConstants.PIVOT_GEAR_RATIO);
    m_pivotSimState.setRotorVelocity(
        Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec())
            * IntakeConstants.PIVOT_GEAR_RATIO);

    // Update roller simulation
    m_rollerSim.setInputVoltage(m_rollerSimState.getMotorVoltageMeasure().in(Volts));
    m_rollerSim.update(0.020);

    // Feed roller simulation results back to motor simulation
    m_rollerSimState.setRawRotorPosition(
        m_rollerSim.getAngularPosition().times(IntakeConstants.ROLLER_GEAR_RATIO));
    m_rollerSimState.setRotorVelocity(
        m_rollerSim.getAngularVelocity().times(IntakeConstants.ROLLER_GEAR_RATIO));
  }

  /**
   * Checks if the pivot is at the target position.
   *
   * @return true if at position within tolerance.
   */
  public boolean atPosition() {
    return Math.abs(m_pivotPosition.getValue().in(Rotations) - m_pivotPositionSetpoint)
        < IntakeConstants.kPivotPositionToleranceRotations;
  }

  /**
   * Creates a command to deploy the intake (pivot down and run rollers).
   *
   * @return A command that deploys the intake.
   */
  public Command deployCommand() {
    return this.run(
            () -> {
              m_pivotPositionSetpoint = IntakeConstants.kPivotDeployedPosition;
              m_pivotMotor.setControl(
                  m_pivotPositionRequest.withPosition(IntakeConstants.kPivotDeployedPosition));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("DeployIntake");
  }

  /**
   * Creates a command to deploy the intake (pivot down and run rollers).
   *
   * @return A command that deploys the intake.
   */
  public Command autoDeployCommand() {
    return this.run(
            () -> {
              m_pivotPositionSetpoint = IntakeConstants.kPivotDeployedPosition;
              m_pivotMotor.setControl(
                  m_pivotPositionRequest.withPosition(IntakeConstants.kPivotDeployedPosition));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .until(
            () -> {
              return atPosition();
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("AutoDeployIntake");
  }

  public Command deployOnly() {
    return this.runOnce(
            () -> {
              m_pivotPositionSetpoint = IntakeConstants.kPivotDeployedPosition;
              m_pivotMotor.setControl(
                  m_pivotPositionRequest.withPosition(IntakeConstants.kPivotDeployedPosition));
            })
        .withName("DeployOnly");
  }

  /**
   * Creates a command to stow the intake (pivot up and stop rollers).
   *
   * @return A command that stows the intake.
   */
  public Command stowCommand() {
    return this.run(
            () -> {
              m_pivotPositionSetpoint = IntakeConstants.kPivotStowedPosition;
              m_pivotMotor.setControl(
                  m_pivotPositionRequest.withPosition(IntakeConstants.kPivotStowedPosition));
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("StowIntake");
  }

  public Command runRollers() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRollers");
  }

  public Command runAutoRollers() {
    return this.runOnce(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
            })
        .withName("RunIntakeRollers");
  }

  public Command runRollersReverse() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = -IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(-IntakeConstants.kIntakeVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRollersReverse");
  }

  /**
   * Creates a command to eject game pieces (run rollers in reverse).
   *
   * @return A command that ejects game pieces.
   */
  public Command ejectCommand() {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = IntakeConstants.kEjectVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kEjectVoltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("EjectIntake");
  }

  /**
   * Creates a command to set the pivot to a specific position.
   *
   * @param positionRotations The target position in rotations.
   * @return A command that moves the pivot.
   */
  public Command setPivotPositionCommand(double positionRotations) {
    return this.run(
            () -> {
              m_pivotPositionSetpoint = positionRotations;
              m_pivotMotor.setControl(m_pivotPositionRequest.withPosition(positionRotations));
            })
        .withName("SetPivotPosition");
  }

  /**
   * Creates a command to run the roller at a specific voltage.
   *
   * @param voltage The voltage to run at.
   * @return A command that runs the roller.
   */
  public Command runRollerVoltageCommand(double voltage) {
    return this.run(
            () -> {
              m_rollerVoltageSetpoint = voltage;
              m_rollerMotor.setControl(m_rollerVoltageRequest.withOutput(voltage));
            })
        .finallyDo(
            () -> {
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("RunIntakeRoller");
  }

  /**
   * Creates a command that stops both motors.
   *
   * @return A command that stops the intake.
   */
  public Command stopCommand() {
    return this.runOnce(
            () -> {
              m_pivotPositionSetpoint = m_pivotPosition.getValue().in(Rotations);
              m_pivotMotor.setControl(m_neutralRequest);
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("StopIntake");
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
   * Creates a command that continuously wiggles the pivot between the deployed position and 20
   * degrees above it while feeding. Returns the pivot to the deployed position when the command
   * ends.
   *
   * @return A command that wiggles the pivot during feeding.
   */
  public Command feedingWigglePivotCommand() {
    return this.run(
            () -> {
              double targetPos =
                  m_wiggleGoingToTop
                      ? IntakeConstants.kPivotDeployedPosition
                          + IntakeConstants.kPivotFeedingWiggleOffset
                      : IntakeConstants.kPivotDeployedPosition;
              m_pivotPositionSetpoint = targetPos;
              m_pivotMotor.setControl(m_pivotPositionRequest.withPosition(targetPos));
              m_rollerVoltageSetpoint = IntakeConstants.kIntakeVoltage;
              m_rollerMotor.setControl(
                  m_rollerVoltageRequest.withOutput(IntakeConstants.kIntakeVoltage));
              if (m_wiggleTimer.hasElapsed(0.60)) {
                m_wiggleGoingToTop = !m_wiggleGoingToTop;
                m_wiggleTimer.reset();
                m_wiggleTimer.start();
              }
            })
        .beforeStarting(
            () -> {
              m_wiggleGoingToTop = false;
              m_wiggleTimer.reset();
              m_wiggleTimer.start();
            })
        .finallyDo(
            () -> {
              m_pivotPositionSetpoint = IntakeConstants.kPivotDeployedPosition;
              m_pivotMotor.setControl(
                  m_pivotPositionRequest.withPosition(IntakeConstants.kPivotDeployedPosition));
              m_rollerVoltageSetpoint = 0.0;
              m_rollerMotor.setControl(m_neutralRequest);
            })
        .withName("FeedingWigglePivot");
  }

  /**
   * Creates a command for tuning mode. Moves the pivot to the position from
   * Intake/Tuning/PivotPositionRotations. Use with tuningEnabledTrigger() to schedule based on the
   * Enabled entry.
   *
   * @return A command that runs the intake in tuning mode.
   */
  public Command tuningCommand() {
    return this.run(
            () -> {
              double targetPosition = m_tuningPivotPositionSub.get();
              m_pivotPositionSetpoint = targetPosition;
              m_pivotMotor.setControl(m_pivotPositionRequest.withPosition(targetPosition));
            })
        .finallyDo(() -> m_pivotMotor.setControl(m_neutralRequest))
        .withName("IntakeTuning");
  }
}
