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
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final MotionMagicVoltage m_pivotPositionRequest = new MotionMagicVoltage(0);
  private final VoltageOut m_rollerVoltageRequest = new VoltageOut(0);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  // Track setpoints for logging
  private double m_pivotPositionSetpoint;
  private double m_rollerVoltageSetpoint;

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
    m_pivotVelocityPub = intakeTable.getDoubleTopic("PivotVelocityRPS").publish();
    m_pivotVoltagePub = intakeTable.getDoubleTopic("PivotVoltage").publish();
    m_pivotSetpointPub = intakeTable.getDoubleTopic("PivotSetpoint").publish();
    m_rollerVelocityPub = intakeTable.getDoubleTopic("RollerVelocityRPS").publish();
    m_rollerVoltagePub = intakeTable.getDoubleTopic("RollerVoltage").publish();
    m_rollerVoltageSetpointPub = intakeTable.getDoubleTopic("RollerVoltageSetpoint").publish();
    m_pivotStatorCurrentPub = intakeTable.getDoubleTopic("PivotStatorCurrent").publish();
    m_pivotSupplyCurrentPub = intakeTable.getDoubleTopic("PivotSupplyCurrent").publish();
    m_rollerStatorCurrentPub = intakeTable.getDoubleTopic("RollerStatorCurrent").publish();
    m_rollerSupplyCurrentPub = intakeTable.getDoubleTopic("RollerSupplyCurrent").publish();
    m_atPositionPub = intakeTable.getBooleanTopic("AtPosition").publish();
    m_currentCommandPub = intakeTable.getStringTopic("CurrentCommand").publish();

    // Initialize simulation
    m_pivotSimState = m_pivotMotor.getSimState();
    m_rollerSimState = m_rollerMotor.getSimState();

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
            Units.degreesToRadians(0)); // Starting angle

    // Roller flywheel simulation
    m_rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(1),
                IntakeConstants.ROLLER_MOI,
                IntakeConstants.ROLLER_GEAR_RATIO),
            DCMotor.getFalcon500(1));
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
    m_pivotMotor.setPosition(IntakeConstants.kPivotStowedPosition);
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
    m_pivotVelocityPub.set(m_pivotVelocity.getValue().in(RotationsPerSecond));
    m_pivotVoltagePub.set(m_pivotVoltage.getValue().in(Volts));
    m_pivotSetpointPub.set(m_pivotPositionSetpoint);
    m_rollerVelocityPub.set(m_rollerVelocity.getValue().in(RotationsPerSecond));
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
            new Rotation3d(0, -pivotAngleRad, 0)));
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
}
