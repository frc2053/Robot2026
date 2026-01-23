// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {


  private TalonFX m_shooterMotorLeft;
  private TalonFX m_shooterMotorRight;
  private TalonFX m_shooterMotorTopRoller;

  private StatusSignal<AngularVelocity> m_leftMotorVel;
  private StatusSignal<AngularVelocity> m_rightMotorVel;
  private StatusSignal<AngularVelocity> m_rollerVel;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorLeft = new TalonFX(ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
    m_shooterMotorRight = new TalonFX(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);
    m_shooterMotorTopRoller = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ROLLER_ID);

    TalonFXConfiguration mainShooterConfig = new TalonFXConfiguration().withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(ShooterConstants.SHOOTER_STATOR_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ShooterConstants.SHOOTER_SUPPLY_LIMIT));

    StatusCode shooterLeftConfigResult = m_shooterMotorLeft.getConfigurator().apply(mainShooterConfig);
    if (!shooterLeftConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to left shooter motor!");
    }
    StatusCode shooterRightConfigResult = m_shooterMotorRight.getConfigurator().apply(mainShooterConfig);
    if (!shooterRightConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to right shooter motor!");
    }
    StatusCode rollerConfigResult = m_shooterMotorTopRoller.getConfigurator().apply(mainShooterConfig);
    if (!rollerConfigResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply config to roller shooter motor!");
    }

    m_leftMotorVel = m_shooterMotorLeft.getVelocity();
    m_rightMotorVel = m_shooterMotorRight.getVelocity();
    m_rollerVel = m_shooterMotorTopRoller.getVelocity();

    StatusCode setUpdateFreqResult = BaseStatusSignal.setUpdateFrequencyForAll(100, m_leftMotorVel, m_rightMotorVel, m_rollerVel);
    if(!setUpdateFreqResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply update frequency for shooter subsystem!");
    }
    StatusCode optiResult = ParentDevice.optimizeBusUtilizationForAll(m_shooterMotorLeft, m_shooterMotorRight, m_shooterMotorTopRoller);
    if(!optiResult.isOK()) {
      DataLogManager.log("ERROR! Not able to apply optimization for shooter subsystem!");
    }
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_leftMotorVel, m_rightMotorVel, m_rollerVel);
  }
}
