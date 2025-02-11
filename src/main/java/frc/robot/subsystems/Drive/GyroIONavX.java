// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class GyroIONavX implements GyroIO{  
    private final AHRS m_gyro = new AHRS(NavXComType.kI2C.kMXP_SPI);  

  @Override
  public void updateInputs(GyroIOInputs inputs){
    inputs.connected = m_gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(m_gyro.getYaw());
    inputs.pitchPosition = Rotation2d.fromDegrees(m_gyro.getPitch());
    inputs.rollPosition = Rotation2d.fromDegrees(m_gyro.getRoll());
    inputs.yawVelocity = m_gyro.getRate();
    // inputs.yawVelocityRadPerSec = m_gyro.getYawve()
  }
}