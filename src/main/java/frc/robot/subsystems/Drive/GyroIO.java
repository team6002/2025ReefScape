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
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  // Modified to add pitch and 
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();    public double yawVelocity = 0.0;
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public boolean calib = false;
    public double angleAdjustment;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset(){}
  
  public default void set(Rotation2d rotation2d){}
}