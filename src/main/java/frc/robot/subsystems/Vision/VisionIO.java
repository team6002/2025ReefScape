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

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
  // Modified to add pitch and 
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d CameraPose = new Pose2d();
    public boolean target = false;
    public double targetYaw = 0;
    public double targetDistance;
    public double targetXDistance;
    public double targetYDistance;
  }

    public default void updateInputs(VisionIOInputs inputs) {}
    public default void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){}
    public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {return null;}
    public default double getTargetYaw(){return 0;}
    public default Matrix<N3, N1> getEstimationStdDevs() {return null;}
    public default double getTargetYDistance(){return 0;}
    public default double getTargetXDistance(){return 0;}
    public default double getTargetDistance(){return 0;}
  
}