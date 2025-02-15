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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
  // Modified to add pitch and 
  @AutoLog
  public static class VisionIOInputs {
    // public Pose2d CameraPose = new Pose2d();
    public boolean LTarget = false;
    public Transform3d LTargetPose = new Transform3d();
    public boolean RTarget = false;
    public Transform3d RTargetPose = new Transform3d();
  }

    public default void updateInputs(VisionIOInputs inputs) {}
    public default void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){}
    public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {return null;}
    public default Optional<EstimatedRobotPose> getEstimatedGlobalPosenew(Pose2d prevEstimatePose) {return null;}
    public default Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {return null;}
    public default Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {return null;}
    public default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {return null;}
    public default Transform3d getTargetLPose(){return null;}
    public default Transform3d getTargetRPose(){return null;}
    public default void setMultiTagFallbackStrategy(PoseStrategy poseStrategy){}
  
}