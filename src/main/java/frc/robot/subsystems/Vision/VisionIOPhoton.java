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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.GyroIO.GyroIOInputs;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class VisionIOPhoton implements VisionIO{  
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kFrontCameraName);
    private final PhotonPoseEstimator photonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
    private Matrix<N3, N1> curStdDevs;

    public void setFrontCameraPipeline(int pipeline){
        camera.setPipelineIndex(pipeline);
    }

    public void setMultiTagFallbackStrategy(PoseStrategy poseStrategy){
        photonEstimator.setMultiTagFallbackStrategy(poseStrategy);
    }
    
    public void setFrontCameraDriverMode(boolean bootlean){
        camera.setDriverMode(bootlean);
    }

    @Override 
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }
    @Override
    public double getTargetYaw(){
        return 
        new Rotation2d(Math.toRadians(180)).plus(getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().toRotation2d()).getDegrees();
    }   
    
    @Override
    public void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.CameraPose = getEstimatedGlobalPose();
        inputs.targetYaw = getTargetYaw();
        inputs.target = camera.getLatestResult().hasTargets();
        
    }
}