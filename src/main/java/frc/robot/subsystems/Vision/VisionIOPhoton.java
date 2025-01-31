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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


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
        if (camera.getLatestResult().hasTargets()){
            return camera.getLatestResult();
        }else return null;
    } 
    
    @Override
    public Transform3d getTargetPose(){
        if (camera.getLatestResult().hasTargets()){
            return camera.getLatestResult().getBestTarget().getBestCameraToTarget();
        }else return null;
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

    @Override
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        int totalTags = -1; // a tag counter that counts all of the tags
        double avgDist = 0;
        double avgAng = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue; 
            totalTags ++;
            // if (angFilter(totalTags)) continue;
            numTags++;
            // if(tgt.getFiducialId() != 4 || tgt.getFiducialId() != 7) continue;
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (avgDist > 6)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        // getLatestResult(th).getBestTarget().getPoseAmbiguity();
        return estStdDevs;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.CameraPose = getEstimatedGlobalPose();
        if (camera.getLatestResult().hasTargets()){
            inputs.targetPose = getTargetPose();
        }
        inputs.target = camera.getLatestResult().hasTargets();   
        
    }

       
}