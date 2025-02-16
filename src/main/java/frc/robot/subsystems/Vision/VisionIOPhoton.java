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

import java.util.ArrayList;
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
    private final PhotonCamera LCamera = new PhotonCamera(VisionConstants.kLeftCameraName);
    private final PhotonCamera RCamera = new PhotonCamera(VisionConstants.kRightCameraName);
    private final PhotonPoseEstimator photonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLCam);
    private Matrix<N3, N1> curStdDevs;

    public void setCameraPipeline(int LPipeline, int RPipeline){
        LCamera.setPipelineIndex(LPipeline);
        RCamera.setPipelineIndex(RPipeline);
    }

    @Override
    public void setMultiTagFallbackStrategy(PoseStrategy poseStrategy){
        photonEstimator.setMultiTagFallbackStrategy(poseStrategy);
    }
    
    public void setCameraDriverMode(boolean bootlean){
        LCamera.setDriverMode(bootlean);
        RCamera.setDriverMode(bootlean);
    }

    @Override 
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : RCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        for (var change : LCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }


    public PhotonPipelineResult getLatestLResult() {
        if (LCamera.getLatestResult().hasTargets()){
            return LCamera.getLatestResult();
        }else return null;
    } 

    public PhotonPipelineResult getLatestRResult() {
        if (RCamera.getLatestResult().hasTargets()){
            return RCamera.getLatestResult();
        }else return null;
    }

    @Override
    public Transform3d getTargetLPose(){
        if (LCamera.getLatestResult().hasTargets()){
            if (LCamera.getLatestResult().getBestTarget().getBestCameraToTarget()== null){
                return null;
            }
            return LCamera.getLatestResult().getBestTarget().getBestCameraToTarget().plus(VisionConstants.kRobotToLCam.inverse());
        }else return null;
    }

    @Override
    public Transform3d getTargetRPose(){
        if (RCamera.getLatestResult().hasTargets()){
            return RCamera.getLatestResult().getBestTarget().getBestCameraToTarget().plus(VisionConstants.kRobotToRCam.inverse());
        
        }else return null;
    }

    // @Override
    // public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
    //     var estStdDevs = VisionConstants.kSingleTagStdDevs;
    //     var targets = getLatestLResult().getTargets();
    //     int numTags = 0; // tag counter that counts all tags that are within the filter;
    //     double avgDist = 0;
    //     for (var tgt : targets) {
    //         var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //         if (tagPose.isEmpty()) continue; 
    //         // if (angFilter(totalTags)) continue;
    //         numTags++;
    //         // if(tgt.getFiducialId() != 4 || tgt.getFiducialId() != 7) continue;
    //         avgDist +=
    //             tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    //     }
    //     if (numTags == 0) return estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //     // estStdDevs;
    //     avgDist /= numTags;
    //     // Decrease std devs if multiple targets are visible
    //     if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    //     // Increase std devs based on (average) distance
    //     if (numTags == 1 && avgDist > 4)
    //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //     if (avgDist > 6)
    //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //     else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    //     // getLatestResult(th).getBestTarget().getPoseAmbiguity();
    //     return estStdDevs;
    // }

    @Override// need to figure out how to get it to actually work with 2
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        if (LCamera.getLatestResult().hasTargets()){
            targets = getLatestLResult().getTargets();
        }else if (RCamera.getLatestResult().hasTargets()){
            targets = getLatestRResult().getTargets();
        }else{
            return estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue; 
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

    public boolean angFilter(List<PhotonTrackedTarget> targets, int TagNum){// tag num is the index number for the target Table
        return (new Rotation2d(Math.toRadians(180)).plus(targets.get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() > 45
             || new Rotation2d(Math.toRadians(180)).plus(targets.get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() < -45
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.CameraPose = getEstimatedGlobalPose();
        if (LCamera.getLatestResult().hasTargets()){
            inputs.LTargetPose = getTargetLPose();
        }
        inputs.LTarget = LCamera.getLatestResult().hasTargets(); 
          
        if (RCamera.getLatestResult().hasTargets()){
            inputs.RTargetPose = getTargetRPose();
        }
        inputs.RTarget = RCamera.getLatestResult().hasTargets();   
        
    }

       
}