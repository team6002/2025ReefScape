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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private final PhotonPoseEstimator LphotonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLCam);
    private final PhotonPoseEstimator LphotonEstimatorLast = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, VisionConstants.kRobotToLCam);
    private final PhotonPoseEstimator RphotonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRCam);
    private final PhotonPoseEstimator RphotonEstimatorLast = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, VisionConstants.kRobotToRCam);
    private Matrix<N3, N1> curStdDevs;

    public void setCameraPipeline(int LPipeline, int RPipeline){
        LCamera.setPipelineIndex(LPipeline);
        RCamera.setPipelineIndex(RPipeline);
    }

    @Override
    public void setMultiTagFallbackStrategy(PoseStrategy poseStrategy){
        LphotonEstimator.setMultiTagFallbackStrategy(poseStrategy);
    }
    
    public void setCameraDriverMode(boolean bootlean){
        LCamera.setDriverMode(bootlean);
        RCamera.setDriverMode(bootlean);
    }

    @Override 
    public Optional<EstimatedRobotPose> getLEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : LCamera.getAllUnreadResults()) {
            visionEst = LphotonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }
    
    public Optional<EstimatedRobotPose> getLEstimatedGlobalPoseLast() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : LCamera.getAllUnreadResults()) {
            visionEst = LphotonEstimatorLast.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    @Override 
    public Optional<EstimatedRobotPose> getREstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : RCamera.getAllUnreadResults()) {
            visionEst = RphotonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }
    
    public Optional<EstimatedRobotPose> getREstimatedGlobalPoseLast() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : RCamera.getAllUnreadResults()) {
            visionEst = RphotonEstimatorLast.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    @Override
    public void setLastLPose(Pose2d lastpose){
        LphotonEstimatorLast.setLastPose(lastpose);
    }

    @Override
    public void setLastRPose(Pose2d lastpose){
        RphotonEstimatorLast.setLastPose(lastpose);
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
    public Pose2d getCurrentLPose(){
        int tagNum = LCamera.getLatestResult().getBestTarget().getFiducialId();
        for (PhotonTrackedTarget target : LCamera.getLatestResult().targets){
            if (Math.abs(target.getYaw()) <= 25){
                continue;
            }
            tagNum = target.getFiducialId();
        }
        Pose2d tagLocation = new Pose2d();
        tagLocation = new Pose2d(VisionConstants.kTagLayout.getTagPose(tagNum).get().getX(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getY(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getRotation().toRotation2d());
        Pose2d currentPose = PhotonUtils.estimateFieldToRobot(
            new Transform2d(getTargetLPose().getTranslation().toTranslation2d(), getTargetLPose().getRotation().toRotation2d())
            , tagLocation
            , new Transform2d(VisionConstants.kRobotToLCam.inverse().getTranslation().toTranslation2d(), VisionConstants.kRobotToLCam.getRotation().toRotation2d().unaryMinus())
            );
        return currentPose;
    }
    
    @Override
    public Pose2d getCurrentRPose(){
        int tagNum = RCamera.getLatestResult().getBestTarget().getFiducialId();
        for (PhotonTrackedTarget target : RCamera.getLatestResult().targets){
            if (Math.abs(target.getYaw()) <= 25){
                continue;
            }
            tagNum = target.getFiducialId();
        }
        Pose2d tagLocation = new Pose2d();
        tagLocation = new Pose2d(VisionConstants.kTagLayout.getTagPose(tagNum).get().getX(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getY(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getRotation().toRotation2d());
        Pose2d currentPose = PhotonUtils.estimateFieldToRobot(
            new Transform2d(getTargetRPose().getTranslation().toTranslation2d(), getTargetRPose().getRotation().toRotation2d())
            , tagLocation
            , new Transform2d(VisionConstants.kRobotToRCam.inverse().getTranslation().toTranslation2d(), VisionConstants.kRobotToRCam.getRotation().toRotation2d().unaryMinus())
            );
        return currentPose;
    }

    @Override
    public Transform3d getTargetLPose(){
        if (LCamera.getLatestResult().hasTargets()){
            if (LCamera.getLatestResult().getBestTarget().getBestCameraToTarget()== null){
                return null;
            }
            return LCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
        }else return null;
    }

    @Override
    public Transform3d getTargetRPose(){
        if (RCamera.getLatestResult().hasTargets()){
            return RCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
        
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
    public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        targets = getLatestLResult().getTargets();
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = LphotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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

    @Override// need to figure out how to get it to actually work with 2
    public Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        targets = getLatestRResult().getTargets();
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = LphotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
            inputs.LTargetPose = getTargetLPose().plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToLCam.getX(), 0.0, -VisionConstants.kRobotToLCam.getZ()), VisionConstants.kRobotToLCam.getRotation()));
        }
        inputs.LTarget = LCamera.getLatestResult().hasTargets(); 
          
        if (RCamera.getLatestResult().hasTargets()){
            inputs.RTargetPose = getTargetRPose().plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToRCam.getX(), 0, -VisionConstants.kRobotToRCam.getZ()), VisionConstants.kRobotToRCam.getRotation()));
        }
        inputs.RTarget = RCamera.getLatestResult().hasTargets();   
        
    }

       
}