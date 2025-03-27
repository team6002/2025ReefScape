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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
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
    private final PhotonCamera TCamera = new PhotonCamera(VisionConstants.kTopCameraName);
    private final PhotonPoseEstimator LphotonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLCam);
    private final PhotonPoseEstimator LphotonEstimatorLast = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, VisionConstants.kRobotToLCam);
    private final PhotonPoseEstimator RphotonEstimator = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRCam);
    private final PhotonPoseEstimator RphotonEstimatorLast = 
        new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, VisionConstants.kRobotToRCam);

    TimeInterpolatableBuffer<Rotation2d> rotationBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    public void setCameraPipeline(int LPipeline, int RPipeline){
        LCamera.setPipelineIndex(LPipeline);
        RCamera.setPipelineIndex(RPipeline);
    }

    public void setTCameraPipeline(int TPipeline){
        TCamera.setPipelineIndex(TPipeline);
    }

    @Override
    public double getTcameraYaw(){
        try {
            return TCamera.getLatestResult().getBestTarget().getYaw();    
        } catch (Exception e) {
            return Double.MAX_VALUE;
        }
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

    // @Override
    // public Pose2d getCurrentLPose(){
    //     int tagNum = LCamera.getLatestResult().getBestTarget().getFiducialId();
    //     for (PhotonTrackedTarget target : LCamera.getLatestResult().targets){
    //         if (Math.abs(target.getYaw()) <= 25){
    //             continue;
    //         }
    //         tagNum = target.getFiducialId();
    //     }
    //     Pose2d tagLocation = new Pose2d();
    //     tagLocation = new Pose2d(VisionConstants.kTagLayout.getTagPose(tagNum).get().getX(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getY(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getRotation().toRotation2d());
    //     Pose2d currentPose = PhotonUtils.estimateFieldToRobot(
    //         new Transform2d(getTargetLPose().getTranslation().toTranslation2d(), getTargetLPose().getRotation().toRotation2d())
    //         , tagLocation
    //         , new Transform2d(VisionConstants.kRobotToLCam.inverse().getTranslation().toTranslation2d(), VisionConstants.kRobotToLCam.getRotation().toRotation2d().unaryMinus())
    //         );
    //     return currentPose;
    // }
    
    // @Override
    // public Pose2d getCurrentRPose(){
    //     int tagNum = RCamera.getLatestResult().getBestTarget().getFiducialId();
    //     for (PhotonTrackedTarget target : RCamera.getLatestResult().targets){
    //         if (Math.abs(target.getYaw()) <= 25){
    //             continue;
    //         }
    //         tagNum = target.getFiducialId();
    //     }
    //     Pose2d tagLocation = new Pose2d();
    //     tagLocation = new Pose2d(VisionConstants.kTagLayout.getTagPose(tagNum).get().getX(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getY(), VisionConstants.kTagLayout.getTagPose(tagNum).get().getRotation().toRotation2d());
    //     Pose2d currentPose = PhotonUtils.estimateFieldToRobot(
    //         new Transform2d(getTargetRPose().getTranslation().toTranslation2d(), getTargetRPose().getRotation().toRotation2d())
    //         , tagLocation
    //         , new Transform2d(VisionConstants.kRobotToRCam.inverse().getTranslation().toTranslation2d(), VisionConstants.kRobotToRCam.getRotation().toRotation2d().unaryMinus())
    //         );
    //     return currentPose;
    // }

    @Override
    public Pose2d getTargetLPose(){
        try {
            if (LCamera.getLatestResult().hasTargets()){
                if (LCamera.getLatestResult().hasTargets()){
                    var rawTranslation = LCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
                    var RotatedPose = new Pose3d(rawTranslation.getTranslation().getX(), rawTranslation.getY() , rawTranslation.getZ(), rawTranslation.getRotation()).rotateBy(VisionConstants.kRobotToLCam.getRotation());
                    var ProcessedPose3d = new Pose3d(RotatedPose.getTranslation().plus(VisionConstants.kRobotToLCam.getTranslation()), RotatedPose.getRotation());
                    var targetPose2d = ProcessedPose3d;
                    // .rotateBy(VisionConstants.kRobotToLCam.getRotation());
                    //  new Pose2d(rawPose.getTranslation().toTranslation2d().minus(VisionConstants.kRobotToLCam.getTranslation().toTranslation2d()), rawPose.getRotation().toRotation2d().plus(VisionConstants.kRobotToLCam.getRotation().toRotation2d()));
                    return targetPose2d.toPose2d();
                
                }else return null;
            }else return null;
        } catch (Exception e){
            return null;
        }
    }

    @Override
    public Pose2d getTargetRPose(){
        try{
            if (RCamera.getLatestResult().hasTargets()){
                var rawTranslation = RCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
                    var RotatedPose = new Pose3d(rawTranslation.getTranslation().getX(), rawTranslation.getY() , rawTranslation.getZ(), rawTranslation.getRotation()).rotateBy(VisionConstants.kRobotToRCam.getRotation());
                    var ProcessedPose3d = new Pose3d(RotatedPose.getTranslation().plus(VisionConstants.kRobotToRCam.getTranslation()), RotatedPose.getRotation());
                    var targetPose2d = ProcessedPose3d;
                    // .rotateBy(VisionConstants.kRobotToRCam.getRotation());
                //  new Pose2d(rawPose.getTranslation().toTranslation2d().minus(VisionConstants.kRobotToLCam.getTranslation().toTranslation2d()), rawPose.getRotation().toRotation2d().plus(VisionConstants.kRobotToLCam.getRotation().toRotation2d()));
                return targetPose2d.toPose2d();
            }else return null;
        } catch (Exception e){
            return null;
        }
    }

    @Override// need to figure out how to get it to actually work with 2
    public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        targets = getLatestLResult().getTargets();
        int totalTags = -1; // a tag counter that counts all of the tags
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = LphotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue; 
            if (angFilter(targets, totalTags)) continue;
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
        if (numTags == 1 && avgDist > 3)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (avgDist > 3)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1);

        // getLatestResult(th).getBestTarget().getPoseAmbiguity();
        return estStdDevs;
    }

    // @Override// need to figure out how to get it to actually work with 2
    // public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
    //     var estStdDevs = VisionConstants.kSingleTagStdDevs;
    //     List<PhotonTrackedTarget> targets = new ArrayList<>();
    //     targets = getLatestLResult().getTargets();
    //     int numTags = 0; // tag counter that counts all tags that are within the filter;
    //     double avgDist = 0;
    //     for (var tgt : targets) {
    //         var tagPose = LphotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
    public Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        targets = getLatestRResult().getTargets();
        int totalTags = -1; // a tag counter that counts all of the tags
        int numTags = 0; // tag counter that counts all tags that are within the filter;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = RphotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue; 
            if (angFilter(targets, totalTags)) continue;
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
        if (numTags == 1 && avgDist > 3)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (avgDist > 3)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1);

        // getLatestResult(th).getBestTarget().getPoseAmbiguity();
        return estStdDevs;
    }

    public boolean angFilter(List<PhotonTrackedTarget> targets, int TagNum){// tag num is the index number for the target Table
        return (new Rotation2d(Math.toRadians(180)).plus(targets.get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() > 60
             || new Rotation2d(Math.toRadians(180)).plus(targets.get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() < -60
        );
    }

    // public boolean angFilterSTD(int TagNum){// tag num is the index number for the target Table
    //     return (new Rotation2d(Math.toRadians(180)).plus(getLatestResult().getTargets().get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() > 45
    //          || new Rotation2d(Math.toRadians(180)).plus(getLatestResult().getTargets().get(TagNum).getBestCameraToTarget().getRotation().toRotation2d()).getDegrees() < -45);
    // }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // inputs.CameraPose = getEstimatedGlobalPose();
        // if (getTcameraYaw == Double.MAX_VALUE){
        inputs.TCameraYaw = getTcameraYaw();
        // }
        if (LCamera.getLatestResult().hasTargets()){
            inputs.LTargetPose = getTargetLPose();//.plus(VisionConstants.kRobotToLCam).inverse();
            // .plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToLCam.getX(), -VisionConstants.kRobotToLCam.getY(), VisionConstants.kRobotToLCam.getZ()), VisionConstants.kRobotToLCam.getRotation()));
        }
        inputs.LTarget = LCamera.getLatestResult().hasTargets(); 
          
        if (RCamera.getLatestResult().hasTargets()){
            inputs.RTargetPose = getTargetRPose();//.plus(VisionConstants.kRobotToRCam).inverse();
            // .plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToRCam.getX(), -VisionConstants.kRobotToRCam.getY(), VisionConstants.kRobotToRCam.getZ()), VisionConstants.kRobotToRCam.getRotation()));
        }
        inputs.RTarget = RCamera.getLatestResult().hasTargets();   
        
    }

    public PhotonPipelineResult getLCamResult(){
        return LCamera.getLatestResult();
    }

    public PhotonPipelineResult getRCamResult(){
        return RCamera.getLatestResult();
    }
     /**
     * Pass the robot rotation that is measured with the IMU to the vision system This should be updated every loop
     *
     * @param robotRotation Actual rotation of the robot
     */
    @Override
    public void setRobotRotation(Rotation2d robotRotation) {
        // Put the rotation in a buffer
        rotationBuffer.addSample(RobotController.getFPGATime()-.02, robotRotation.plus(Rotation2d.k180deg));
    }

    /**
     * Determine several options for the robot pose in field space when only 1 tag is visible This algorithm will
     * discard the rotation of the tag and use the robot rotation instead for improved accuracy
     *
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    public Pose3d[] retrieveSingleTagEstimates(PhotonPipelineResult latestResult, Transform3d CameraToRobot) {
        Pose3d[] possibleRobotposes = new Pose3d[2];
        PhotonTrackedTarget target = latestResult.getBestTarget();

        // Only proceed if the target can be found in the april tag field layout
        // and the robot rotation can be retrieved at the time that the result was determined
        Optional<Pose3d> tagPoseOptional = VisionConstants.kTagLayout.getTagPose(target.getFiducialId());
        Optional<Rotation2d> robotRotationOptional = rotationBuffer.getSample(latestResult.getTimestampSeconds());
        if (tagPoseOptional.isPresent() && robotRotationOptional.isPresent()) {
            Pose3d tagPose = tagPoseOptional.get();
            Rotation3d tagRotation = tagPose.getRotation();
            Rotation2d robotRotation = robotRotationOptional.get();

            // Now convert robot rotation to Rotation3d and subtract it from the tag pose to get the
            // relative rotation between robot and target
            Rotation3d robotToTargetRot = tagRotation.minus(new Rotation3d(0, 0, robotRotation.getRadians()));
            // Now we can include the rotation between the camera and the robot
            Rotation3d cameraToTargetRot = robotToTargetRot.plus(CameraToRobot.inverse().getRotation());

            // Now we can combine the rotation of the robot with the translation determined by the
            // camera
            Transform3d[] camToTargetOptions = {
                new Transform3d(target.getBestCameraToTarget().getTranslation(), cameraToTargetRot),
                new Transform3d(target.getAlternateCameraToTarget().getTranslation(), cameraToTargetRot)
            };

            for (int i = 0; i < camToTargetOptions.length; i++) {
                Transform3d camToTarget = camToTargetOptions[i];
                possibleRobotposes[i] =
                        PhotonUtils.estimateFieldToRobotAprilTag(camToTarget, tagPose, CameraToRobot.inverse());
            }
            // logRotationDiff(tagPose.plus(target.getBestCameraToTarget().inverse()));
        }
        return possibleRobotposes;
    }

    /**
     * Determine several options for the robot pose in field space when multiple tags are visible
     *
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    public Pose3d[] retrieveMultiTagEstimates(PhotonPipelineResult latestResult, Transform3d CameraToRobot) {
        // Retrieve the camera pose in field space represented as transform
        Transform3d bestMultiPose = latestResult.getMultiTagResult().get().estimatedPose.best;
        Transform3d alternateMultiPose = latestResult.getMultiTagResult().get().estimatedPose.alt;

        // logRotationDiff(new Pose3d().transformBy(bestMultiPose));

        // Convert to field space robot pose and return
        return new Pose3d[] {
            new Pose3d().transformBy(bestMultiPose).transformBy(CameraToRobot),
            new Pose3d().transformBy(alternateMultiPose).transformBy(CameraToRobot)
        };
    }

    // @Override
    // public double getLLatency(){
    //     return LCamera.
    // } 
}