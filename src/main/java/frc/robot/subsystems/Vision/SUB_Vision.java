/*
* MIT License
*
* Copyright (c) PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class SUB_Vision {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public SUB_Vision(VisionIO io) {
        this.io = io;
    }

    public double getTcameraYaw(){
        return io.getTcameraYaw();
    }

    public boolean getHasLTarget(){
        return inputs.LTarget;
    }

    public boolean getHasRTarget(){
        return inputs.RTarget;
    }

    // public Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {
    //     return io.getREstimationStdDevs(estimatedPose);
    // }

    // public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
    //     return io.getLEstimationStdDevs(estimatedPose);
    // }

    public Matrix<N3, N1> getLEstimationStdDevs(Pose2d estimatedPose) {
        try{
        // if (estimatedPose.equals(estimatedPose)){
        //     return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // }
        return io.getLEstimationStdDevs(estimatedPose);
        } catch (Exception e){
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
    }

    public Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {
        try{
        // if (estimatedPose.equals(estimatedPose)){
        //     return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // }
        return io.getREstimationStdDevs(estimatedPose);
        } catch (Exception e){
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
    }

    public void updateInputs(){
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
    
    public Optional<EstimatedRobotPose> getLEstimatedGlobalPose() {
        return io.getLEstimatedGlobalPose();
    }

    public Optional<EstimatedRobotPose> getLEstimatedGlobalPoseLast() {
        return io.getLEstimatedGlobalPoseLast();
    }

    public Optional<EstimatedRobotPose> getREstimatedGlobalPose() {
        return io.getREstimatedGlobalPose();
    }

    public Optional<EstimatedRobotPose> getREstimatedGlobalPoseLast() {
        return io.getREstimatedGlobalPoseLast();
    }

    public void setLastLPose(Pose2d lastPose){
        io.setLastLPose(lastPose);
    }

    public void setLastRPose(Pose2d lastPose){
        io.setLastRPose(lastPose);
    }

    public Pose2d getCurrentLPose(){
        return io.getCurrentLPose();
    }

    public Pose2d getCurrentRPose(){
        return io.getCurrentRPose();
    }

    public Pose2d getTargetLPose(){
        return io.getTargetLPose();
        // .plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToLCam.getX(), -VisionConstants.kRobotToLCam.getY(), -VisionConstants.kRobotToLCam.getZ()), VisionConstants.kRobotToLCam.getRotation()));
    }

    public Pose2d getTargetRPose(){
        return io.getTargetRPose();
        // .plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToRCam.getX(), -VisionConstants.kRobotToRCam.getY(), -VisionConstants.kRobotToRCam.getZ()), VisionConstants.kRobotToRCam.getRotation()));
    }

    public void setRobotRotation(Rotation2d robotRotation){
        io.setRobotRotation(robotRotation);
    }
    public Pose3d getLPose(Pose2d actualRobotPose){
        var possiblePoses = io.retrieveSingleTagEstimates(io.getLCamResult(), VisionConstants.kRobotToLCam.inverse());
        var ClosestPose = selectClosestPose(possiblePoses, actualRobotPose);
        return ClosestPose;
    } 

    public Pose3d getRPose(Pose2d actualRobotPose){
        var possiblePoses = io.retrieveSingleTagEstimates(io.getRCamResult(), VisionConstants.kRobotToRCam.inverse());
        var ClosestPose = selectClosestPose(possiblePoses, actualRobotPose);
        return ClosestPose;
    } 

    private Pose3d selectClosestPose(Pose3d[] fieldSpaceRobotPoses, Pose2d actualRobotPose) {
        // Get the distance closest to the current robot pose if multiple poses are determined for a
        // camera
        double minDistance = Double.POSITIVE_INFINITY;
        Pose3d selectedPose = null;
        Pose3d actualRobotPose3d = new Pose3d(actualRobotPose);

        for (Pose3d fieldSpaceRobotPose : fieldSpaceRobotPoses) {
            double distance = fieldSpaceRobotPose.getTranslation().getDistance(actualRobotPose3d.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                selectedPose = fieldSpaceRobotPose;
            }
        }
        return selectedPose;
    }
    // public double getLLatency(){
    //     // return
    // }
}