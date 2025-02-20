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

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class SUB_Vision {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    // private Matrix<N3, N1> curStdDevs;
    private List<AprilTag> invertedTags;


    public SUB_Vision(VisionIO io) {
        this.io = io;
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
        return io.getLEstimationStdDevs(estimatedPose);
    }

    public Matrix<N3, N1> getREstimationStdDevs(Pose2d estimatedPose) {
        return io.getREstimationStdDevs(estimatedPose);
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

    public Transform3d getTargetLPose(){
        return io.getTargetLPose().plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToLCam.getX(), 0, -VisionConstants.kRobotToLCam.getZ()), VisionConstants.kRobotToLCam.getRotation()));
    }

    public Transform3d getTargetRPose(){
        return io.getTargetRPose().plus(new Transform3d (new Translation3d(-VisionConstants.kRobotToRCam.getX(), 0, -VisionConstants.kRobotToRCam.getZ()), VisionConstants.kRobotToRCam.getRotation()));
    }
}