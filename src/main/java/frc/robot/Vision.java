package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public Field2d purevision = new Field2d();
    public StructPublisher<Pose3d> publish3d0 = NetworkTableInstance.getDefault()
        .getStructTopic("03d", Pose3d.struct).publish();

    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }



    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Transform3d robotToFrontRightCam =
        new Transform3d(
            new Translation3d(
                Inches.of(12.040283+-0.673182).in(Meters), 
                Inches.of(-10.040908+0.807295).in(Meters),
                Inches.of(6.805750+2.396424).in(Meters)
            ),
            new Rotation3d(0, Math.toRadians(-60), Math.toRadians(-65))
        );

    public static final Transform3d robotToFrontLeft =
        new Transform3d(
            new Translation3d(
                Inches.of(12.040283+-0.673182).in(Meters), 
                Inches.of(-10.040908-0.807295).in(Meters),
                Inches.of(6.805750+2.396424).in(Meters)
            ),
            new Rotation3d(0, Math.toRadians(-60), Math.toRadians(65))
        );

    public PhotonPoseEstimator photonEstimatorFrontRight;
    public PhotonPoseEstimator photonEstimatorFrontLeft;
    public PhotonCamera cameraFrontRight = new PhotonCamera("Right");
    public PhotonCamera cameraFrontLeft = new PhotonCamera("Left");

    public Vision(boolean is_simulation, EstimateConsumer estConsumer) {
        photonEstimatorFrontRight = new PhotonPoseEstimator(
            tagLayout,
            robotToFrontRightCam);

        this.estConsumer = estConsumer;
        if (is_simulation) {
            
        }
    }

    @Override
    public void simulationPeriodic() {
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(purevision);
        Optional<EstimatedRobotPose> visionEstLeft = Optional.empty();
        for (var result : cameraFrontRight.getAllUnreadResults()) {
            visionEstLeft = photonEstimatorFrontRight.estimateCoprocMultiTagPose(result);
            if (visionEstLeft.isEmpty()) {
                visionEstLeft = photonEstimatorFrontRight.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEstLeft, result.getTargets());

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            // }

            visionEstLeft.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        purevision.setRobotPose(est.estimatedPose.toPose2d());
                        
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }


        // Right Camera
        Optional<EstimatedRobotPose> visionEstRight = Optional.empty();
        for (var result : cameraFrontLeft.getAllUnreadResults()) {
            visionEstRight = photonEstimatorFrontRight.estimateCoprocMultiTagPose(result);
            if (visionEstRight.isEmpty()) {
                visionEstRight = photonEstimatorFrontRight.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEstRight, result.getTargets());

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            // }

            visionEstRight.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        purevision.getObject("RightCamera").setPose(est.estimatedPose.toPose2d());
                        publish3d0.set(est.estimatedPose);
                        
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimatorFrontRight.getFieldTags().getTagPose(tgt.getFiducialId());
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
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
}
