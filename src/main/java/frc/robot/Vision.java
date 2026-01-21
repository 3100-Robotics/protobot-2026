package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private Supplier<Pose2d> robotPoseSupplier;

    private static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Front Camera
    private PhotonCamera cameraf = new PhotonCamera("photonvisionf");
    Translation3d robotToCamerafTrl = new Translation3d(1, 0, 1);
    Rotation3d robotToCamerafRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCameraf = new Transform3d(robotToCamerafTrl, robotToCamerafRot);

    // Low Camera
    private PhotonCamera cameral = new PhotonCamera("photonvisionl");
    Translation3d robotToCameralTrl = new Translation3d(1, 0, 0);
    Rotation3d robotToCameralRot = new Rotation3d(0, Math.toRadians(4), 0);
    Transform3d robotToCameral = new Transform3d(robotToCameralTrl, robotToCameralRot);

    // Simulation
    private VisionSystemSim visionSim;
        // Front Camera
        SimCameraProperties camerafProp;
        // Low Camera
        SimCameraProperties cameralProp;

    public Vision(boolean is_simulation, Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        if (is_simulation) {
            visionSim = new VisionSystemSim("simtags");
            visionSim.addAprilTags(tagLayout);  

            // Front Camera
            camerafProp = new SimCameraProperties();
            camerafProp.setCalibration(640, 480, Rotation2d.fromDegrees(50));
            camerafProp.setCalibError(0.25, 0.08);
            camerafProp.setFPS(20);
            camerafProp.setAvgLatencyMs(35);
            camerafProp.setLatencyStdDevMs(5);

            PhotonCameraSim camerafSim = new PhotonCameraSim(cameraf, camerafProp);
            visionSim.addCamera(camerafSim, robotToCameraf);

            // Low Camera
            cameralProp = new SimCameraProperties();
            cameralProp.setCalibration(640, 480, Rotation2d.fromDegrees(50));
            cameralProp.setCalibError(0.25, 0.08);
            cameralProp.setFPS(20);
            cameralProp.setAvgLatencyMs(35);
            cameralProp.setLatencyStdDevMs(5);

            PhotonCameraSim cameralSim = new PhotonCameraSim(cameral, cameralProp);
            visionSim.addCamera(cameralSim, robotToCameral);
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(robotPoseSupplier.get());
    }

    @Override
    public void periodic() {
        // var result = cameraf.getLatestResult();
        // boolean hasTargets = result.hasTargets();
        // if (hasTargets) {
        //     List<PhotonTrackedTarget> targets = result.getTargets();
        //     for (int i = 0; i < targets.size(); i++) {
        //         PhotonTrackedTarget target = targets.get(i);
        //         int targetID = target.getFiducialId();
        //         double poseAmbiguity = target.getPoseAmbiguity();
        //         Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        //         Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        //         SmartDashboard.putString("tag " + targetID, String.valueOf(poseAmbiguity));
        //         atag_field.getObject(String.valueOf(targetID)).setPose(
        //             new Pose2d(bestCameraToTarget.getX(), bestCameraToTarget.getY(), bestCameraToTarget.getRotation().toRotation2d())
        //         );
        //     }
        // }
    }
}
