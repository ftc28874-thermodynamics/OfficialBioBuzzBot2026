package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class visionManager {

    public enum AllianceColor {
        RED, BLUE
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // swyft ov9281 camera calibration values
    private static final int CAM_WIDTH = 1280;
    private static final int CAM_HEIGHT = 800;
    private static final double FX = 799.494952;
    private static final double FY = 799.832060;
    private static final double CX = 644.011016;
    private static final double CY = 447.804766;

    // simple turret step and deadband settings
    private static final double STEP_SIZE = 0.005;
    private static final double CENTER_TOLERANCE_DEG = 2.0;
    private double turretPosition = 0.5;

    // initializes the vision portal and apriltag processor
    public void init(HardwareMap hardwareMap, String webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setLensIntrinsics(FX, FY, CX, CY)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(CAM_WIDTH, CAM_HEIGHT))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // finds the best visible cluster for the given alliance color
    private AprilTagClusterDetection getAllianceCluster(AllianceColor targetAlliance) {
        if (aprilTagProcessor == null) return null;

        AprilTagClusterDetection bestCluster = null;

        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

                if (cluster.metadata != null && cluster.metadata.name != null) {
                    String name = cluster.metadata.name.toUpperCase();
                    boolean matchesColor = (targetAlliance == AllianceColor.RED && name.contains("RED")) ||
                            (targetAlliance == AllianceColor.BLUE && name.contains("BLUE"));

                    if (matchesColor) {
                        if (bestCluster == null || cluster.percentClusterFound > bestCluster.percentClusterFound) {
                            bestCluster = cluster;
                        }
                    }
                }
            }
        }
        return bestCluster;
    }

    // returns true if the hive is tipped, false otherwise
    public boolean isHiveTipped(AllianceColor targetAlliance) {
        AprilTagClusterDetection cluster = getAllianceCluster(targetAlliance);
        if (cluster == null) {
            return false;
        }
        return Math.abs(cluster.ftcPose.roll) >= 90.0;
    }

    // returns the tracking pose for the hive
    public AprilTagPoseFtc trackHive(AllianceColor targetAlliance) {
        AprilTagClusterDetection cluster = getAllianceCluster(targetAlliance);
        if (cluster == null) {
            return null;
        }
        return cluster.ftcPose;
    }

    // steps the turret opposite the tag offset until centered
    public double getCenteredTurretPosition(AllianceColor targetAlliance) {
        AprilTagPoseFtc pose = trackHive(targetAlliance);
        boolean tipped = isHiveTipped(targetAlliance);

        if (pose != null && !tipped) {
            if (pose.bearing > CENTER_TOLERANCE_DEG) {
                turretPosition -= STEP_SIZE;
            } else if (pose.bearing < -CENTER_TOLERANCE_DEG) {
                turretPosition += STEP_SIZE;
            }
            turretPosition = Range.clip(turretPosition, 0.0, 1.0);
        }

        return turretPosition;
    }

    // closes the vision portal
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}