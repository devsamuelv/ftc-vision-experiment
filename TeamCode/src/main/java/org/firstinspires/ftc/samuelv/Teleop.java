package org.firstinspires.ftc.samuelv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp(name = "Apriltag Pose Estimation Teleop")
public class Teleop extends LinearOpMode {
    DeviceMap device_map;
    VisionPortal portal_one, portal_two;
    AprilTagProcessor processor_one, processor_two;

    Position camera_one_position = new Position();
    Position camera_two_position = new Position();
    Pose3D robot_pose = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0,0,0));
    YawPitchRollAngles camera_one_orientation = new YawPitchRollAngles(AngleUnit.RADIANS,0, 0, 0, 0);
    YawPitchRollAngles camera_two_orientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0,0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        device_map = new DeviceMap(hardwareMap);

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        processor_one = new AprilTagProcessor.Builder()
                .setCameraPose(camera_one_position, camera_one_orientation)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        processor_two = new AprilTagProcessor.Builder()
                .setCameraPose(camera_two_position, camera_two_orientation)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal.Builder portal_builder_one = new VisionPortal.Builder();
        portal_builder_one.setCamera(device_map.camera_one);
        portal_builder_one.setLiveViewContainerId(portal1ViewId);
        portal_builder_one.addProcessor(processor_one);

        portal_one = portal_builder_one.build();

        VisionPortal.Builder portal_builder_two = new VisionPortal.Builder();
        portal_builder_two.setCamera(device_map.camera_two);
        portal_builder_two.setLiveViewContainerId(portal2ViewId);
        portal_builder_two.addProcessor(processor_two);

        portal_two = portal_builder_two.build();

        waitForStart();

        while (!isStopRequested()) {
            // Teleop
            ArrayList<AprilTagDetection> detections_one = processor_one.getDetections();
            ArrayList<AprilTagDetection> detections_two = processor_two.getDetections();
            double robot_x_accum = 0;
            double robot_y_accum = 0;

            for (AprilTagDetection detection: detections_one) {
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;

                robot_x_accum += x;
                robot_y_accum += y;
            }

            for (AprilTagDetection detection: detections_two) {
                double x = detection.robotPose.getPosition().x;
                double y = detection.robotPose.getPosition().y;

                robot_x_accum += x;
                robot_y_accum += y;
            }

            double robot_x_average = robot_x_accum / (detections_one.size() + detections_two.size());
            double robot_y_average = robot_y_accum / (detections_one.size() + detections_two.size());

            Pose2D robot_pose_estimated = new Pose2D(DistanceUnit.METER, robot_x_average, robot_y_average, AngleUnit.RADIANS, 0);

            telemetry.addData("Estimated Robot X", robot_pose_estimated.getX(DistanceUnit.METER));
            telemetry.addData("Estimated Robot Y", robot_pose_estimated.getY(DistanceUnit.METER));
            telemetry.update();
        }
    }
}
