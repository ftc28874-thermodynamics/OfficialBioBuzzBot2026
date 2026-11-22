package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.visionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.teamcode.robot.visionManager.AllianceColor;

@TeleOp(name = "Camera Test")
public class visionTest extends LinearOpMode {

    private visionManager vision = new visionManager();
    private ServoImplEx turretServo1;
    private ServoImplEx turretServo2;

    @Override
    public void runOpMode() {
        vision.init(hardwareMap, "Webcam 1");

        // init servos
        turretServo1 = hardwareMap.get(ServoImplEx.class, "turretServo1");
        turretServo2 = hardwareMap.get(ServoImplEx.class, "turretServo2");
        // axons use a diff pwm range so this is slightly more accurate than the default
        turretServo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // centers both servos on init
        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            // gets updated position to center on the red hive
            double pos = vision.getCenteredTurretPosition(AllianceColor.RED);
            turretServo1.setPosition(pos);
            turretServo2.setPosition(pos);


            boolean tipped = vision.isHiveTipped(AllianceColor.RED);
            AprilTagPoseFtc pose = vision.trackHive(AllianceColor.RED);

            if (pose != null) {
                telemetry.addData("status", tipped ? "tipped" : "centering");
                telemetry.addData("bearing", "%.2f", pose.bearing);
                telemetry.addData("position", "%.3f", pos);
            } else {
                telemetry.addLine("red hive not in frame");
            }

            telemetry.update();
        }

        vision.close();
    }
}