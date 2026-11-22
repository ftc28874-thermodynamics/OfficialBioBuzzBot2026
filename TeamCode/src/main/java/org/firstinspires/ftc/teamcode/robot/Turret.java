package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private static Servo turretServo = null;
    private static double turretPosition = 0.5;
    private static boolean prevLeftBumper = false;
    private static boolean prevRightBumper = false;
    private static int activeDirection = 0;

    public static void init(HardwareMap hwMap) {
        turretServo = hwMap.get(Servo.class, "turretServo");
        turretServo.setPosition(turretPosition); // Centers turret on init
    }

    public static double getTurretPos() {
        return turretServo.getPosition();
    }

    public static void turnTurretDirection(boolean right, double turretSpeed) {
        if (right) {
            turretPosition += turretSpeed;
        } else {
            turretPosition -= turretSpeed;
        }

        turretPosition = Math.max(0.0, Math.min(1.0, turretPosition));
        turretServo.setPosition(turretPosition);
    }

    // Calling this in your OpMode will look like:
    // Turret.updateTurret(gamepad1.left_bumper, gamepad1.right_bumper, 0.005);
    public static void updateTurret(boolean leftPressed, boolean rightPressed, double turretSpeed) {
        // Checks if right bumper is pressed right now AND was NOT pressed in the last loop
        if (rightPressed && !prevRightBumper) {
            activeDirection = 1;
        } else if (leftPressed && !prevLeftBumper) {
            activeDirection = -1;
        }

        // Checks if the turret was moving right, but driver just let go of the right bumper
        if (activeDirection == 1 && !rightPressed) {
            activeDirection = leftPressed ? -1 : 0;
        } else if (activeDirection == -1 && !leftPressed) {
            activeDirection = rightPressed ? 1 : 0;
        }

        // Moves servo right or left
        if (activeDirection == 1) {
            turnTurretDirection(true, turretSpeed);
        } else if (activeDirection == -1) {
            turnTurretDirection(false, turretSpeed);
        }

        // Saves old vars for next loop
        prevLeftBumper = leftPressed;
        prevRightBumper = rightPressed;
    }
}