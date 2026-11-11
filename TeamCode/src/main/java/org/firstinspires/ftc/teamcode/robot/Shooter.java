package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    // shooter vars
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotorEx shooterTop = null;
    private static DcMotorEx shooterBottom = null;
    private static final double COUNTS_PER_REVOLUTION = 28;

    private static final double SHOOTER_RPM_HIGH = 6000.0;
    private static final double SHOOTER_RPM_LOW = 100.0;
    private static final double Kp = 0.001; // needs to be tuned
    private static final double Kd = 0.001; // needs to be tuned
    private static final double Ki = 0.001; // needs to be tuned
    private static final double Kf = 0.001; // needs to be tuned
    private static double  integralSum = 0;
    private static double lastError = 0;

    // turret vars
    private static Servo turretServo = null;
    private static double turretPosition = 0.5;
    private static boolean prevLeftBumper = false;
    private static boolean prevRightBumper = false;
    private static int activeDirection = 0;



    public static void init(HardwareMap hwMap) {
        shooterTop = hwMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hwMap.get(DcMotorEx.class, "shooterBottom");
        turretServo = hwMap.get(Servo.class, "turretServo");

        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterTop.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum shooterSpeeds {
        CLOSE_RPM(1000.0), // need to find
        STRAIGHT_RPM(1700.0), // need to find

        CORNERS_RPM(2500); // need to find

        public final double speeds;
        shooterSpeeds(double speeds) {this.speeds = speeds;}

    }

    private enum powerState {
        RUN(1.0),
        NOT_RUN(0.0),
        RUN_SHOOTER(0.5),
        REVERSE(-1.0);
        private final double power;
        powerState(double power){this.power = power;}
    }

    public static double getTurretPos() {return turretServo.getPosition();}

    public static double PIDControl(double reference, double state) {

        double error = reference - state;


        double dt = runtime.seconds();
        runtime.reset();


        if (Math.abs(error) < 100.0) {
            integralSum += error * dt;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;


        double output = (error * Kp) +
                (integralSum * Ki) +
                (derivative * Kd) +
                (reference * Kf);


        return Range.clip(output, -1.0, 1.0);
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

    public static void updateTurret(boolean leftPressed, boolean rightPressed, double turretSpeed) { // calling this will look sm like Shooter.updateTurret(gamepad1.left_bumper, gamepad1.right_bumper, 0.005);

        // checks if right bumper is pressed right now AND was NOT pressed in the last loop
        if (rightPressed && !prevRightBumper) {
            activeDirection = 1;
            // the opposite of the line above this
        } else if (leftPressed && !prevLeftBumper) {
            activeDirection = -1;
        }
           // checks if the turret was moving right, but driver js let go of the right bumper.
        if (activeDirection == 1 && !rightPressed) {
            activeDirection = leftPressed ? -1 : 0;
          // opposite of the line above
        } else if (activeDirection == -1 && !leftPressed) {
            activeDirection = rightPressed ? 1 : 0;
        }
          // calls the turnTurretDirection thing to night servo right or left
        if (activeDirection == 1) {
            turnTurretDirection(true, turretSpeed);
        } else if (activeDirection == -1) {
            turnTurretDirection(false, turretSpeed);
        }
          // saves old vars for next loop
        prevLeftBumper = leftPressed;
        prevRightBumper = rightPressed;
    }

}
