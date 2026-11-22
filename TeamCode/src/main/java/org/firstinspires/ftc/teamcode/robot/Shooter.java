package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
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
    private static double integralSum = 0;
    private static double lastError = 0;

    public enum shooterSpeeds {
        CLOSE_RPM(1000.0), // need to find
        STRAIGHT_RPM(1700.0), // need to find
        CORNERS_RPM(2500); // need to find

        public final double speeds;
        shooterSpeeds(double speeds) {
            this.speeds = speeds;
        }
    }

    private enum powerState {
        RUN(1.0),
        NOT_RUN(0.0),
        RUN_SHOOTER(0.5),
        REVERSE(-1.0);

        private final double power;
        powerState(double power) {
            this.power = power;
        }
    }

    public static void init(HardwareMap hwMap) {
        shooterTop = hwMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hwMap.get(DcMotorEx.class, "shooterBottom");

        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterTop.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

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
}