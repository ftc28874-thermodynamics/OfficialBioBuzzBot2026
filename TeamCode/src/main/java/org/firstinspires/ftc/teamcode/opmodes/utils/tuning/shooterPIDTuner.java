package org.firstinspires.ftc.teamcode.opmodes.utils.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter PID Tuner")
public class shooterPIDTuner extends LinearOpMode {

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private ElapsedTime timer = new ElapsedTime();

    private static final double TARGET_VELOCITY = 2200.0;
    private static final double ALLOWABLE_ERROR = 35.0;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Dual Shooter Tuner Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setPower(1.0);
        shooter2.setPower(1.0);

        timer.reset();
        double maxVelocity = 0.0;

        while (opModeIsActive() && timer.seconds() < 3.5) {
            double currentVel = Math.abs(shooter1.getVelocity());
            if (timer.seconds() > 1.5) {
                maxVelocity = Math.max(maxVelocity, currentVel);
            }
            telemetry.addData("Current Velocity", currentVel);
            telemetry.addData("Peak Velocity", maxVelocity);
            telemetry.update();
        }

        if (maxVelocity <= 100) {
            shooter1.setPower(0);
            shooter2.setPower(0);
            telemetry.addLine("Error: Low velocity reading.");
            telemetry.update();
            return;
        }

        double calculatedF = 32767.0 / maxVelocity;
        double calculatedP = 0.1 * calculatedF;
        double calculatedI = 0.0;
        double calculatedD = 0.0;

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean tuned = false;
        int iterations = 0;
        final int MAX_ITERATIONS = 12;

        while (opModeIsActive() && !tuned && iterations < MAX_ITERATIONS) {
            iterations++;
            PIDFCoefficients currentCoeffs = new PIDFCoefficients(
                    calculatedP, calculatedI, calculatedD, calculatedF
            );

            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentCoeffs);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentCoeffs);

            shooter1.setVelocity(TARGET_VELOCITY);
            shooter2.setVelocity(TARGET_VELOCITY);

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 2.0) {
                telemetry.addData("Tuning Iteration", iterations);
                telemetry.addData("Target Velocity", TARGET_VELOCITY);
                telemetry.addData("Current Velocity", shooter1.getVelocity());
                telemetry.addData("P", calculatedP);
                telemetry.addData("F", calculatedF);
                telemetry.update();
            }

            timer.reset();
            double totalError = 0;
            int samples = 0;

            while (opModeIsActive() && timer.seconds() < 1.0) {
                double vel = shooter1.getVelocity();
                totalError += Math.abs(TARGET_VELOCITY - vel);
                samples++;
                sleep(20);
            }

            double avgError = totalError / Math.max(1, samples);

            if (avgError <= ALLOWABLE_ERROR) {
                tuned = true;
            } else {
                calculatedP *= 1.15;
            }
        }

        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        shooter1.setPower(0);
        shooter2.setPower(0);

        while (opModeIsActive()) {
            telemetry.addData("kP", calculatedP);
            telemetry.addData("kI", calculatedI);
            telemetry.addData("kD", calculatedD);
            telemetry.addData("kF", calculatedF);
            telemetry.update();
            idle();
        }
    }
}