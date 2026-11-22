package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;

    private static final double INTAKE_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // runs the intake forward to collect game elements
    public void runIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    // stops the intake motor
    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    // reverses the intake to eject game elements
    public void reverseIntake() {
        intakeMotor.setPower(REVERSE_POWER);
    }
}