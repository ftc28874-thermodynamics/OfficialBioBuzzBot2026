package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flower {
    private Servo flowerServo;

    private final double OPEN_POS = 1.0;
    private final double CLOSED_POS = 0.0;

    public Flower(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, "flower_servo");
    }

    public void open() {
        flowerServo.setPosition(OPEN_POS);
    }

    public void close() {
        flowerServo.setPosition(CLOSED_POS);
    }
    public void setPosition(double position) {
        flowerServo.setPosition(position);
    }
}
