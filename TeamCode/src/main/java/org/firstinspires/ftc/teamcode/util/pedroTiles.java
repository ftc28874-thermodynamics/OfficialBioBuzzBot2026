package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public enum pedroTiles {
    TILE_1(132.0, 60.0, 0),
    TILE_2(108.0, 60.0, 0),
    TILE_3(84.0,  60.0, 0),
    TILE_4(60.0,  60.0, 0),
    TILE_5(36.0,  60.0, 0),
    TILE_6(12.0,  60.0, 0),

    TILE_7(12.0,  36.0, 0),
    TILE_8(36.0,  36.0, 0),
    TILE_9(60.0,  36.0, 0),
    TILE_10(84.0, 36.0, 0),
    TILE_11(108.0, 36.0, 0),
    TILE_12(132.0, 36.0, 0),

    TILE_13(132.0, 12.0, 0),
    TILE_14(108.0, 12.0, 0),
    TILE_15(84.0,  12.0, 0),
    TILE_16(60.0,  12.0, 0),
    TILE_17(36.0,  12.0, 0),
    TILE_18(12.0,  12.0, 0);

    private final double x;
    private final double y;
    private final double headingDeg;
    private final Pose pose;

    pedroTiles(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.headingDeg = headingDeg;
        this.pose = PoseFactory.degrees().of(x, y, headingDeg);
    }

    public Pose getPose() {
        return pose;
    }

    public Pose getPoseWithHeading(double customHeadingDegrees) {
        return PoseFactory.degrees().of(x, y, customHeadingDegrees);
    }

    public Pose getOffsetPose(double deltaX, double deltaY, double customHeadingDegrees) {
        return PoseFactory.degrees().of(x + deltaX, y + deltaY, customHeadingDegrees);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingDegrees() {
        return headingDeg;
    }
}