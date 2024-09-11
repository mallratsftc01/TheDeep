package epra.location;

import epra.IMUExpanded;
import epra.math.geometry.Angle;
import epra.math.geometry.Point;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.Map;

/**
 * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
 * <p></p>
 * Uses odometer encoders to determine robot pose.*/
public class Odometery {

    private enum Orientation { LEFT, RIGHT, PERPENDICULAR }

    private Map<Orientation, DcMotorEx> encoder = new HashMap<>();
    private Map<Orientation, Point> displacement = new HashMap<>();
    private Map<Orientation, Integer> pos = new HashMap<>();
    private Map<Orientation, Integer> delta = new HashMap<>();
    private Angle deltaAngle = new Angle(0.0);

    private IMUExpanded imu;

    private Pose pose;

    /**
     * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Uses odometer encoders to determine robot pose.
     * @param leftEncoder The left parallel encoder.
     * @param rightEncoder The right parallel encoder.
     * @param perpendicularEncoder The perpendicular encoder.
     * @param displacementLeft The displacement from the robot center of the left encoder in inches.
     * @param displacementRight The displacement from the robot center of the right encoder in inches.
     * @param displacementPerpendicular The displacement from the robot center of the perpendicular encoder in inches.
     * @param imu The imu.
     * @param startPose The starting pose of the robot on the field.
     * */
    public Odometery(DcMotorEx leftEncoder, DcMotorEx rightEncoder, DcMotorEx perpendicularEncoder, Point displacementLeft, Point displacementRight, Point displacementPerpendicular, IMUExpanded imu, Pose startPose) {
        encoder.put(Orientation.LEFT, leftEncoder);
        encoder.put(Orientation.LEFT, rightEncoder);
        encoder.put(Orientation.LEFT, perpendicularEncoder);
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.LEFT, displacementRight);
        displacement.put(Orientation.LEFT, displacementPerpendicular);
        this.imu = imu;
        pose = startPose;

        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            pos.put(entry.getKey(), entry.getValue().getCurrentPosition());
        }
        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            delta.put(entry.getKey(), entry.getValue().getCurrentPosition() - pos.get(entry.getKey()));
        }
    }

    /**Updates the pos save of the encoders.*/
    public void updatePos() {
        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            pos.replace(entry.getKey(), entry.getValue().getCurrentPosition());
        }
    }

    /**Updates the delta of the encoders.*/
    public void updateDelta() {
        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            delta.replace(entry.getKey(), entry.getValue().getCurrentPosition() - pos.get(entry.getKey()));
        }
    }

}
