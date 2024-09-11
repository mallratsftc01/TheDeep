package epra.location;

import epra.IMUExpanded;
import epra.math.geometry.Angle;
import epra.math.geometry.Geometry;
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
    private Angle phi = new Angle(0.0);

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

    /**Updates the phi (delta theta) value using the encoders.
     * @return The new phi value. */
    public Angle phiEncoder() {
        float l = (float) Math.abs(Geometry.subtract(displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT)).x);
        phi = new Angle((float) (delta.get(Orientation.LEFT) - delta.get(Orientation.RIGHT)) / l);
        return phi;
    }
    /**Updates the phi (delta theta) value using the imu.
     * @return The new phi value. */
    public Angle phiIMU() {
        phi = Geometry.subtract(imu.getYaw(), pose.angle);
        return phi;
    }
    /**Finds the center displacement of the parallel encoders.*/
    public double centerDisplacement() { return (delta.get(Orientation.LEFT) + delta.get(Orientation.RIGHT)) / 2.0; }
    /**Finds the displacement of the perpendicular encoder.*/
    public double perpendicularDisplacement() { return delta.get(Orientation.PERPENDICULAR) - (displacement.get(Orientation.PERPENDICULAR).y * phi.getRadian()); }

    /**Estimates the new pose value.
     * @return The new pose value.*/
    public Pose estimatePose() {
        updateDelta();
        updatePos();
        phiEncoder();
        Point p0 = new Point(
                centerDisplacement(),
                perpendicularDisplacement()
        );
        Point p1 = new Point(
                (p0.x * Geometry.cos(pose.angle)) - (p0.x * Geometry.sin(pose.angle)),
                (p0.y * Geometry.sin(pose.angle)) + (p0.y * Geometry.cos(pose.angle))
        );
        Point p2 = new Point(
                (p1.x * (Geometry.sin(phi)) / phi.getRadian() + (p1.x * (Geometry.cos(phi) - 1.0) / phi.getRadian())),
                (p1.y * (1.0 - Geometry.cos(phi)) / phi.getRadian()) + (p1.y * (Geometry.sin(phi)) / phi.getRadian())
        );
        pose = new Pose(
                Geometry.add(pose.point, p2),
                Geometry.add(pose.angle, phi)
        );
        return pose;
    }
}
