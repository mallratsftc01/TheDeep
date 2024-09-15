package epra.location;

import epra.IMUExpanded;
import epra.math.geometry.Angle;
import epra.math.geometry.Geometry;
import epra.math.geometry.Point;
import epra.math.geometry.Quadrilateral;
import epra.math.geometry.Vector;
import epra.math.statistics.RollingAverage;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**Uses odometer encoders to determine robot pose.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Odometery {

    private final double INCH_PER_TICK = (48.0 * Math.PI) / 50800.0;

    private enum Orientation { LEFT, RIGHT, PERPENDICULAR }

    private Map<Orientation, DcMotorEx> encoder = new HashMap<>();
    private Map<Orientation, Point> displacement = new HashMap<>();
    private Map<Orientation, Integer> pos = new HashMap<>();
    private Map<Orientation, Integer> delta = new HashMap<>();
    private Angle phi = new Angle(0.0);

    private IMUExpanded imu;

    private Pose pose;

    private long saveTime;

    /**A buffer of the velocity (delta position) of the robot.*/
    private RollingAverage velocityBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);
    /**A buffer of the angle of the velocity (phi) of the robot in radians.*/
    private RollingAverage phiBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);

    /**Uses odometer encoders to determine robot pose.
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
        encoder.put(Orientation.RIGHT, rightEncoder);
        encoder.put(Orientation.PERPENDICULAR, perpendicularEncoder);
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.RIGHT, displacementRight);
        displacement.put(Orientation.PERPENDICULAR, displacementPerpendicular);
        this.imu = imu;
        pose = startPose;

        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            pos.put(entry.getKey(), entry.getValue().getCurrentPosition());
        }
        for (Map.Entry<Orientation, DcMotorEx> entry : encoder.entrySet()) {
            delta.put(entry.getKey(), entry.getValue().getCurrentPosition() - pos.get(entry.getKey()));
        }
        saveTime = System.currentTimeMillis();
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
                (p1.x * (Geometry.sin(phi)) / phi.getRadian() + (p1.x * (Geometry.cos(phi) - 1.0) / phi.getRadian())) * INCH_PER_TICK,
                (p1.y * (1.0 - Geometry.cos(phi)) / phi.getRadian()) + (p1.y * (Geometry.sin(phi)) / phi.getRadian()) * INCH_PER_TICK
        );
        pose = new Pose(
                Geometry.add(pose.point, p2),
                Geometry.add(pose.angle, phi)
        );
        double time = (System.currentTimeMillis() - saveTime) / 1000.0;
        velocityBuffer.addValue(Geometry.pythagorean(p2.x, p2.y) / time);
        phiBuffer.addValue(Geometry.add(phi, new Vector(p2)).getRadian() / time);
        saveTime = System.currentTimeMillis();
        return pose;
    }

    /**Returns the velocity of the robot as a vector (inch/second).*/
    public Vector getVelocity() { return new Vector(velocityBuffer.getAverage(), new Angle((float) phiBuffer.getAverage())); }
    /**Returns the acceleration of the robot as a vector (inch/second²).*/
    public Vector getAcceleration() {
        RollingAverage vd = velocityBuffer.getDerivative();
        RollingAverage ad = phiBuffer.getDerivative();
        return new Vector(vd.getAverage(), new Angle((float) ad.getAverage()));
    }

    /**Draws the robot, its velocity, and acceleration onto the field map.
     * @param packet A telemetry packet to draw to.
     * @param robotShape A quadrilateral representing the 2D shape of the robot relative to the center of the robot.
     * @return A packet modified to contain a drawing of the robot pose onto the field map.*/
    public TelemetryPacket drawPose(TelemetryPacket packet, Quadrilateral robotShape) {
        TelemetryPacket p = packet;
        Quadrilateral shape = new Quadrilateral(
                Geometry.rotate(robotShape.getA(), pose.angle),
                Geometry.rotate(robotShape.getB(), pose.angle),
                Geometry.rotate(robotShape.getC(), pose.angle),
                Geometry.rotate(robotShape.getD(), pose.angle)
        );
        double[] shapeX = {shape.getA().x, shape.getB().x, shape.getC().x, shape.getD().x};
        double[] shapeY = {shape.getA().y, shape.getB().y, shape.getC().y, shape.getD().y};
        Point velocity = Geometry.add(pose.point, getVelocity().toPoint());
        Point acceleration = Geometry.add(velocity, getAcceleration().toPoint());
        p.fieldOverlay()
                //draws center point
                .setFill("blue")
                .fillCircle(pose.point.x, pose.point.y, 1)
                //draws robot outline
                .fillPolygon(Arrays.copyOfRange(shapeX, 0, 1), Arrays.copyOfRange(shapeY, 0, 1))
                .fillPolygon(Arrays.copyOfRange(shapeX, 1, 2), Arrays.copyOfRange(shapeY, 1, 2))
                .fillPolygon(Arrays.copyOfRange(shapeX, 2, 3), Arrays.copyOfRange(shapeY, 2, 3))
                .fillPolygon(new double[] {shapeX[3], shapeX[0]}, new double[] {shapeY[3], shapeY[0]})
                //draw velocity
                .setFill("green")
                .fillPolygon(new double[] {pose.point.x, velocity.x}, new double[] {pose.point.y, velocity.y})
                //draw acceleration
                .setFill("red")
                .fillPolygon(new double[] {acceleration.x, velocity.x}, new double[] {acceleration.y, velocity.y});
        return p;
    }

    /**Draws the robot, its velocity, acceleration, and motor velocities onto the field map.
     * @param packet A telemetry packet to draw to.
     * @param robotShape A quadrilateral representing the 2D shape of the robot relative to the center of the robot.
     * @param motorPos An array of the point locations of the motors relative to the robot.
     * @param motorVelocity An array of vectors representing the angle and power of each motor.
     * @return A packet modified to contain a drawing of the robot pose onto the field map.*/
    public TelemetryPacket drawPose(TelemetryPacket packet, Quadrilateral robotShape, Point[] motorPos, Vector[] motorVelocity) {
        TelemetryPacket p = packet;
        Quadrilateral shape = new Quadrilateral(
                Geometry.rotate(robotShape.getA(), pose.angle),
                Geometry.rotate(robotShape.getB(), pose.angle),
                Geometry.rotate(robotShape.getC(), pose.angle),
                Geometry.rotate(robotShape.getD(), pose.angle)
        );
        double[] shapeX = {shape.getA().x, shape.getB().x, shape.getC().x, shape.getD().x};
        double[] shapeY = {shape.getA().y, shape.getB().y, shape.getC().y, shape.getD().y};
        Point velocity = Geometry.add(pose.point, getVelocity().toPoint());
        Point acceleration = Geometry.add(velocity, getAcceleration().toPoint());
        p.fieldOverlay()
                //draws center point
                .setFill("blue")
                .fillCircle(pose.point.x, pose.point.y, 1)
                //draws robot outline
                .fillPolygon(Arrays.copyOfRange(shapeX, 0, 1), Arrays.copyOfRange(shapeY, 0, 1))
                .fillPolygon(Arrays.copyOfRange(shapeX, 1, 2), Arrays.copyOfRange(shapeY, 1, 2))
                .fillPolygon(Arrays.copyOfRange(shapeX, 2, 3), Arrays.copyOfRange(shapeY, 2, 3))
                .fillPolygon(new double[] {shapeX[3], shapeX[0]}, new double[] {shapeY[3], shapeY[0]})
                //draw velocity
                .setFill("green")
                .fillPolygon(new double[] {pose.point.x, velocity.x}, new double[] {pose.point.y, velocity.y})
                //draw acceleration
                .setFill("red")
                .fillPolygon(new double[] {acceleration.x, velocity.x}, new double[] {acceleration.y, velocity.y});
        //draw motor velocities
        for (int i = 0; i < motorPos.length; i++) {
            Point start = Geometry.add(pose.point, motorPos[i]);
            Point end = Geometry.add(start, motorVelocity[i].toPoint());
            p.fieldOverlay()
                    .setFill("green")
                    .fillPolygon(new double[] {start.x, end.x}, new double[] { start.y, end.y});
        }
        return p;
    }
}
