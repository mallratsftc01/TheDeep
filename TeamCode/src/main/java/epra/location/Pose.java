package epra.location;

import epra.math.geometry.Angle;
import epra.math.geometry.Point;

/**
 * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
 * <p></p>
 * Store a pose value consisting of a point and an angle.*/
public class Pose {

    public Point point;
    public Angle angle;

    /**
     * Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * Store a pose value consisting of a point and an angle.
     * @param point The point to store.
     * @param angle The angle to store.*/
    public Pose(Point point, Angle angle) {
        this.point = point;
        this.angle = angle;
    }

    /**@param point The point to store.*/
    public void setPoint(Point point) { this.point = point; }
    /**@param angle The angle to store.*/
    public void setAngle(Angle angle) { this.angle = angle; }
}
