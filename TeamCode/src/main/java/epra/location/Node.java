package epra.location;

import epra.math.geometry.Vector;

public class Node extends Pose{
    public double g,h,f;
    public Vector v;

    public Node(Pose p, double g, double h, double f, Vector v) {
        super(p.point, p.angle);
        this.g = g;
        this.h = h;
        this.f = f;
        this.v = v;
    }
}
