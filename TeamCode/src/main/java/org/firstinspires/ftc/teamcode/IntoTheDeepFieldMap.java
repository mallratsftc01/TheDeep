package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

import epra.location.FieldMap;
import epra.location.Section;
import epra.math.geometry.Point;
import epra.math.geometry.PolyGroup;
import epra.math.geometry.Quadrilateral;
import epra.math.geometry.Shape2D;
import epra.math.geometry.Triangle;

/**A field map of the field for the 2024-2025 season, Into the Deep.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class IntoTheDeepFieldMap implements FieldMap {

    /**Holds all the different sections of the field.*/
    enum DeepSection implements Section {
        DOES_NOT_EXIST,
        OBSTACLE,
        ASCENT_ZONE_RED,
        ASCENT_ZONE_BLUE,
        NET_ZONE_RED,
        NET_ZONE_BLUE,
        OBSERVATION_ZONE_RED,
        OBSERVATION_ZONE_BLUE,
        SPIKE_MARK_RED,
        SPIKE_MARK_BLUE,
        SUBMERSIBLE,
        UNZONED,
    }

    /**Map of all sections and associated component.*/
    static Map<IntoTheDeepFieldMap.DeepSection, Shape2D> map = new HashMap<>();

    /**The center point of the field.*/
    static Point center = new Point(0.0,0.0);
    /**A quadrilateral representing teh total area of the field.*/
    static Quadrilateral field = new Quadrilateral(
            new Point(72.0, 72.0),
            new Point(-72.0, 72.0),
            new Point(-72.0, -72.0),
            new Point(72.0, -72.0)
    );

    /**Initializes the field map.*/
    static public void init() {
        map.put(DeepSection.NET_ZONE_RED, new Triangle(
                new Point(72.0, -48.0),
                new Point(72.0, -72.0),
                new Point(48.0, -72.0)));
        map.put(DeepSection.NET_ZONE_BLUE, new Triangle(
                new Point(-72.0, 48.0),
                new Point(-72.0, 72.0),
                new Point(-48.0, 72.0)));
        map.put(DeepSection.OBSERVATION_ZONE_RED, new Quadrilateral(
                new Point(72.0, 72.0),
                new Point(58.9, 72.0),
                new Point(58.9, 48.0),
                new Point(72.0, 35.4)));
        map.put(DeepSection.OBSERVATION_ZONE_BLUE, new Quadrilateral(
                new Point(72.0, 72.0),
                new Point(58.9, 72.0),
                new Point(58.9, 48.0),
                new Point(72.0, 35.4)));
        map.put(DeepSection.SUBMERSIBLE, new Quadrilateral(
                new Point(13.75, 21.375),
                new Point(-13.75, 21.375),
                new Point(-13.75, -21.375),
                new Point(13.75, -21.375)));
        map.put(DeepSection.ASCENT_ZONE_RED,
                new PolyGroup( new Shape2D[] {
                    new Quadrilateral(
                            new Point(-13.75, -21.375),
                            new Point(13.75, -21.375),
                            new Point(-13.75, -30.625),
                            new Point(13.75, -30.625)
                    ),
                    new Triangle(
                            new Point(-13.75, -30.625),
                            new Point(13.75, -30.625),
                            new Point(0.0, -41.375)
                    )
                } ));
        map.put(DeepSection.ASCENT_ZONE_BLUE,
                new PolyGroup( new Shape2D[] {
                        new Quadrilateral(
                                new Point(-13.75, 21.375),
                                new Point(13.75, 21.375),
                                new Point(-13.75, 30.625),
                                new Point(13.75, 30.625)
                        ),
                        new Triangle(
                                new Point(-13.75, 30.625),
                                new Point(13.75, 30.625),
                                new Point(0.0, 41.375)
                        )
                } ));
    }

    /**@param point Point to check.
     * @return Which section of the field the point falls into. Returns DOES_NOT_EXIST if the point doesn't fall into any section of the field.*/
    public Section checkPoint(Point point) {
        for (Map.Entry<IntoTheDeepFieldMap.DeepSection, Shape2D> entry : map.entrySet()) {
            if (entry.getValue().checkPoint(point)) { return entry.getKey(); }
        }
        if (field.checkPoint(point)) { return DeepSection.UNZONED; }
        else { return IntoTheDeepFieldMap.DeepSection.DOES_NOT_EXIST; }
    }
}