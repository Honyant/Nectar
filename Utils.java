package org.firstinspires.ftc.teamcode.NectarCore;

import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.Collections;

public class Utils {
    public static ArrayList<Point> sortPoints(ArrayList<Point> points) {
        Collections.sort(points, Utils::compare);
        ArrayList<org.opencv.core.Point> topPoints = new ArrayList<>();
        ArrayList<org.opencv.core.Point> bottomPoints = new ArrayList<>();
        for (int i = 0; i < 2; i++) topPoints.add(points.get(i));
        for (int i = 2; i < 4; i++) bottomPoints.add(points.get(i));
        Collections.sort(topPoints, Utils::compare);
        Collections.sort(bottomPoints, Utils::compare);
        ArrayList<org.opencv.core.Point> allPoints = new ArrayList<>();
        allPoints.add(topPoints.get(0));
        allPoints.add(topPoints.get(1));
        allPoints.add(bottomPoints.get(1));
        allPoints.add(bottomPoints.get(0));
        return allPoints;
    }

    private static int compare(Point p1, Point p2) {
        return Double.compare(p1.x, p2.x);
    }
}
