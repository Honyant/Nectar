package org.firstinspires.ftc.teamcode.NectarCore;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;

public class HomographyEngine {



    /*ul     =======> x
      ur    ||
      lr    ||
      ll    \/ y */
    public static Point[] transformPoints = new Point[]{
            new Point(90.043750, 130.908854),
            new Point(204.786806, 127.580729),
            new Point(225.514583, 190.815104),
            new Point(75.608333, 196.546875)};
    public static double TRANSFORM_WIDTH_INCHES = 11;//this is the piece of paper
    public static double TRANSFORM_HEIGHT_INCHES = 8.5;

    Vector2d robotOffset = new Vector2d(24, 0);//this is the offset of the robot from the center of the piece of paper
    Size renderSize;
    ArrayList<Point> transList = new ArrayList<Point>(Arrays.asList(transformPoints));
    Mat transformMat;
    MatOfPoint2f sourcePoints = new MatOfPoint2f();
    Mat resultMatTemplate;
    private Pose2d curRoboPose;
    Telemetry telemetry;

    public HomographyEngine(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initHomographyMatrix(Mat src, Point[] pts) {
        double ratio = src.size().height / (double) src.rows();

        Point ul = pts[0];
        Point ur = pts[1];
        Point lr = pts[2];
        Point ll = pts[3];

        double widthA = Math.sqrt(Math.pow(lr.x - ll.x, 2) + Math.pow(lr.y - ll.y, 2));
        double widthB = Math.sqrt(Math.pow(ur.x - ul.x, 2) + Math.pow(ur.y - ul.y, 2));
        double maxWidth = Math.max(widthA, widthB) * ratio;

        double heightA = Math.sqrt(Math.pow(ur.x - lr.x, 2) + Math.pow(ur.y - lr.y, 2));
        double heightB = Math.sqrt(Math.pow(ul.x - ll.x, 2) + Math.pow(ul.y - ll.y, 2));
        double maxHeight = Math.max(heightA, heightB) * ratio;

        renderSize= new Size(maxWidth, maxHeight);
        Mat srcMat = new Mat(4, 1, CvType.CV_32FC2);
        Mat dstMat = new Mat(4, 1, CvType.CV_32FC2);
        srcMat.put(0, 0, ul.x * ratio, ul.y * ratio, ur.x * ratio, ur.y * ratio, lr.x * ratio, lr.y * ratio, ll.x * ratio, ll.y * ratio);
        dstMat.put(0, 0, 0.0, 0.0, maxWidth, 0.0, maxWidth, maxHeight, 0.0, maxHeight);

        transformMat = Imgproc.getPerspectiveTransform(srcMat, dstMat);
        resultMatTemplate = new Mat(Double.valueOf(maxHeight).intValue(), Double.valueOf(maxWidth).intValue(), CvType.CV_8UC4);
        srcMat.release();
        dstMat.release();
    }

    //analyze point function
    public Vector2d pixelToWorldPose(Point a) {
        //Convert from pixel coordinates to world coordinates
        Point q = projectiveTransform(a.x, a.y);
        //These may look incorrectly switched, but this is intended for RR coordinate system
        double yCoord = (1 - (q.x) / renderSize.width) * TRANSFORM_WIDTH_INCHES - TRANSFORM_WIDTH_INCHES / 2;
        double xCoord = (1 - (q.y) / renderSize.height) * TRANSFORM_HEIGHT_INCHES - TRANSFORM_HEIGHT_INCHES / 2;
        Vector2d g = new Vector2d(xCoord, yCoord).plus(robotOffset);//relative
        return (getCurRoboPose().vec()).plus(g.rotated(getCurRoboPose().getHeading()));//absolute
    }

    public void init(Mat firstFrame) {
        //Initialize the transform matrix
        sourcePoints.fromList(transList);
        initHomographyMatrix(firstFrame, transformPoints);
        telemetry.setMsTransmissionInterval(20);
    }

    private Point projectiveTransform(double x, double y) {
        final MatOfPoint2f point = new MatOfPoint2f();
        point.alloc(1);
        point.put(0, 0, x, y);
        Core.perspectiveTransform(point, point, transformMat);//opencv should rename this to projectiveTransform
        return new Point(point.get(0, 0)[0], point.get(0, 0)[1]);
    }

    public Pose2d getCurRoboPose() {
        return curRoboPose;
    }

    public void setCurRoboPose(Pose2d curRoboPose) {
        this.curRoboPose = curRoboPose;
    }
}
