package org.firstinspires.ftc.teamcode.NectarCore;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config
public class CalibratorPipeline extends OpenCvPipeline {

    public static int lowerVal=120;
    public static int upperSat=125;
    public static Scalar lowerOrange = new Scalar(0.0, 0.0, lowerVal);
    public static Scalar upperOrange = new Scalar(255.0, upperSat, 255.0);
    public static double epsilonMult = 0.03;
    public static int cutoffLine = 700;
    public static int cutoffLine2 = 1400;

    public Point guess = new Point();
    MatOfPoint2f result2 = new MatOfPoint2f();
    MatOfPoint2f result = new MatOfPoint2f();
    MatOfPoint result1 = new MatOfPoint();
    FtcDashboard dashboard;
    Imgcodecs imageCodecs;
    Mat meme;
    private final boolean sendDash = true;

    CalibratorPipeline(FtcDashboard dashboard) {

        this.dashboard = dashboard;
        imageCodecs = new Imgcodecs();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1); // variable to store mask in
        Core.inRange(input, lowerOrange, upperOrange, mask);
        List<MatOfPoint> contours = new ArrayList();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB_FULL);
        Imgproc.line(input, new Point(0, cutoffLine), new Point(960, cutoffLine), new Scalar(200, 0, 0), 2);
        //Imgproc.drawContours(input, contours, -1, new Scalar(150, 150, 255), 2);
        mask.release();
        hierarchy.release();
        if (!contours.isEmpty()) {
            MatOfPoint biggestContour = contours.get(0);
            for (MatOfPoint a : contours) {

                Rect rect = Imgproc.boundingRect(a);
                if (rect.width < 5 || rect.height < 5 || rect.y < cutoffLine||rect.y > cutoffLine2) continue;
                if (Imgproc.contourArea(a) > Imgproc.contourArea(biggestContour)) biggestContour = a;
            }
            double epsilon = epsilonMult * Imgproc.arcLength(new MatOfPoint2f(biggestContour.toArray()), true);
            MatOfPoint2f thisContour2f = new MatOfPoint2f();
            MatOfPoint approxContour = new MatOfPoint();
            MatOfPoint2f approxContour2f = new MatOfPoint2f();

            biggestContour.convertTo(thisContour2f, CvType.CV_32FC2);
            result = approxContour2f;

            Imgproc.approxPolyDP(thisContour2f, approxContour2f, epsilon, true);
            approxContour2f.convertTo(approxContour, CvType.CV_32S);
            Imgproc.polylines(input, Collections.singletonList(approxContour), true, result.toArray().length == 4 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0), 5);

        }

        Bitmap displayBitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        org.opencv.android.Utils.matToBitmap(input, displayBitmap);
        dashboard.sendImage(displayBitmap);

        return input;
    }

    public ArrayList<Point> getPoints() {
        return new ArrayList<>(Arrays.asList(result.toArray()));
    }
}