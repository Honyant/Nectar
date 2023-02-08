package org.firstinspires.ftc.teamcode.NectarCore;

import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvCvtcolor;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtil.cvGaussianBlur;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public abstract class FreightPipeline extends TimestampedOpenCvPipeline {
    public abstract Pose2d getCorrectedRobotPose(long captureTimeNanos);

    public static int imageNum = 0;
    public static int rowThresh = 10;//20 pixels in a row
    //public static int yellowThresh = 155;// or more y in ycrcb
    public static int startRow = 160;//out of 240
    public static boolean dashSendImage = true;
    public static double cubeHue = 23;
    public static double cubeS = 70;
    public static double cubeV = 100;
    public boolean dashSendField = true;
    public boolean blueSide;


    FtcDashboard dashboard;
    MecanumDrivetrain drive;
    HomographyEngine homographyEngine;
    Vector2d closestCube = new Vector2d(0, 0);
    private Telemetry telemetry;

    public FreightPipeline(boolean b, FtcDashboard dashboard, Telemetry telemetry) {
        this.dashboard = dashboard;
        this.telemetry = telemetry;
        this.homographyEngine = new HomographyEngine(telemetry);
    }

    //convert to ycrcb and extract yellow based on thresholds
    //sum across each row
    //find lowest row that has at least N number of pixels

    public void findHeight(Mat hsv) {
        //----This lovely mess is to find the lowest layer and circumvent java being slow---
        Mat mask = new Mat(hsv.rows(), hsv.cols(), CvType.CV_8UC1); // variable to store mask in
        Core.inRange(hsv, new Scalar(cubeHue - 10, cubeS, cubeV), new Scalar(cubeHue + 10, 255, 255), mask);
        List<MatOfPoint> contours = new ArrayList();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();
        mask.release();//no mem leek
        int max = 0;
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (rect.y + rect.height > max) {
                max = rect.y + rect.height;
            }
        }
        startRow = max;
        //---Love Java being slow---
    }

    @Override
    public void init(Mat firstFrame) {
        homographyEngine.init(firstFrame);
    }

    public Mat processFrame(Mat source, long captureTimeNanos) {
        homographyEngine.setCurRoboPose(getCorrectedRobotPose(captureTimeNanos));
        if (source.width() == 0) return new Mat();
        //gaussian blur of 3
        Mat blur = new Mat();
        cvGaussianBlur(source, blur, 3);
        //convert to HSV
        Mat hsv = new Mat();
        cvCvtcolor(blur, Imgproc.COLOR_RGB2HSV, hsv);

        findHeight(hsv);//get starting row

        //binary thresholding
        Mat binaryMask = new Mat();
        Core.inRange(hsv, new Scalar(cubeHue - 10, cubeS, cubeV), new Scalar(cubeHue + 10, 255, 255), binaryMask);

        //start from bottom
        boolean breakflag = false;
        startRow = Math.min(startRow, binaryMask.rows() - 1);
        for (int i = startRow; i >= 0; i--) {
            //if we spot [threshold] continuous pixels, we report the row
            int count = 0;
            if (!blueSide) {//red
                for (int j = binaryMask.cols() - 1 - 90; j >= 0; j--) {
                    if (binaryMask.get(i, j)[0] == 255) {
                        count++;
                        if (count >= rowThresh) {
                            //return the middle pixel i+thresh/2 and the row number as a point
                            closestCube = homographyEngine.pixelToWorldPose(new Point(j - rowThresh / 2, i));
                            if (i > 2 && i < binaryMask.rows() - 2) {
                                Imgproc.rectangle(source, new Rect(j, i - rowThresh, rowThresh, rowThresh), new Scalar(255, 255, 0), 2);
                                //Imgproc.rectangle(binaryMask, new Rect(j-rowThresh,i-rowThresh,rowThresh,rowThresh), new Scalar(255, 255, 0), 5);

                                Imgproc.putText(source, "Nearest Gold Freight", new Point(j - rowThresh * 3, i + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 0), 1);
                                //Imgproc.putText(binaryMask, "Nearest Gold Freight", new Point(j-rowThresh*3, i+100), Imgproc.FONT_HERSHEY_SIMPLEX, 3, new Scalar(255, 255, 0),5);

                            }
                            breakflag = true;
                            break;
                        }
                    } else {
                        //reset if string of pixels is broken
                        count = 0;
                    }
                }
            } else {
                for (int j = 90; j < binaryMask.cols(); j++) {
                    if (binaryMask.get(i, j)[0] == 255) {
                        count++;
                        if (count >= rowThresh) {
                            //return the middle pixel i+thresh/2 and the row number as a point
                            closestCube = homographyEngine.pixelToWorldPose(new Point(j + rowThresh / 2, i));
                            if (i > 2 && i < binaryMask.rows() - 2) {
                                Imgproc.rectangle(source, new Rect(j - rowThresh, i - rowThresh, rowThresh, rowThresh), new Scalar(255, 255, 0), 2);
                                Imgproc.putText(source, "Nearest Gold Freight", new Point(j + rowThresh * 3, i + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 0), 1);
                            }
                            breakflag = true;
                            break;
                        }
                    } else count = 0;
                }
            }
            if (breakflag) break;
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        DashboardUtil.drawRobot(fieldOverlay, homographyEngine.getCurRoboPose());
        DashboardUtil.drawRing(fieldOverlay, closestCube);

        if (dashSendField) dashboard.sendTelemetryPacket(packet);

        if (dashSendImage) {//send image to dashboard
            if (imageNum == 0) sendImage(source);
            if (imageNum == 1) sendImage(blur);
            if (imageNum == 2) sendImage(hsv);
            if (imageNum == 3) sendImage(binaryMask);
        }
        //free up memory
        binaryMask.release();
        blur.release();
        hsv.release();
        return source;

    }

    private void sendImage(Mat source) {
        Bitmap displayBitmap = Bitmap.createBitmap(source.width(), source.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(source, displayBitmap);
        dashboard.sendImage(displayBitmap);
        displayBitmap.recycle();
    }


    public Vector2d getCubePose() {
        return closestCube;
    }

    public Vector2d getRelCubePose() {
        return closestCube;
    }
}