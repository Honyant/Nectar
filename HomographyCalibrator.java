package org.firstinspires.ftc.teamcode.NectarCore;

import static org.firstinspires.ftc.teamcode.NectarCore.Utils.sortPoints;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;


/*step 1 get a blank white sheet of paper
step 2 oil floats on water
step 3 cove-
step 4 put the robot at the center of your field
step 5 fold paper first horizontally then vertically
step 6 unfold paper, but mark the exact center.
step 7 put center of paper directly in front of robot, and measure offset between both centers
step 8 wait for it to rain
step 9 run program and press A on gamepad to log code to the logcat in andriod studio
step 10 copy what is in logcat and paste to line 43 in ring localizer pipeline
*/

@TeleOp(group="Prototype")
//@Disabled
public class HomographyCalibrator extends LinearOpMode {

    OpenCvWebcam webCam;
    CalibratorPipeline pipeline;
    FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;
    int width = 2304;
    int height = 1536;

    //currently only works for c920 webcams
    //the idea is to do this at a high res and scale down the coordinates for normal use
    //due to the low-res input having a different fov, we do this strange scaling to get the correct coordinates
    public double scalex(double x) {
        return (x * 533 / 2304 - 66.5) * 320 / 400;
    }
    public double scaley(double y) {
        return (y * 355 / 1536 - 27.5) * 240 / 300;
    }

    @SuppressLint("DefaultLocale")
    @Override

    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamF"), cameraMonitorViewId);
        pipeline = new CalibratorPipeline(dashboard);
        CalibratorPipeline.cutoffLine = 700;
        webCam.setPipeline(pipeline);
        // Open async and start streaming inside opened callback
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {webCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStopRequested()) {

            sleep(20);//30hz
            ArrayList<Point> paperPoints = sortPoints(pipeline.getPoints());

            StringBuilder a;
            if (paperPoints.size() == 4) {
                a = new StringBuilder("    public static Point[] transformPoints = new Point[]{\n");
                Collections.reverse(paperPoints);
                for (Point b : paperPoints) {
                    if (paperPoints.indexOf(b) != 3) a.append(String.format("            new Point(%f, %f),\n", scalex(b.x), scaley(b.y)));
                    else a.append(String.format("            new Point(%f, %f)};", scalex(b.x), scaley(b.y)));
                }
                Log.d("code", a.toString());
                dashboardTelemetry.addLine(a.toString());
            }else{
                dashboardTelemetry.addLine("Could not find 4 points");
            }

            dashboardTelemetry.update();

        }
    }


}