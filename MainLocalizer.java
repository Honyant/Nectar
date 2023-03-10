package org.firstinspires.ftc.teamcode.NectarCore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
public abstract class MainLocalizer {

    public static int curExp = 28000;
    public static boolean updateTelemetry = true;
    public static int curGain = 255;
    public static double width = 480;
    public static double height = 360;
    public FreightPipeline pipeline;
    public boolean open = false;
    public boolean blueSide = false;
    OpenCvWebcam webcam;
    long maxExp;
    long minExp;
    int maxGain;
    int minGain;
    ExposureControl exposureControl;
    GainControl gainControl;
    Gamepad gamepad1;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    FtcDashboard dashboard;


    public abstract Pose2d getCorrectedRobotPose2(long captureTimeNanos);


    public MainLocalizer(FtcDashboard dashboard, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        this.dashboard = dashboard;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void initLocalizer() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamF"),
                cameraMonitorViewId);
        pipeline = new FreightPipeline(false, dashboard, telemetry) {

            public Pose2d getCorrectedRobotPose(long captureTimeNanos) {
                return getCorrectedRobotPose2(captureTimeNanos);
            }

        };

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                exposureControl = webcam.getExposureControl();
                gainControl = webcam.getGainControl();

                maxExp = exposureControl.getMaxExposure(TimeUnit.MICROSECONDS);
                minExp = exposureControl.getMinExposure(TimeUnit.MICROSECONDS);
                maxGain = gainControl.getMaxGain();
                minGain = gainControl.getMinGain();

                gainControl.setGain(curGain);

                exposureControl.setAePriority(false);
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(curExp, TimeUnit.MICROSECONDS);
                open = true;
                initLoop();
                pipeline.blueSide = blueSide;
            }

            @Override
            public void onError(int errorCode) {
            }
        });

    }

    public void initLoop() {
        if (!open) return;
        webcam.getPtzControl().setZoom(webcam.getPtzControl().getMinZoom());

        float changeExp = -gamepad1.left_stick_y / 100;
        float changeGain = -gamepad1.right_stick_y / 100;

        int changeExpInt = (int) (changeExp * 500);
        int changeGainInt = (int) (changeGain * 5);

        curExp += changeExpInt;
        curGain += changeGainInt;

        curExp = (int) Math.max(curExp, minExp);
        curExp = (int) Math.min(curExp, maxExp);

        curGain = Math.max(curGain, minGain);
        curGain = Math.min(curGain, maxGain);

        gainControl.setGain(curGain);
        exposureControl.setExposure(curExp, TimeUnit.MICROSECONDS);


        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addLine("\nExposure: left stick; Gain: right stick");
        telemetry.addData("Exposure", "Min:%d, Max:%d, Cur:%d", minExp, maxExp, curExp);
        telemetry.addData("Gain", "Min:%d, Max:%d, Cur:%d", minGain, maxGain, curGain);

        //dont update telemetry if other program is doing it
        if (updateTelemetry) telemetry.update();

    }

    public void terminator() {
        webcam.stopStreaming();
    }

    public Vector2d getFreightPose() {
        return pipeline.getCubePose();
    }

    public Vector2d getRelativeFreightPose() {
        return pipeline.getRelCubePose();
    }



}
