/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.NectarCore;

import static org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest.updateDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Dependencies.BaseAutonomous;
import org.firstinspires.ftc.teamcode.Dependencies.TurretRobot;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;

//@Disabled
@TeleOp(name = "FullAutoExample", group = "Prototype")  // @Autonomous(...) is the other common choice
public class FullAutoExample extends LinearOpMode {

    MainLocalizer mainLocalizer;
    TurretRobot mysteryRobot = new TurretRobot();
    public static double scale = 1;

    @Override
    public void runOpMode() {
        mysteryRobot.teleopInit(hardwareMap);
        //updateLoc=false;
        MainLocalizer.updateTelemetry=true;
        MecanumDrivetrain.sendTelemetry = false;
        mysteryRobot.drive.setPoseEstimate(BaseAutonomous.flipPose(new Pose2d(11, -65, Math.toRadians(0))));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        mainLocalizer = new MainLocalizer(dashboard, hardwareMap, telemetry, gamepad1) {
            @Override
            public Pose2d getCorrectedRobotPose2(long captureTimeNanos) {
                return mysteryRobot.drive.getCorrectedPoseEstimate().plus
                        (mysteryRobot.drive.getCorrectedPoseVelocity().times(
                                scale * ((double) (captureTimeNanos - mysteryRobot.drive.lastTimestamp) / 1000000000)));
            }

        };
        mainLocalizer.initLocalizer();
        //we need to wait before pressing start but sleep doesnt work to help this so maybe put this code in other thread idk
        //sleep(2000);
        while (!isStarted()) {
            mainLocalizer.initLoop();
            sleep(100);//need this

        }
        mysteryRobot.drive.setPoseEstimate(BaseAutonomous.flipPose(new Pose2d(11, -66.5, Math.toRadians(0))));
        while (opModeIsActive()&&!isStopRequested()) {

            updateDrive(gamepad1, mysteryRobot.drive, telemetry, 1);
            telemetry.update();
            //mainLocalizer.update();
            Vector2d target = mainLocalizer.getFreightPose();
            if (gamepad1.a){//!ringLocalizer.storedRings.isEmpty()) {
                retriveAndScore();
            }
            // Don't burn CPU cycles busy-looping in this sample

            idle();
        }
        mainLocalizer.terminator();
    }


    public void retriveAndScore() {

        /*mysteryRobot.intake.in();
        Vector2d target = mainLocalizer.getFreightPose();
        drive.updatePoseEstimate();
        Pose2d currentPose = drive.getPoseEstimate();
        double angRad = Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX());
        drive.turn(fixAngle(angRad - currentPose.getHeading() + Math.toRadians(180)));
        Trajectory trajA = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .lineToLinearHeading(new Pose2d(mainLocalizer.storedRings.get(0).ringPose, angRad + Math.toRadians(180)))
                .build();
        mainLocalizer.pipeline.deleteRingIndex(0);
        drive.followTrajectory(trajA);
        sleep(400);

        trajA = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(0, 32, 0), 0)
                .addTemporalMarker(0.6, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        mysteryRobot.shooter.setVelo(750);
                    }
                })
                .build();
        drive.followTrajectory(trajA);
        sleep(350);*/
    }




}