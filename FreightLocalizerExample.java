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
@TeleOp(name = "FreightLocalizerExample", group = "Prototype")  // @Autonomous(...) is the other common choice
public class FreightLocalizerExample extends LinearOpMode {

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

        while (!isStarted()) {
            mainLocalizer.initLoop();
            sleep(100);//need this
        }

        mysteryRobot.drive.setPoseEstimate(BaseAutonomous.flipPose(new Pose2d(11, -66.5, Math.toRadians(0))));
        while (opModeIsActive()&&!isStopRequested()) {
            mysteryRobot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            mysteryRobot.drive.update();
            telemetry.update();
            Vector2d globalPose = mainLocalizer.getFreightPose();
            telemetry.addData("Global Position", globalPose.getX() + "," + globalPose.getY());
            idle();
        }
        mainLocalizer.terminator();
    }




}