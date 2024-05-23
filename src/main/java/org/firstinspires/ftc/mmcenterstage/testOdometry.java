package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name = "odometry test", group = "mm")

public class testOdometry extends LinearOpMode {

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public MM_Robot robot = new MM_Robot(this, currentGamepad1, previousGamepad1, dashboardTelemetry);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please Wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.drivetrain.goToCoordinates(24, 0,.3);

    }
}

