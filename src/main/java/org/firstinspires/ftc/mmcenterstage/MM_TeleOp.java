package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele-op", group = "mm")

public class MM_TeleOp extends LinearOpMode {

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

        while(opModeIsActive()){
            robot.drivetrain.driveWithSticks();
            //robot.collector.collect();

            telemetry.update();
        }

    }
}

