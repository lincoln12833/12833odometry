package org.firstinspires.ftc.mmcenterstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MM_Robot {
    private LinearOpMode opMode;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Telemetry dashboardTelemetry;

    public MM_Drivetrain drivetrain;
    //public Collector collector;

    public MM_Robot(LinearOpMode opMode, Gamepad currentGamepad1, Gamepad previousGamepad1, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.currentGamepad1 = currentGamepad1;
        this.previousGamepad1 = previousGamepad1;
        this.dashboardTelemetry = dashboardTelemetry;

    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode, currentGamepad1, previousGamepad1, dashboardTelemetry);
        //collector = new Collector(opMode);
    }

}
