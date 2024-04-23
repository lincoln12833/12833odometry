package org.firstinspires.ftc.mmcenterstage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class MM_Drivetrain {
    private final LinearOpMode opMode;
    private ElapsedTime timer = new ElapsedTime();

    @Config
    public static class DrivePower {
        public static double MAX_DRIVE_POWER = .7;
        public static double APRIL_TAG_THRESHOLD = 2;
        public static double DRIVE_P_COEFF = .0166;
        public static double RAMP_WAIT_TIME = .1;
    }

    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;

    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;

    //public MM_AprilTags aprilTags;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Telemetry dashboardTelemetry;
    boolean isSlow = false;

    public static double MAX_DRIVE_POWER = .5;
    public static double MIN_DRIVE_POWER = .28;
    public static double DRIVE_P_COEFF = .0166;

    public static double STRAFE_P_COEFF = .02;

    private final double WHEEL_DIAMETER = 2;
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private final double TICKS_PER_REVOLUTION = 8192;
    private final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

    static double COORDINATE_THRESHOLD = .3;

    double xIntercept = 0; //TODO add changeable start
    double yIntercept = 0;

    public MM_Drivetrain(LinearOpMode opMode, Gamepad currentGamepad1, Gamepad previousGamepad1, Telemetry dashboardTelemetry) {
        this.opMode = opMode;
        this.currentGamepad1 = currentGamepad1;
        this.previousGamepad1 = previousGamepad1;
        this.dashboardTelemetry = dashboardTelemetry;
        init();
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = opMode.gamepad1.right_stick_x;

        double FLPower = drivePower + strafePower + rotatePower;
        double FRPower = drivePower - strafePower - rotatePower;
        double BLPower = drivePower - strafePower + rotatePower;
        double BRPower = drivePower + strafePower - rotatePower;

        if (currentGamepad1.a && !previousGamepad1.a) {
            isSlow = !isSlow;
        }

        double maxMotorPwr = Math.max(Math.max(Math.abs(FLPower), Math.abs(FRPower)),
                Math.max(Math.abs(BLPower), Math.abs(BRPower)));

        if (maxMotorPwr > 1) {
            FLPower /= maxMotorPwr;
            FRPower /= maxMotorPwr;
            BLPower /= maxMotorPwr;
            BRPower /= maxMotorPwr;
        }

        if (isSlow) {
            FLPower *= 0.5;
            FRPower *= 0.5;
            BLPower *= 0.5;
            BRPower *= 0.5;
        }

        flMotor.setPower(FLPower * 0.7);
        frMotor.setPower(FRPower * 0.7);
        blMotor.setPower(BLPower * 0.7);
        brMotor.setPower(BRPower * 0.7);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(opMode.gamepad1);
    }


    public void goToCoordinates(double targetX, double targetY){
        double leftError;
        double rightError;
        double errorY;

        leftError = targetX - xIntercept;
        rightError = targetX - xIntercept;
        errorY = targetY - yIntercept;

        while (Math.abs(leftError) > COORDINATE_THRESHOLD || rightError > COORDINATE_THRESHOLD || Math.abs(errorY) > COORDINATE_THRESHOLD){
            leftError = targetX - (leftEncoder.getCurrentPosition() / TICKS_PER_INCH);
            rightError = targetX - (rightEncoder.getCurrentPosition() / TICKS_PER_INCH);
            errorY = targetY - (errorY / TICKS_PER_INCH);

            flMotor.setPower((leftError * DRIVE_P_COEFF * MAX_DRIVE_POWER) + (errorY * STRAFE_P_COEFF * MAX_DRIVE_POWER));
            frMotor.setPower((rightError * DRIVE_P_COEFF * MAX_DRIVE_POWER) - (errorY * STRAFE_P_COEFF * MAX_DRIVE_POWER));
            blMotor.setPower((leftError * DRIVE_P_COEFF * MAX_DRIVE_POWER) - (errorY * STRAFE_P_COEFF * MAX_DRIVE_POWER));
            brMotor.setPower((rightError * DRIVE_P_COEFF * MAX_DRIVE_POWER) + (errorY * STRAFE_P_COEFF * MAX_DRIVE_POWER));
        }
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }


//    public void driveToAprilTag() {
//        boolean rampedUp = false;
//
//        timer.reset();
//        while (opMode.opModeIsActive() && getError("y", 9, 9) > DrivePower.APRIL_TAG_THRESHOLD) {
//
//
//            if (!rampedUp) {
//                for (double i = 0; i < DrivePower.MAX_DRIVE_POWER; i += .1) {
//
//                    flMotor.setPower(i);
//                    frMotor.setPower(i);
//                    blMotor.setPower(i);
//                    brMotor.setPower(i);
//
//                    timer.reset();
//
//                    while(opMode.opModeIsActive() && timer.time() < DrivePower.RAMP_WAIT_TIME ){
//                        dashboardTelemetry.addData("time", timer.time());
//                        dashboardTelemetry.addData("error", getError("y", 6, 9));
//                        dashboardTelemetry.addData("i", i);
//                        dashboardTelemetry.update();
//
//                        if(getError("y", 6, 9) <= DrivePower.APRIL_TAG_THRESHOLD){
//                            break;
//                        }
//                    }
//                    if(getError("y", 6, 9) <= DrivePower.APRIL_TAG_THRESHOLD){
//                        break;
//                    }
//
//                }
//                rampedUp = true;
//            }
//
//            double power = Math.abs(getError("y", 10, 2) * DrivePower.DRIVE_P_COEFF * DrivePower.MAX_DRIVE_POWER);
//            power = Math.min(power, DrivePower.MAX_DRIVE_POWER);
//
//            flMotor.setPower(power);
//            frMotor.setPower(power);
//            blMotor.setPower(power);
//            brMotor.setPower(power);
//
//        }
//        flMotor.setPower(0);
//        frMotor.setPower(0);
//        blMotor.setPower(0);
//        brMotor.setPower(0);
//
//    }


//    private double getError(String axis, double target, int targetId) {
//        double error = 0;
//
//        //AprilTagDetection tagId = getId(targetId);
//
//        if (axis.equals("y") && tagId != null) {
//            error = target - tagId.ftcPose.y;
//            return Math.abs(error);
//        } else {
//            opMode.telemetry.addData("no aprilTags", "detected");
//        }
//        if (axis.equals("x")) {
//            error = target - tagId.ftcPose.x;
//        }
//        return Math.abs(error);
//    }

//    public AprilTagDetection getId(int id) {
//        List<AprilTagDetection> currentDetections = aprilTags.aprilTagProcessor.getDetections();
//        opMode.telemetry.addData("list size", currentDetections.size());
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.id == id) {
//                opMode.telemetry.addData("id", detection.id);
//                return detection;
//            }
//
//        }
//
//        return null;
//    }


    public void init() {
        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        leftEncoder = opMode.hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = opMode.hardwareMap.get(DcMotorEx.class, "rightEncoder");
        backEncoder = opMode.hardwareMap.get(DcMotorEx.class, "backEncoder");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

       // aprilTags = new MM_AprilTags(opMode);
    }
}

