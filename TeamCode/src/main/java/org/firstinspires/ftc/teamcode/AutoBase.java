package org.firstinspires.ftc.teamcode;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import epra.Controller;
import epra.IMUExpanded;
import epra.JSONReader;
import epra.location.Odometry;
import epra.math.geometry.Point;
import epra.movement.DcMotorExFrame;
import epra.movement.DriveTrain;
import epra.movement.Motor;
import epra.movement.MotorController;

@Autonomous
public class AutoBase extends LinearOpMode {

    private final long LOOP_TIME = 27 * 1000;

    private final String JSON_FILE_NAME = "json/auto/left.json";
    private final String END_JSON_FILE_NAME = "json/drive/end_left.json";

    private MotorController northEastMotor;
    private MotorController southEastMotor;
    private MotorController southWestMotor;
    private MotorController northWestMotor;

    private MotorController horizontalArmMotor;
    private MotorController verticalArmMotor;

    private Servo horizontalClaw;
    private MotorController horizontalWrist;
    private MotorController verticalBucket;

    private IMU imu1;
    private IMU imu2;
    private IMUExpanded imuX;

    private Odometry odometry;

    ArrayList<String> filenames;
    ArrayList<Step> steps;

    @Override
    public void runOpMode() {
        DcMotorExFrame neMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor"));
        DcMotorExFrame nwMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor"));
        DcMotorExFrame seMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor"));
        DcMotorExFrame swMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor"));
        swMotor.setDirection(Motor.Direction.REVERSE);
        seMotor.setDirection(Motor.Direction.REVERSE);

        northEastMotor = new MotorController(neMotor);
        northWestMotor = new MotorController(nwMotor);
        southEastMotor = new MotorController(seMotor);
        southWestMotor = new MotorController(swMotor);

        DcMotorExFrame haMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalMotor"));
        DcMotorExFrame vaMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "verticalMotor"));
        vaMotor.setDirection(Motor.Direction.REVERSE);

        horizontalArmMotor = new MotorController(haMotor);
        horizontalArmMotor.tuneTargetPID(0.005, 0, 0.9);
        verticalArmMotor = new MotorController(vaMotor);
        verticalArmMotor.tuneTargetPID(0.0023, 0.0, 0.9);
        verticalArmMotor.setHoldPow(0.00002);

        horizontalClaw = hardwareMap.get(Servo.class, "horizontalClaw");
        DcMotorExFrame wMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalWrist"));
        horizontalWrist = new MotorController(wMotor);
        verticalBucket = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "bucketMotor")));
        verticalBucket.tuneTargetPID(0.2, 0, 15.5);

        DriveTrain drive = new DriveTrain(new String[] {"north_west_motor", "north_east_motor", "south_west_motor", "south_east_motor"}, new MotorController[] {northWestMotor, northEastMotor, southWestMotor, southEastMotor}, new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK}, DriveTrain.DriveType.MECANUM);
        drive.tuneAnglePID(1, 0.00025, 270);
        drive.tunePointPID(0.25, 0.00000000, 35);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu1 = hardwareMap.get(IMU.class, "imu 1");
        //imu2 = hardwareMap.get(IMU.class, "imu 2");
        imu1.initialize(new IMU.Parameters(orientationOnRobot));
        //imu2.initialize(new IMU.Parameters(orientationOnRobot));
        imuX = new IMUExpanded(imu1);

        filenames = new ArrayList<>();
        steps = new ArrayList<>();
        filenames.addAll(Arrays.asList(JSONReader.readAuto(JSON_FILE_NAME)));
        /*//TEST CODE
        File file = AppUtil.getInstance().getSettingsFile(filenames.get(0));
        Gson gson = new Gson();
        Type stepListType = new TypeToken<List<Step>>() {}.getType();
        List<Step> directions;
        FileReader reader = null;
        try {
            reader = new FileReader(file);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        directions = gson.fromJson(reader, stepListType);*/
        steps.addAll(Arrays.asList(JSONReader.readSteps(filenames.get(0))));

        odometry = new Odometry(northWestMotor, southWestMotor, northEastMotor,
                new Point(7.92784216, 3.75),
                new Point(-8, 3.75),
                new Point(0, 2.0),
                imuX,
                steps.get(0).getPose()
        );
        drive.setTargetPose(steps.get(0).getPose());
        steps.remove(0);

        filenames.remove(0);

        waitForStart();
        long startTime = System.currentTimeMillis();
        long saveTime = startTime;
        long tickTime = startTime;
        while (System.currentTimeMillis() - startTime < LOOP_TIME && (!filenames.isEmpty() || !steps.isEmpty())) {
            odometry.estimatePose();

            if (steps.isEmpty()) {
                steps.addAll(Arrays.asList(JSONReader.readSteps(filenames.get(0))));
                filenames.remove(0);
                saveTime = System.currentTimeMillis();
            }

            verticalArmMotor.setTarget((int) steps.get(0).lift_target);
            horizontalArmMotor.setTarget((int) steps.get(0).arm_target);

            horizontalClaw.setPosition(0.0);
            horizontalWrist.setTarget(-30);
            if (steps.get(0).claw_open && horizontalWrist.getCurrentPosition() > -30) {
                horizontalClaw.setPosition(1.0);
            }

            if (!steps.get(0).wrist_down) {
                if (horizontalWrist.getCurrentPosition() > -30) {
                    horizontalWrist.moveToTarget(0.5, 0.1, true);
                }
                horizontalWrist.setHoldPow(0.001);
            } else {
                horizontalWrist.setHoldPow(-0.01);
                horizontalWrist.setPower(0.1);
            }

            verticalBucket.setTarget(steps.get(0).bucket_pos);
            verticalBucket.moveToTarget(0.5, 0.1, true);

            drive.setTargetPose(steps.get(0).getPose());
            boolean atPose = drive.posPIDMecanumDrive(odometry.getPose(), steps.get(0).pos_tolerance, steps.get(0).angle_tolerance, steps.get(0).drive_max, true);

            long s = System.currentTimeMillis();
            telemetry.addData("ms/t", s - tickTime);
            telemetry.addData("Paths Remaining: ", filenames.size());
            telemetry.addData("Steps Remaining in Path: ", steps.size());
            telemetry.addData("At Pose: ", atPose);
            telemetry.addData("X Pose: ", odometry.getPose().point.x);
            telemetry.addData("Y Pose: ", odometry.getPose().point.y);
            telemetry.addData("Arm Pos: ", horizontalArmMotor.getCurrentPosition());
            telemetry.addData("Arm Pow: ", horizontalArmMotor.getPower());
            telemetry.addData("Lift Pos: ", verticalArmMotor.getCurrentPosition());
            telemetry.addData("Lift Pow: ", verticalArmMotor.getPower());
            telemetry.addData("Lift Target: ", steps.get(0).lift_target);
            tickTime = s;

            if (atPose
                    && (verticalArmMotor.moveToTarget(steps.get(0).lift_max, steps.get(0).lift_tolerance,steps.size() == 1) || Math.abs(verticalArmMotor.getCurrentPosition() - steps.get(0).lift_target) <= 50)
                    && horizontalArmMotor.moveToTarget(steps.get(0).arm_max, steps.get(0).arm_tolerance, steps.size() == 1)
                    && System.currentTimeMillis() - saveTime >= steps.get(0).millis) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                telemetry.update();
                continue;
            }

            //fail safe if it stalls
            if (steps.get(0).millis == 0.0 && (System.currentTimeMillis() - saveTime) >= 10000.0) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
            }

            telemetry.update();
        }

        steps.addAll(Arrays.asList(JSONReader.readSteps(END_JSON_FILE_NAME)));
        saveTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 30000 && !steps.isEmpty()) {
            odometry.estimatePose();

            verticalArmMotor.setTarget((int) steps.get(0).lift_target);
            horizontalArmMotor.setTarget((int) steps.get(0).arm_target);

            horizontalClaw.setPosition(0.0);
            horizontalWrist.setTarget(-30);
            if (steps.get(0).claw_open && horizontalWrist.getCurrentPosition() > -30) {
                horizontalClaw.setPosition(1.0);
            }

            if (steps.get(0).wrist_down) {
                if (horizontalWrist.getCurrentPosition() > -30) {
                    horizontalWrist.moveToTarget(0.5, 0.1, true);
                }
                horizontalWrist.setHoldPow(0.001);
            } else {
                horizontalWrist.setHoldPow(-0.01);
                horizontalWrist.setPower(0.1);
            }

            verticalBucket.setTarget(steps.get(0).bucket_pos);
            verticalBucket.moveToTarget(0.5, 0.1, true);

            drive.setTargetPose(steps.get(0).getPose());
            boolean atPose = drive.posPIDMecanumDrive(odometry.getPose(), steps.get(0).pos_tolerance, steps.get(0).angle_tolerance, steps.get(0).drive_max, true);

            if (atPose
                    && (verticalArmMotor.moveToTarget(steps.get(0).lift_max, steps.get(0).lift_tolerance,steps.size() == 1) || Math.abs(verticalArmMotor.getCurrentPosition() - steps.get(0).lift_target) <= 25)
                    && horizontalArmMotor.moveToTarget(steps.get(0).arm_max, steps.get(0).arm_tolerance, steps.size() == 1)
                    && System.currentTimeMillis() - saveTime >= steps.get(0).millis) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                continue;
            }

            //fail safe if it stalls
            if (steps.get(0).millis == 0.0 && (System.currentTimeMillis() - saveTime) >= 1000.0) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
            }
        }
    }
}