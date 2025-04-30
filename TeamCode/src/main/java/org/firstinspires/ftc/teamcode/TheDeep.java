package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.location.IMUExpanded;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TheDeep extends LinearOpMode {
    private MotorController northEastMotor;
    private MotorController southEastMotor;
    private MotorController southWestMotor;
    private MotorController northWestMotor;

    DriveTrain drive;

    private MotorController horizontalArmMotor;
    private MotorController verticalArmMotor;
    //private MotorController climberMotor;

    private Servo horizontalClaw;
    private MotorController horizontalWrist;
    private MotorController verticalBucket;

    private Controller controller1;
    private Controller controller2;

    private IMU imu1;
    private IMU imu2;
    private IMUExpanded imuX;

    private Odometry odometry;

    FtcDashboard dashboard;

    int deltaDiff = 0;

    long lastPing;

    @Override
    public void runOpMode() throws InterruptedException {

        northEastMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")));
        northWestMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")));
        southEastMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")));
        southWestMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")));
        southWestMotor.setDirection(Motor.Direction.REVERSE);
        southEastMotor.setDirection(Motor.Direction.REVERSE);


        horizontalArmMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalMotor")));
        horizontalArmMotor.tuneTargetPID(0.005, 0, 0.9);
        verticalArmMotor = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "verticalMotor")));
        verticalArmMotor.tuneTargetPID(0.0023, 0.0, 0.9);
        verticalArmMotor.setHoldPow(0.00002);
        verticalArmMotor.setDirection(Motor.Direction.REVERSE);

        horizontalClaw = hardwareMap.get(Servo.class, "horizontalClaw");
        verticalBucket = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "bucketMotor")));
        verticalBucket.tuneTargetPID(0.2, 0, 15.5);
        DcMotorExFrame wMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalWrist"));
        horizontalWrist = new MotorController(wMotor);
        horizontalWrist.tuneTargetPID(0.5, 0, 0);

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        drive = new DriveTrain(new String[] {"north_west_motor", "north_east_motor", "south_west_motor", "south_east_motor"}, new MotorController[] {northWestMotor, northEastMotor, southWestMotor, southEastMotor}, new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK}, DriveTrain.DriveType.MECANUM);
        drive.tuneAnglePID(1, 0.00025, 270);
        drive.tunePointPID(0.25, 0.00000001, 35);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu1 = hardwareMap.get(IMU.class, "imu 1");
        //imu2 = hardwareMap.get(IMU.class, "imu 2");
        imu1.initialize(new IMU.Parameters(orientationOnRobot));
        //imu2.initialize(new IMU.Parameters(orientationOnRobot));
        imuX = new IMUExpanded(imu1);

        odometry = new Odometry(northWestMotor, southWestMotor, northEastMotor,
                new Point(7.92784216, 3.75),
                new Point(-8, 3.75),
                new Point(0, 2.0),
                imuX,
                new Pose(
                        new Point(0, 64.0),
                        new Angle(180.0)
                )
        );

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean fieldOrientedDrive = true;
        boolean angleCorrectionFOD = false;

        boolean fodFlag = false;
        boolean acfodFlag = false;

        boolean useLiftPID = false;

        waitForStart();
        lastPing = System.currentTimeMillis();
        while (opModeIsActive()) {

            odometry.estimatePose();

            //Driver Controller

            //Toggles between normal drive, field oriented drive, and angle correcting field oriented drive
            if (controller1.getButton(Controller.Key.BUMPER_LEFT) && controller1.buttonSingle(Controller.Key.STICK_LEFT)) {
                if (!fodFlag) {
                    fieldOrientedDrive = !fieldOrientedDrive;
                    fodFlag = true;
                }
            } else {
                fodFlag = false;
            }
            if (controller1.getButton(Controller.Key.BUMPER_RIGHT) && controller1.buttonSingle(Controller.Key.STICK_RIGHT)) {
                if (!acfodFlag) {
                    angleCorrectionFOD = !angleCorrectionFOD;
                    acfodFlag = true;
                } else {
                    acfodFlag = false;
                }
            }
            if (!fieldOrientedDrive) {
                angleCorrectionFOD = false;
            }

            //Lowers the speeds when the joysticks are pressed
            Vector drivePow = Geometry.scale(controller1.analogDeadband(Controller.Stick.LEFT_STICK), (controller1.getButton(Controller.Key.STICK_LEFT) ? 0.5 : 1.0));
            float turnPow = controller1.analogDeadband(Controller.Key.RIGHT_STICK_X) * (controller1.getButton(Controller.Key.STICK_RIGHT) ? 0.5f : 1.0f);

            //Uses the correct drive method
            if (angleCorrectionFOD) {
                drive.fieldOrientedMecanumDrive(controller1.analogDeadband(Controller.Stick.RIGHT_STICK), drivePow, imuX.getYaw(), 0.1, true);
            } else if (fieldOrientedDrive) {
                drive.fieldOrientedMecanumDrive(turnPow, drivePow, imuX.getYaw());
            } else {
                drive.mecanumDrive(turnPow, drivePow);
            }

            //Operator Controller

            //Sets targets for the lid PID
            if (controller2.analogDeadband(Controller.Key.LEFT_STICK_Y) != 0.0) {
                useLiftPID = false;
            }
            /*boolean[] dpad = {controller2.getButton(Controller.Key.UP), controller2.getButton(Controller.Key.LEFT), controller2.getButton(Controller.Key.RIGHT), controller2.getButton(Controller.Key.DOWN)};
            if (dpad[0] || dpad[1] || dpad[2] || dpad[3]) { useLiftPID = true; }
            if (dpad[0]) { verticalArmMotor.setTarget(3600); }
            else if (dpad[1]) { verticalArmMotor.setTarget(2000); }
            else if (dpad[2]) { verticalArmMotor.setTarget(1600); }
            else if (dpad[3]) { verticalArmMotor.setTarget(0); }*/

            //Drives the lift via PID or joystick only if the arm is retracted
            if (horizontalArmMotor.getCurrentPosition() == horizontalArmMotor.getCurrentPosition()) {
                double maxPow = (controller2.getButton(Controller.Key.STICK_LEFT)) ? -0.5 : -1.0;
                if (useLiftPID) {
                    verticalArmMotor.moveToTarget(maxPow, 0.001, true);
                } else {
                    verticalArmMotor.setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y) * maxPow);
                }
            }

            //Drives the arm via joystick only if the lift is retracted
            if (verticalArmMotor.getCurrentPosition() == verticalArmMotor.getCurrentPosition()) {
                double maxPow = (controller2.getButton(Controller.Key.STICK_RIGHT)) ? 0.5 : 1.0;
                horizontalArmMotor.setPower(controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y) * maxPow);
            }
            /*else {
                if (verticalArmMotor.getCurrentPosition() > 300) {
                    if (controller2.getButton(Controller.Key.A)) {
                        verticalBucket.setTarget(150);
                    } else if (verticalArmMotor.getCurrentPosition() < 1500) {
                        verticalBucket.setTarget(-50);
                    }
                }
            }*/

            //Zeros the motors if the corresponding bumper and stick are clicked
            if (controller2.getButton(Controller.Key.BUMPER_LEFT) && controller2.getButton(Controller.Key.STICK_LEFT)) {
                verticalArmMotor.zero();
            }
            if (controller2.getButton(Controller.Key.BUMPER_RIGHT) && controller2.getButton(Controller.Key.STICK_RIGHT)) {
                horizontalArmMotor.zero();
            }

            //If x is pressed the claw is opened/closed
            horizontalClaw.setPosition(0.0);
            horizontalWrist.setTarget(-30);
            if (controller2.buttonToggleSingle(Controller.Key.X)) {
                horizontalClaw.setPosition(1.0);
            }

            //dpad controls the wrist
            horizontalWrist.setPower((controller2.getButton(Controller.Key.DOWN) ? 0.5 : 0.0) - (controller2.getButton(Controller.Key.UP) ? 0.5 : 0.0));
            if (controller2.getButton(Controller.Key.DOWN) || controller2.getButton(Controller.Key.UP)) {
                horizontalWrist.setHoldPow(0.0);
            } else if (horizontalWrist.getCurrentPosition() > 0.0) {
                horizontalWrist.setHoldPow(-0.0005);
            } else {
                horizontalWrist.setHoldPow(0.0005);
            }
                //If A is pressed bucket moves down, if b is pressed bucket moves up
            if (controller2.getButton(Controller.Key.B)) { verticalBucket.setPower(-0.5); }
            else if (controller2.getButton(Controller.Key.A)) { verticalBucket.setPower(0.5); }
            else { verticalBucket.setPower(0); }


            /*if (verticalBucket.getCurrentPosition() != verticalBucket.getTarget()) {
                verticalBucket.moveToTarget(0.5, 0.1, true);
            }*/

                /*//Drives the climber motor
            if (controller2.getButton(Controller.Key.X)) { climberMotor.setPower(1.0); }
            else if (controller2.getButton(Controller.Key.Y)) { climberMotor.setPower(-1.0); }
            else { climberMotor.setPower(0.0); }*/


            TelemetryPacket packet = new TelemetryPacket();

            packet.fieldOverlay()
                    .setFill("blue")
                    .fillCircle(odometry.getPose().point.x, odometry.getPose().point.y, 2);

            packet.put("Ping Time", System.currentTimeMillis() - lastPing);
            lastPing = System.currentTimeMillis();

            packet.put("X", odometry.getPose().point.x);
            packet.put("Y", odometry.getPose().point.y);
            packet.put("Angle", odometry.getPose().angle.getDegree());

            packet.put("Lift Pos", verticalArmMotor.getCurrentPosition());
            packet.put("Arm Pos", horizontalArmMotor.getCurrentPosition());
            packet.put("Bucket Pos", verticalBucket.getCurrentPosition());
            packet.put("Bucket Target", verticalBucket.getTarget());
            packet.put("Wrist Pos", horizontalWrist.getCurrentPosition());

            packet.put("Use Lift PID", useLiftPID);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
