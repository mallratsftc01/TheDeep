package org.firstinspires.ftc.teamcode;

import epra.Controller;
import epra.math.geometry.Geometry;
import epra.math.geometry.Vector;
import epra.movement.DriveTrain;
import epra.IMUExpanded;
import epra.movement.Motor;
import epra.movement.MotorController;
import epra.movement.DcMotorExFrame;
import epra.location.Odometry;
import epra.location.Pose;
import epra.math.geometry.Angle;
import epra.math.geometry.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TheDeep extends LinearOpMode {
    private MotorController northEastMotor;
    private MotorController southEastMotor;
    private MotorController southWestMotor;
    private MotorController northWestMotor;

    private MotorController horizontalArmMotor;
    private MotorController verticalArmMotor;
    private MotorController climberMotor;

    private CRServo horizontalClaw;
    private MotorController horizontalWrist;
    private Servo verticalBucket;

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
        DcMotorExFrame cMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "climberMotor"));
        vaMotor.setDirection(Motor.Direction.REVERSE);

        horizontalArmMotor = new MotorController(haMotor);
        horizontalArmMotor.tuneTargetPID(0.005, 0, 0.9);
        verticalArmMotor = new MotorController(vaMotor);
        verticalArmMotor.tuneTargetPID(0.0023, 0.0, 0.9);
        verticalArmMotor.setHoldPow(0.00002);
        climberMotor = new MotorController(cMotor);

        horizontalClaw = hardwareMap.get(CRServo.class, "horizontalClaw");
        verticalBucket = hardwareMap.get(Servo.class, "verticalClaw");
        DcMotorExFrame wMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalWrist"));
        horizontalWrist = new MotorController(wMotor);

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        DriveTrain drive = new DriveTrain(new String[] {"north_west_motor", "north_east_motor", "south_west_motor", "south_east_motor"}, new MotorController[] {northWestMotor, northEastMotor, southWestMotor, southEastMotor}, new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK}, DriveTrain.DriveType.MECANUM);
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

        Config config = new Config();

        boolean fieldOrientedDrive = true;
        boolean angleCorrectionFOD = false;

        boolean useLiftPID = false;

        waitForStart();
        lastPing = System.currentTimeMillis();
        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            odometry.estimatePose();

            //Driver Controller

                //Toggles between normal drive, field oriented drive, and angle correcting field oriented drive
            if (controller1.buttonSingle(Controller.Key.BUMPER_LEFT) && controller1.buttonSingle(Controller.Key.STICK_LEFT)) { fieldOrientedDrive = !fieldOrientedDrive; }
            if (controller1.buttonSingle(Controller.Key.BUMPER_RIGHT) && controller1.buttonSingle(Controller.Key.STICK_RIGHT)) { angleCorrectionFOD = !angleCorrectionFOD; }
            if (!fieldOrientedDrive) { angleCorrectionFOD = false; }

                //Lowers the speeds when the joysticks are pressed
            Vector drivePow = Geometry.scale(controller1.analogDeadband(Controller.Stick.LEFT_STICK), (controller1.getButton(Controller.Key.STICK_LEFT) ? 0.5 : 1.0));
            float turnPow = controller1.analogDeadband(Controller.Key.RIGHT_STICK_X) * (controller1.getButton(Controller.Key.STICK_RIGHT) ? 0.5f : 1.0f);

                //Uses the correct drive method
            if (angleCorrectionFOD) { drive.fieldOrientedMecanumDrive(controller1.analogDeadband(Controller.Stick.RIGHT_STICK), drivePow, imuX.getYaw(), 0.1, true); }
            else if (fieldOrientedDrive) { drive.fieldOrientedMecanumDrive(turnPow, drivePow, imuX.getYaw()); }
            else { drive.mecanumDrive(turnPow, drivePow); }

            //Operator Controller

            double bucketPos = verticalBucket.getPosition();

                //Sets targets for the lid PID
            if (controller2.analogDeadband(Controller.Key.LEFT_STICK_Y) != 0.0) { useLiftPID = false; }
            boolean[] dpad = {controller2.getButton(Controller.Key.UP), controller2.getButton(Controller.Key.LEFT), controller2.getButton(Controller.Key.RIGHT), controller2.getButton(Controller.Key.DOWN)};
            if (dpad[0] || dpad[1] || dpad[2] || dpad[3]) { useLiftPID = true; }
            if (dpad[0]) { verticalArmMotor.setTarget(3600); }
            else if (dpad[1]) { verticalArmMotor.setTarget(2000); }
            else if (dpad[2]) { verticalArmMotor.setTarget(1600); }
            else if (dpad[3]) { verticalArmMotor.setTarget(0); }

                //Drives the lift via PID or joystick only if the arm is retracted
            if (horizontalArmMotor.getCurrentPosition() > -100) {
                double maxPow = (controller2.getButton(Controller.Key.STICK_LEFT)) ? 0.5 : 1.0;
                if (useLiftPID) { verticalArmMotor.moveToTarget(maxPow, 0.001, true); }
                else { verticalArmMotor.setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y) * maxPow); }
            }

                //Drives the arm via joystick only if the lift is retracted
            if (verticalArmMotor.getCurrentPosition() < 100) {
                double maxPow = (controller2.getButton(Controller.Key.STICK_RIGHT)) ? 0.5 : 1.0;
                horizontalArmMotor.setPower(-controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y) * maxPow);
            }
                //Dumps the bucket if A is pressed
            else {
                bucketPos = (controller2.getButton(Controller.Key.A)) ? 0.0 : 0.5;
            }

                //Zeros the motors if the corresponding bumper and stick are clicked
            if (controller2.buttonSingle(Controller.Key.BUMPER_LEFT) && controller2.buttonSingle(Controller.Key.STICK_LEFT)) {
                verticalArmMotor.zero();
            }
            if (controller2.buttonSingle(Controller.Key.BUMPER_RIGHT) && controller2.buttonSingle(Controller.Key.STICK_RIGHT)) {
                horizontalArmMotor.zero();
            }

                //If the arm is extended and the left trigger is pressed lowers and activates the claw, otherwise raises the claw
            horizontalClaw.setPower(0.0);
            if (horizontalArmMotor.getCurrentPosition() < -500) {
                if (controller2.analogDeadband(Controller.Key.LEFT_TRIGGER) != 0.0) {
                    horizontalWrist.setPower(0.0);
                    horizontalClaw.setPower(1.0);
                } else {
                    horizontalWrist.setPower(-0.2);
                }
            }

                //If both the arm and the lift are retracted and the right trigger is pressed a sample in the claw is transferred to the bucket
            if (controller2.analogDeadband(Controller.Key.RIGHT_TRIGGER) != 0.0 && horizontalArmMotor.getCurrentPosition() > -100 && verticalArmMotor.getCurrentPosition() < 100) {
                bucketPos = 1.0;
                horizontalClaw.setPower(-1.0);
            }
                //Cycles the bucket's position if B is pressed
            if (controller2.buttonSingle(Controller.Key.B)) {
                bucketPos = (bucketPos + 0.5) % 1.5;
            }
            verticalBucket.setPosition(bucketPos);

                //Drives the climber motor
            if (controller2.getButton(Controller.Key.X)) { climberMotor.setPower(1.0); }
            else if (controller2.getButton(Controller.Key.Y)) { climberMotor.setPower(-1.0); }
            else { climberMotor.setPower(0.0); }

            TelemetryPacket packet = new TelemetryPacket();
            /*odometry.drawPose(new Quadrilateral(
                            new Point(9.0, 9.0),
                            new Point(-9.0, 9.0),
                            new Point(-9.0, -9.0),
                            new Point(9.0, -9.0)
                    ), true
            );*/

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

            /*packet.put("Right Stick Angle", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getDegree());
            packet.put("Right Stick length", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getLength());
            packet.put("Right Stick x", gamepad1.right_stick_x);
            packet.put("Right Stick y", gamepad1.right_stick_y);*/

            /*packet.put("Right Encoder", odometry.getPos(Odometry.Orientation.RIGHT));
            packet.put("Left Encoder", odometry.getPos(Odometry.Orientation.LEFT));
            packet.put("Perpendicular Encoder", odometry.getPos(Odometry.Orientation.PERPENDICULAR));

            packet.put("Delta Right", odometry.getDelta(Odometry.Orientation.RIGHT));
            packet.put("Delta Left", odometry.getDelta(Odometry.Orientation.LEFT));
            packet.put("Delta Perpendicular", odometry.getDelta(Odometry.Orientation.PERPENDICULAR));

            packet.put("Phi", odometry.getPhi().getDegree());

            packet.put("Center Displacement", odometry.centerDisplacement());
            packet.put("Perpendicular Displacement", odometry.perpendicularDisplacement());*/

            //packet.put("Current Angle: ", imuX.getYaw().getDegree());
            //packet.put("Target Angle: ", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getDegree());
            //packet.put("Right Pow, direction, distance: ", Arrays.toString(drive.gyroMecanumDrive(controller1.analogDeadband(Controller.Key.LEFT_STICK_X), controller1.analogDeadband(Controller.Key.LEFT_STICK_Y), controller1.analogDeadband(Controller.Stick.RIGHT_STICK), imuX)));
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
