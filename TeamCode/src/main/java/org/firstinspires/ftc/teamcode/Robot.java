package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Components.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.rotatePosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.turret_saved_positions;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.Velocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.aVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.angle;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.differtime;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.xVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.yVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.ypos;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.random;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.CarouselCR;
import org.firstinspires.ftc.teamcode.Components.ChassisFactory;
import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.LedColor;
import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Components.OpenCVMasterclass;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.VSLAMChassis;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;

import java.util.Arrays;

public class Robot {

    private LinearOpMode op = null;
    public final static boolean isCorgi = true;
    boolean shouldIntake = true;
    public static int isBlue = 1;
    boolean isExtended = false;
    boolean flipped = false;
    public static boolean isBall = false;
    public static boolean resetten = true;
    public static boolean faked = false, rotated = false;
    boolean outModed = false;
    double trueStartAngle = 0;
    boolean shouldFlipIntake = false;
    boolean isReversing = false;
    double shareFlipTime = 0,startRotateTime;
    boolean isFlipping = false;
    boolean isExtending = false;
    double flipDelay = .3, reverseDelay = .7;
    double[] startTime = {-2, 0, 0, 0, 0, 0, 0, -10, 0, 100, 0, 0, 0};
    double magnitude;
    double angleInRadian;
    double angleInDegree;
    public static double startAngle;
    double power = 0.5;
    double turret_angle_control_distance = 0;
    boolean slowMode = false;
    boolean autoAiming = false;
    boolean shared_shipping_hub = false;
    boolean alliance_shipping_hub = false;
    public static boolean retracting = false;
    double time;
    boolean changed = false;
    double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 30;     // period of each cycle


    double MAX_POS = 1.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position

    boolean rampUp = true;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    int x = 0;

    // Hardware Objects
    private BasicChassis drivetrain = null;
    private CarouselCR rotation = null;
    private Intake intake = null;
    private Turret turret = null;
    private LedColor led_bank = null;
    private OpenCVMasterclass openCV = null;
    private tseDepositor TSE = null;
    private StateMachine checker = null;
    private Logger logger;

    public Robot(LinearOpMode opMode, BasicChassis.ChassisType chassisType, boolean isTeleop, boolean vuforiaNAVIGATIONneeded, double startAng) {
        op = opMode;
        startAngle = startAng;
        trueStartAngle=startAng;
        logger = new Logger(opMode);
        checker = new StateMachine(op, isTeleop, logger);
        //This link has a easy to understand explanation of ClassFactories. https://www.tutorialspoint.com/design_pattern/factory_pattern.htm
        drivetrain = ChassisFactory.getChassis(chassisType, op, vuforiaNAVIGATIONneeded, isTeleop);
        rotation = new CarouselCR(op);
        intake = new Intake(op, isTeleop, checker);
//        led_bank = new LedColor(op); //LED has to be declared before calling it
        turret = new Turret(op, led_bank, isTeleop, checker);
        openCV = new OpenCVMasterclass(op);
        TSE = new tseDepositor(op, isTeleop);

    }

    public int BlueElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.BlueTeamElem();
    }

    public int RedElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        return openCV.RedTeamElem();
    }

    public void flipBasketArmToPosition(double position) {
        turret.FlipBasketArmToPosition(position);
    }

    public void flipBasketToPosition(double position) {
        turret.FlipBasketToPosition(position);
    }

    public void flipBasket() {
        turret.FlipBasket(0);
    }

    public void flipBasketArmHigh() {
        turret.FlipBasketArmHigh();
    }

    public double[] BlueWarehouseScam() {
        return openCV.BlueWarehouseScam();
    }

    public void stopAllMotors() {
        drivetrain.stopAllMotors();
    }


    /*/******** Left Front Motor **********/
   /* public void moveMotorLeftFront(double distance) {
        drivetrain.moveMotorLeftFront(distance);
    }

    /******** Right Front Motor **********/
   /* public void moveMotorRightFront(double distance) {
        drivetrain.moveMotorRightFront(distance);
    }

    /******** Left Back Motor **********/
    /*public void moveMotorLeftBack(double distance) {
        drivetrain.moveMotorLeftBack(distance);
    }

    /******** Right Back Motor **********/
    /*public void moveMotorRightBack(double distance) {
        drivetrain.moveMotorRightBack(distance);
    }*/

    /******** shooterMotor **********/
    /*public void moveShooterMotor(int distance, int power) {
        shooter.moveShooterMotor(distance, power);
    }
    public double getAngle() {
        return drivetrain.getAngle();
    }


    /**
     * Directional Movement
     **/
    public void moveMultidirectional(double power, double angle, float rightStick, boolean isSlow) {
        drivetrain.moveMultidirectional(power, angle, rightStick, isSlow);
    }

    public void moveMultidirectionalMecanum(double power, float leftStickx, float leftSticky, float rightStick) {
        drivetrain.moveMultidirectionalMecanum(power, leftStickx, leftSticky, rightStick);
    }

    public void goToPositionWithoutStop(int direction, double yPosition, double xPosition, double power) {
        drivetrain.goToPositionWithoutStop(direction, yPosition, xPosition, power);
    }

    public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power, double targetAnglu) {
        drivetrain.tripleSplineToPosition(direction, x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, power, targetAnglu);
    }

    public void tripleSplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power) {
        drivetrain.tripleSplineToPositionHead(direction, x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, power);
    }

    public void partOfPolySplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power, double targetAnglu) {
        drivetrain.partOfPolySplineToPosition(direction, x0, y0, x1, y1, x2, y2, x3, y3, start, end, targetAnglu, power);
    }

    public void partOfPolySplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power) {
        drivetrain.partOfPolySplineToPositionHead(direction, x0, y0, x1, y1, x2, y2, x3, y3, start, end, power);
    }

    public void autonomousBarrierPathWarehouseToShared(double barrierX, double barrierY, double power) {
        double targetY = barrierY - 20;
        double targetX = barrierX;
        double[] currentPos = {xpos, ypos, angle};
        double maxAngle = 50;
        double positionAngle = atan2(xpos - barrierX, ypos - barrierY);
        if (maxAngle > positionAngle) {
            drivetrain.partOfPolySplineToPositionHead(0, currentPos[0], currentPos[1], currentPos[0], currentPos[1], barrierX, barrierY, targetX, targetY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0, currentPos[0], currentPos[1], barrierX, barrierY, targetX, targetY, targetX - 10, targetY, true, true, power);
        } else {
            double[] position = {(barrierY * tan(50) + currentPos[1] / tan(50) - barrierX + currentPos[0]) / (tan(50) + 1 / tan(50)), ((barrierY * tan(50) + currentPos[1] / tan(50) - barrierX + currentPos[0]) / (tan(50) + 1 / tan(50)) - barrierY) * tan(50) + barrierX};
            double[] newPosition = {(4 * position[0] + barrierX) / 5, (4 * position[1] + barrierY) / 5};
            drivetrain.partOfPolySplineToPositionHead(0, currentPos[0], currentPos[1], currentPos[0], currentPos[1], newPosition[0], newPosition[1], barrierX, barrierY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0, currentPos[0], currentPos[1], newPosition[0], newPosition[1], barrierX, barrierY, targetX, targetY, true, true, power);
            drivetrain.partOfPolySplineToPositionHead(0, newPosition[0], newPosition[1], barrierX, barrierY, targetX, targetY, targetX - 10, targetY, true, true, power);
        }
    }

    public void teleopLoop(int red, double startx, double starty) {
        /** gamepad 1**/
        isBlue=red;
        startAngle=trueStartAngle*red;
        float forward = -op.gamepad1.left_stick_y;
        float strafe = -op.gamepad1.left_stick_x; //remove dis boi son// DIY!
        float turning = -op.gamepad1.right_stick_x;
        boolean startIntake = op.gamepad1.right_bumper;
        boolean reverseIntake = op.gamepad1.y;
        boolean flipIntake = op.gamepad1.left_bumper;
//        boolean extendTSE = op.gamepad2.dpad_right;
//        boolean manualretractTSE = op.gamepad2.dpad_left;
//        boolean autoretractTSE = op.gamepad1.x;
        boolean sequence = op.gamepad1.dpad_up;
        boolean TSEArmUp = op.gamepad1.dpad_up;
        /** gamepad 2**/
        float turretTurn = op.gamepad2.left_stick_x;
        float turretUpNDown = op.gamepad2.right_stick_y;
        float extendTurret = -op.gamepad2.left_stick_y;
//        boolean extendAutoTSE = op.gamepad2.dpad_up;
        boolean autoretractTurret = op.gamepad2.y;
        boolean basketArm = op.gamepad2.right_bumper;
        boolean autoAim = op.gamepad2.left_bumper;
        boolean basket = op.gamepad2.a;
        boolean resetTurret = op.gamepad2.dpad_down;
        boolean plusTurret = op.gamepad2.dpad_up;
        boolean unsave_turret_position = op.gamepad2.b;
        boolean capper = false;


        time = op.getRuntime();
        //according to blue side left auto next to barrier
        changed = false;
        int up = 0;
        intake.updateIntakeStates();
        turret.updateTurretPositions();
//        if (xpos > (60) * red - startx) {
//            if (!shared_shipping_hub && red == 1) {
//                changed = true;
//            }
//            if (!alliance_shipping_hub && red == -1) {
//                changed = true;
//            }
//            if (red == 1) {
//                shared_shipping_hub = true;
//                alliance_shipping_hub = false;
//            } else {
//                shared_shipping_hub = false;
//                alliance_shipping_hub = true;
//            }
//            up = 0;
//        } else if (xpos < (60) * red - startx) {
//            if (!shared_shipping_hub && red == -1) {
//                changed = true;
//            }
//            if (!alliance_shipping_hub && red == 1) {
//                changed = true;
//            }
//            if (red == 1) {
//                alliance_shipping_hub = true;
//                shared_shipping_hub = false;
//            } else {
//                alliance_shipping_hub = false;
//                shared_shipping_hub = true;
//            }
//            up = 1;
//        }

        if (changed) {
            if (!checker.getState(StateMachine.States.BASKET_ARM_REST)) {
                    turret.FlipBasketArmToPosition(.45);
            }
            //according to blue side left auto next to barrier
        }
        if (isExtended) {
            retracting = false;
            autoretractTurret = false;
            autoAim = false;
            autoAiming = false;
        }
        if(autoAiming){
            turret_angle_control_distance=17*118.0/270.0/35.0;
        }
        if(retracting){
            turret_angle_control_distance=0;
        }
        turret_angle_control_distance -= turretUpNDown / 300;
        if (turret_angle_control_distance > 1) {
            turret_angle_control_distance = 1;
        }
        if (turret_angle_control_distance < 0) {
            turret_angle_control_distance = 0;
        }
        op.telemetry.addData("diSTANCE", turret_angle_control_distance);

        if (startIntake && shouldIntake) {
            intake.startIntake();
        } else if (isReversing) {
        } else {
            stopIntake();
        }

        if (reverseIntake && shouldIntake || isReversing) {
            op.telemetry.addData("reversing", "intake");
            intake.reverseIntake(0.7);
        }

        if (flipIntake && time > startTime[3] + .3) {
            startTime[3] = time;
            if (!intake.flipIntake()) {
                retracting = true;
            }
        }
        if(resetTurret){
            turret.resetExtension();
            op.sleep(200);
        }
        if(plusTurret){
            turret.plusExtension();
            op.sleep(200);
        }
        if (turretTurn != 0&&!retracting) {
            autoAiming = false;
            TurretManualRotation(turretTurn);
        } else if (!retracting && !autoAiming && time > startTime[8] + 2) {
            turret.stopTurn();
        }
//        if (extendAutoTSE) {
//            startTime[6] = op.getRuntime();
////            TSE.setTseCrServoPower(1.0);
//            isExtending = true;
//            isExtended = true;
//        }
        if (capper && time > startTime[8] + 0.4) {
            capThats();
            startTime[8] = time;
        }
        if (op.getRuntime() > .147 * 44 + startTime[6]) {
            isExtending = false;
        }
        if (extendTurret != 0 || retracting) {
            autoAiming = false;
            TurretManualExtension(extendTurret);
        } else if (time > startTime[8] + 2.0) {
            turret.stopExtend();
            if (outModed) {
                turret.runTurretWithoutEncoder();
                outModed = false;
            }
        } else if (time < startTime[8] + 2.0) {
            outModed = true;
        }

        if (turretUpNDown != 0) {
            autoAiming = false;
            TurretAngleControlRotating(turret_angle_control_distance);
        }

        if (autoAim) {
            if (op.getRuntime() > startTime[2] + 0.5) {
//                autoAiming = !autoAiming;
                startTime[2] = op.getRuntime();
                TurretStop();
            }

            op.telemetry.addData("robot position", Arrays.toString(new double[]{xpos, ypos, angle}));
        }
        spinCarousel();
        if (autoAiming && !retracting) {
            intake.flipIntakeToPosition(0.0);
            fakeAutoAim();
        }
        op.telemetry.addData("autoaiming", autoAiming);
        op.telemetry.addData("up", up);

        if (basket && time > startTime[4] + .3) {
            startTime[4] = time;
            FlipBasket(up);
            op.sleep(200);
            SavePosition(up);
        }
        if (basketArm) {
            if (time > startTime[5] + 0.3) {
                startTime[5] = time;
                turret.FlipBasketArmHigh();
            }
        }

        if (TSEArmUp && time > startTime[12] + 0.2) {
            startTime[12] = time;
            TSE.toggleTSEPosition();
            intake.flipIntakeToPosition(.79);
        }


//        if (extendTSE) {
//            TSE.setTseCrServoPower(1);
//        }
//        if (!extendTSE && !manualretractTSE && !autoretractTSE && !isExtending) {
//            TSE.setTseCrServoPower(0.0);
//        }
//
//        if (manualretractTSE) {
//            TSE.setTseCrServoPower(-1);
//        }
//
//        if (autoretractTSE) {
//            //insert set tse to 0 function
//        }

        /** turret stuff **/


        if (unsave_turret_position) {
            UnsavePosition();
        }

        /** insert basket stuff **/
        /** insert basket stuff **/


//            if (slowDown) {
//                if(slowMode){
//                    slowMode=false;
//                }
//                else{
//                    slowMode=true;
//                }
//
//                slowMode = !slowMode;
//                op.sleep(150);
//            }

        /** add stuff u want to do with intake when switch is on HERE **/
        if ((checker.getState(StateMachine.States.SEQUENCING) || checker.checkIf(StateMachine.States.SEQUENCING)) && !retracting) {
            op.telemetry.addData("el button", "is clicked");
            isFlipping = true;
            if (!checker.getState(StateMachine.States.SEQUENCING)) {
                startTime[0] = op.getRuntime() + 9;
                startTime[1] = op.getRuntime() + 10;
                turret.FlipBasketToPosition(.87);
                intake.startIntake();
            }
            checker.setState(StateMachine.States.SEQUENCING, true);
            turret.FlipBasketArmToPosition(0.00);
            if (checker.checkIf(StateMachine.States.FLIPPING) && checker.getState(StateMachine.States.INTAKE_DOWN)) {
                intake.startIntake();
                intake.flipIntakeToPosition(0.77);
                turret.FlipBasketToPosition(.87);
                isBall = intake.isBall();
                startTime[0] = op.getRuntime();
                startTime[1] = op.getRuntime() + 10;
            } else if (!checker.getState(StateMachine.States.FLIPPING) && checker.getState(StateMachine.States.INTAKE_DOWN)) {
                retracting = true;
            }
            op.telemetry.addData("transferring", !checker.getState(StateMachine.States.TRANSFERRING));
            op.telemetry.addData("flipping", !checker.getState(StateMachine.States.FLIPPING));
            op.telemetry.addData("straig", checker.getState(StateMachine.States.TURRET_STRAIGHT));
            op.telemetry.addData("extend", !checker.getState(StateMachine.States.EXTENDED));
            op.telemetry.addData("raise", !checker.getState(StateMachine.States.RAISED));
            op.telemetry.addData("basket", checker.getState(StateMachine.States.BASKET_TRANSFER));
            op.telemetry.addData("intakedown", !checker.getState(StateMachine.States.INTAKE_DOWN));
            if (op.getRuntime() > startTime[0] + 0.5 && !checker.getState(StateMachine.States.INTAKE_DOWN)&&!isReversing) {
               stopIntake();
            }

            if (op.getRuntime() > startTime[0] + 0.8 && !checker.getState(StateMachine.States.INTAKE_DOWN)&&!checker.getState(StateMachine.States.TRANSFERRING)) {
                checker.setState(StateMachine.States.TRANSFERRING, true);
                isReversing = true;
                op.telemetry.addData("reversing ", "intake");
                startTime[1] = op.getRuntime();
                if(!isBall) {
                    intake.reverseIntake(0.63);
                }
                else{
                    startTime[1]-=0.1;
                    intake.reverseIntake(0.53);
                }
            }

            if (op.getRuntime() > startTime[1] + 0.4) {
                isReversing = false;
                isFlipping = false;
                flipped = false;
                intake.stopIntake();
                turret.FlipBasketToPosition(.6);
//                turret.FlipBasketArmToPosition(.3);
                autoAiming = true;
                checker.setState(StateMachine.States.TRANSFERRING, false);
                checker.setState(StateMachine.States.SEQUENCING, false);
            }

        }
        if (autoretractTurret || retracting) {
            autoAiming = false;
            if (!checker.getState(StateMachine.States.INTAKE_DOWN)&&abs(startAngle-EncoderChassis.angle)<45) {
                intake.flipIntakeToPosition(0.0);
            }
            retracting = TurretReset(0.5);
        }

        magnitude = forward;
//        if(ypos<20||angle>30||magnitude<0) {
            drivetrain.moveMultidirectional(magnitude, angleInDegree, turning, slowMode); // It is 0.95, because the robot DCs at full power.
//        }
//        else{
//            drivetrain.setRightMotorVelocities(pow(48-EncoderChassis.ypos,1/2.0)/4.46*30*29.8 + angle *20);
//            drivetrain.setLeftMotorVelocities(pow(48-EncoderChassis.ypos,1/2.0)/4.46*30*29.8- angle *20);
//        }
        }

    public void mazeTeleopLoop() {


        /** gamepad 1**/
        float forward = -op.gamepad1.left_stick_y;
        float strafe = -op.gamepad1.left_stick_x; //remove dis boi son
        float turning = -op.gamepad1.right_stick_x;


        magnitude = forward;
        drivetrain.moveMultidirectionalMecanum(0.9, strafe, forward, turning); // It is 0.95, because the robot DCs at full power.
    }

    public void autoAim(double[][] turret_saved_positions, int red) {
        double angle = 180 - Math.atan2(-(turret_saved_positions[1][0] - xpos), (turret_saved_positions[1][1] - ypos)) * 180 / PI - VSLAMChassis.angle;
        angle %= 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        turret.TurretRotate(angle);
        op.telemetry.addData("angle", angle);
        op.telemetry.addData("trutx", turret_saved_positions[1][0]);
        op.telemetry.addData("truty", turret_saved_positions[1][1]);

        double turret_angle_control_pos = Math.atan2(turret_saved_positions[1][2], Math.sqrt(Math.pow(xpos - turret_saved_positions[1][0] * red, 2) + Math.pow(ypos - turret_saved_positions[1][1], 2)));
        turret.AutoAngleControlRotating(turret_angle_control_pos);
    }

    public void fakeAutoAim() {
        double angle = -60;
        retracting=false;
        if(abs(startAngle-EncoderChassis.angle)%360<45) {
            if(!flipped) {
                flipBasketArmToPosition(0.55);
                flipped=true;
                shareFlipTime = op.getRuntime();
            }
            if(op.getRuntime()-shareFlipTime>0.3) {
                if (!isBall) {
                    turret.TurretRotate(turret_saved_positions[0][0][1] * isBlue);
                    turret.AutoAngleControlRotating(17);
                    if (abs(turret_saved_positions[0][0][1] * isBlue - rotatePosition) < 200||isBlue==-1) {
                        turret.turretExtendo(turret_saved_positions[0][0][0]);
                    }
                } else {
                    turret.TurretRotate(turret_saved_positions[0][1][1] * isBlue);
                    turret.AutoAngleControlRotating(0);
                    if (abs(turret_saved_positions[0][1][1] * isBlue - rotatePosition) < 200||isBlue==-1) {
                        turret.turretExtendo(turret_saved_positions[0][1][0]);
                    }
                }
            }
        }
        else{
            if(time-shareFlipTime>3.0&&checker.getState(StateMachine.States.BASKET_ARM_REST)&&!flipped) {
                flipBasketArmToPosition(0.25);
                shareFlipTime=time;
                flipped = true;
            }
            if(checker.getState(StateMachine.States.INTAKE_DOWN)){
                flipIntakeToPosition(0.76);
            }
            if(time-shareFlipTime>0.1){
                turret.TurretSlotate(turret_saved_positions[0][2][1]*isBlue);
                turret.AutoAngleControlRotating(0);
                turret.turretExtendo(turret_saved_positions[0][2][0]);
            }
        }
    }

    public void rotateToPosition(double targetAngle) {
        turret.rotateToPosition(targetAngle);
    }

    public void capThats() {
        turret.capBasket();
    }

    public void updateTurretPositions() {
        turret.updateTurretPositions();
    }

    public void TurretExtend(double height_inches, double extension_inches, double power) {
        turret.TurretExtend(height_inches, extension_inches, power);
    }

    public void TurretExtendSimple(int extension_inches, double power) {
        turret.TurretExtendSimple(extension_inches, power);
    }

    public void TurretRotate(double targetAngle) {
        TurretRotate(targetAngle);
    }

    public void BasketArmFlipLowExtend(double power) {
        turret.BasketArmFlipLowExtend(power);
    }

    public void BasketArmFlipHighExtend(double power) {
        turret.BasketArmFlipHighExtend(power);
    }

    public void TurretAngleControlRotating(double target_point) {
        turret.TurretAngleControlRotating(target_point);
    }

    public void TurretSlidesToPosition(double x, double y, double z, double power, boolean retract) {
        turret.TurretSlidesToPosition(x, y, z, power, retract);
    }

    public void setMotorPowers(double power) {
        drivetrain.setMotorPowers(power);
    }

    public void TurretManualRotation(double rotation) {
        turret.TurretManualRotation(rotation);
    }

    public void TurretManualExtension(double turret_extension) {
        turret.TurretManualExtension(turret_extension);
    }

    public void turretManualElevation(double elevation) {
        turret.TurretManualElevation(elevation);
    }

    public void TurretManualFlip() {
        turret.TurretManualFlip();
    }

    public void FlipBasketArmLow() {
        turret.FlipBasketArmLow();
    }

    public void FlipBasketToPosition(double torget) {
        turret.FlipBasketToPosition(torget);
    }

    public void FlipBasketArmHigh() {
        turret.FlipBasketArmHigh();
    }

    public void FlipBasketArmToPosition(double torget) {
        turret.FlipBasketArmToPosition(torget);
    }

    public void FlipBasket(int up) {
        turret.FlipBasket(up);
    }

    public void SavePosition(int up) {
        turret.SavePosition(up);
    }

    public void UnsavePosition() {
        turret.UnsavePosition();
    }

    public boolean TurretReset(double power) {
        return turret.TurretReset(power);
    }

    public void TurretStop() {
        turret.TurretStop();
    }


    public void autoAim(double shipping_hub_x, double shipping_hub_y) {
        double[] robot_position = {xpos, ypos, angle};
        double turret_relative_rotation_angle = robot_position[2] - extendPosition;
        double turret_target_rotation_angle = atan((robot_position[0] - shipping_hub_x) / (robot_position[1] - shipping_hub_y)) + turret_relative_rotation_angle;
        turret.TurretRotate(turret_target_rotation_angle);
    }

    public void AngleControlRotation(double degrees) {
        turret.AngleControlRotating(degrees);
    }

    public void spinCarousel() {
        rotation.spinCarousel();
    }

    public void spinCarouselAutonomousBlue() {
        rotation.spinCarouselAutonomousBlue();
    }

    public void spinCarouselAutonomousRed() {
        rotation.spinCarouselAutonomousRed();
    }

    public void startIntake() {
        intake.startIntake();
    }

    public void reverseIntake(double power) {
        intake.reverseIntake(power);
    }

    public void stopIntake() {
        intake.stopIntake();
    }

    public void setPosition(float xPosition, float yPosition, float newAngle) {
        drivetrain.setPosition(xPosition, yPosition, newAngle);
    }

    public void aimHigh() {
        turret.aimHigh();
    }

    public double[] track() {
        return drivetrain.track();
    }//track[0] is y value, track[1] is x value, and track[2] is angle

    public void goToPosition(int direction, double yPosition, double xPosition, double newAngle, double power) {
        drivetrain.goToPosition(direction, yPosition, xPosition, newAngle, power);
    }

    public boolean goToPositionTeleop(int direction, double yPosition, double xPosition, double power) {
        return drivetrain.goToPositionTeleop(direction, yPosition, xPosition, power);
    }

    public void flipIntakeToPosition(double torget) {
        intake.flipIntakeToPosition(torget);
    }

    public void turnInPlace(double target, double power) {
        drivetrain.turnInPlace(target, power);
    }

    public void toggleTSEPosition() {
        TSE.toggleTSEPosition();
    }
    public void tseToPosition(double position){TSE.tseToPosition(position);}

    public boolean autoIntake(double power, double randRange, double times) {
        resetten = false;
        faked = false;
        double[] thoseCurves = {0, 10, 12, 15, 5, 12, 8, 8};
        double[] whereTonext = {.33, .66, 1.5, 1.8, 2.1, 2.4};
        double starterTime = op.getRuntime();
        boolean block = false;
        intake.flipIntakeToPosition(0.0);
        intake.startIntake();
        turret.runTurretWithoutEncoder();
        track();
        double angleDiff = -angle;
        drivetrain.turnInPlace(-atan2(EncoderChassis.xpos, 20) * 180 / PI, 1.0);
        while (!block && op.getRuntime() < 27) {
            starterTime = op.getRuntime();
            double time = op.getRuntime();

            while (time - starterTime < 1.85 + times / 10) {
                if(ypos<15){
                    angleDiff=-atan2(EncoderChassis.xpos+2, 15-ypos) * 180 / PI-angle;
                }
                else {
                    angleDiff = -angle;
                }
                drivetrain.track();
                turret.updateTurretPositions();
                time = op.getRuntime();
                if (!resetten) {
                    boolean isReset = turret.TurretReset(1.0);
                } else if (resetten) {
                    turret.stopExtend();
                    turret.stopTurn();
                }
                if (resetten && time < startTime[9]) {
                    turret.FlipBasketToPosition(.88);
                    turret.FlipBasketArmToPosition(0.03);
                    startTime[9] = time;
                }
                if (ypos < 15) {
                    drivetrain.setRightMotorVelocities(pow(45-EncoderChassis.ypos,1/3.0)/2.7*40*29.8 - angleDiff *20);
                    drivetrain.setLeftMotorVelocities(pow(45-EncoderChassis.ypos,1/3.0)/2.7*40*29.8+ angleDiff *20);
//                    drivetrain.setRightMotorPowers(abs(pow((33 + 0.5 * times - ypos) / (35 + 0.5 * times), 2)) - angleDiff / 100 + 0.1);
//                    drivetrain.setLeftMotorPowers(abs(pow((33 + 0.5 * times - ypos) / (35 + 0.5 * times), 2)) + angleDiff / 100 - 0.1);
                } else if(ypos<45){
                    drivetrain.setRightMotorVelocities(pow(45-EncoderChassis.ypos,1/3.0)/2.7*40*29.8 - (angleDiff + thoseCurves[(int) times]) *20);
                    drivetrain.setLeftMotorVelocities(pow(45-EncoderChassis.ypos,1/3.0)/2.7*40*29.8+ (angleDiff + thoseCurves[(int) times]) *20);
//                    drivetrain.setRightMotorPowers(abs(pow((33 + 0.5 * times - ypos) / (35 + 0.5 * times), 2)) - (angleDiff + thoseCurves[(int) times]) / 25);
//                    drivetrain.setLeftMotorPowers( abs(pow((33 + 0.5 * times - ypos) / (35 + 0.5 * times), 2)) + (angleDiff + thoseCurves[(int) times]) / 25);
                }
                else{
                    drivetrain.setRightMotorVelocities((50- EncoderChassis.ypos)*29.8);
                    drivetrain.setLeftMotorVelocities((50-EncoderChassis.ypos)*29.8);
                }
                if (intake.isSwitched()) {
                    drivetrain.setRightMotorVelocities(-100);
                    drivetrain.setLeftMotorVelocities(-100);
                    if (time > 26) {
                        stopIntake();
                        return false;
                    }
                    sheeeeesh(0.0, 4, 0.8, 0);
                    block = true;
                    break;
                }
            }
            if (!block) {
//                turret.TurretReset(0.5);
                drivetrain.setMotorPowers(-0.4);
                op.sleep(500);
                thoseCurves[(int) times] = 0.5 + random() * randRange;

                drivetrain.turnInPlace(thoseCurves[(int) times], 0.5);
            }
        }
        stopAllMotors();
        return block;
    }

    public void sheeeeesh(double x, double y, double power, int direction) {
        double[][] point = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
        double[] currentPosition = track();
        point[0][0] = currentPosition[0];
        point[0][1] = currentPosition[1];
        point[1][0] = currentPosition[0];
        point[1][1] = currentPosition[1];
        point[2][0] = 0.0;
        point[2][1] = 15;
        point[3][0] = x;
        point[3][1] = y;
        point[4][0] = x;
        point[4][1] = y;
        double startpower = power;
        boolean autoAiming = false;
        double[] startPosition = currentPosition;
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double tt = 0;
        double difference = 0;
        double error = 0;
        double powerconst = 1, targetspeed = 0;
        double mpconst = 0;
        double p = .2;
        double pd = .2;
        double D = .02;
        double I = 0;
        double[] tarcurpos = {0, 0, 0};
        double xError = 0, posxError = 0, yError = 0, posyError = 0, xCorrection = 0, yCorrection = 0, t = 0, yInt = 0, xInt = 0, angleConst;
        double lastAngle = track()[2], anglediff = 0, targetaVelocity = 0;
        double startTime = 400;
        boolean reversing = false;
        double[] starterTimes = {100, 100, 100, 100, 100, 100};
        turret.runTurretWithoutEncoder();
        intake.flipIntakeToPosition(0.77);
        starterTimes[4] = op.getRuntime();
        boolean resetting = false;
        for (int i = 0; i < 2; i++) {
            target_position[0] = point[i + 2][0];
            target_position[1] = point[i + 2][1];
            target_position[2] = 0;
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
            t = 0;
            tt = 0;
            double t2 = 0;
            targetspeed = startpower * 30;
            double xDerivative = 0, yDerivative = 0;
            boolean td = true;
            while (op.opModeIsActive() && (abs(difference) >= 2.5)&&point[3][1]<ypos && op.getRuntime() < 29.8) {
                turret.updateTurretPositions();
                double nowTime = op.getRuntime();
                if (!resetten && !autoAiming && starterTimes[1] == 100) {
                    turret.TurretReset(0.5);
                    resetting = true;
                } else if (resetten && resetting) {
                    resetting = false;

                    turret.stopExtend();
                    turret.stopTurn();
                    turret.FlipBasketToPosition(.88);
                    turret.FlipBasketArmToPosition(0.03);
                }
                if (resetten && starterTimes[0] == 100) {
                    starterTimes[0] = nowTime;
                }
                if (nowTime - starterTimes[0] > .2 && resetten && nowTime - starterTimes[4] > 0.74) {
                    intake.reverseIntake(1.0);
                    reversing = true;
                    starterTimes[0] = 500;
                    starterTimes[1] = nowTime;
                }
                if (reversing) {
                    intake.reverseIntake(1.0);
                }
                if (nowTime - starterTimes[1] > 0.4) {
                    intake.flipIntakeToPosition(0.2);
                    turret.FlipBasketToPosition(.58);
                    starterTimes[1] = 101;
                    starterTimes[2] = nowTime;
                }
                if (nowTime - starterTimes[2] > .1) {
                    turret.FlipBasketArmToPosition(.55);
                    intake.stopIntake();
                    reversing = false;
                    starterTimes[2] = 100;
                    autoAiming = true;
                    starterTimes[5] = nowTime;
                }
                if (ypos < 5) {
                    if (autoAiming) {
                        fakeAutoAim();
                        if (faked && rotated) {
                            turret.stopExtend();
                            turret.stopTurn();
                            starterTimes[1] = 501;
                        }
                        stopIntake();
                    }
                }
                op.telemetry.addData("autoAim", autoAiming);
                op.telemetry.addData("resetten", resetten);
                op.telemetry.addData("start0", starterTimes[0]);
                op.telemetry.addData("start1", starterTimes[1]);
                currentPosition = track();
                double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                if ((oneDistance + Velocity / 5 + 1.0 / 4.0) / (oneDistance + twoDistance) > t && td) {
                    t = (oneDistance + Velocity / 5 + 1.0 / 4.0) / (oneDistance + twoDistance);
                    t2 = (oneDistance + Velocity / 5 + 1.0 / 4.0) / (oneDistance + twoDistance);
                }
                if (tt >= 1) {
                    if (op.getRuntime() < startTime) {
                        startTime = op.getRuntime();
                    }
                    if (op.getRuntime() > startTime + 1.5 / power) {
                        break;
                    }
                    tt = 1;
                }
                if (t >= 1) {
                    t = 1;
                }
                if (!td) {
                    t = 1;
                    td = true;
                }
                target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t2 + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t2, 2) +
                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t2, 3));

                target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t2 + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t2, 2) +
                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t2, 3));
                xDerivative = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                        3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)));
                yDerivative = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                if (yDerivative == 0) {
                    yDerivative = 0.00001;
                }
                if (xDerivative == 0) {
                    xDerivative = 0.00001;
                }

                mpconst = yDerivative / xDerivative;

                target_position[2] = atan2(xDerivative, yDerivative) * 180 / PI + (direction - 1) * 180;
                anglediff = target_position[2] - lastAngle;
                targetaVelocity = anglediff / differtime;
                lastAngle = target_position[2];
                if (target_position[2] > 180) {
                    target_position[2] -= 360;
                }
                if (target_position[2] < -180) {
                    target_position[2] += 360;
                }
                op.telemetry.addData("angleTarget", target_position[2]);
                if ((oneDistance + 1.0 / 4.0) / (oneDistance + twoDistance) > tt) {
                    tt = (oneDistance + 1.0 / 4) / (oneDistance + twoDistance);
                }
                if (tt > 1) {
                    tt = 1;
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * tt + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(tt, 2) +
                        (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(tt, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * tt + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(tt, 2) +
                        (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(tt, 3));
                double txDerivative = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * tt +
                        3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(tt, 2)));
                double tyDerivative = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * tt +
                        3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(tt, 2)));
                tarcurpos[2] = atan2(txDerivative, tyDerivative) * 180 / PI - (direction - 1) * 180;
                if (tyDerivative == 0) {
                    tyDerivative = 0.00001;
                }
                if (txDerivative == 0) {
                    txDerivative = 0.00001;
                }
                double mpconst2 = tyDerivative / txDerivative;
                if (mpconst == 1) {
                    mpconst = 1.001;
                }
                if (mpconst2 == 1) {
                    mpconst2 = 1.001;
                }
                if (xVelocity == 0) {
                    xVelocity = 0.0001;
                }
                double targetXVelocity = txDerivative / abs(txDerivative) * ((sqrt(pow(startpower * 30, 2) / abs(1 - pow(mpconst2, 2)))));
                double targetYVelocity = targetXVelocity * mpconst2;
                xError = targetXVelocity - xVelocity;
                posxError = tarcurpos[0] - currentPosition[0];
                yError = targetYVelocity - yVelocity;
                posyError = (tarcurpos[1] - currentPosition[1]);
                targetXVelocity = xDerivative / abs(xDerivative) * ((sqrt(pow(startpower * 30, 2) / abs(1 - pow(mpconst, 2)))));
                targetYVelocity = targetXVelocity * mpconst;

                xCorrection = pd * xError + p * posxError + I * xInt + D * (xError + posxError / differtime);
                yCorrection = pd * yError + p * posyError + I * yInt + D * (yError + posyError / differtime);
                x = point[i + 2][0] - currentPosition[0];
                y = point[i + 2][1] - currentPosition[1];
                if (x == 0) {
                    x = 0.0001;
                }
                if (y == 0) {
                    y = 0.0001;
                }
                if (targetspeed == 0) {
                    targetspeed = 0.01;
                }
                if (xCorrection == 0) {
                    xCorrection = 0.001;
                }
                if (yCorrection == 0) {
                    yCorrection = 0.001;
                }

                target_position[2] = (atan2(targetXVelocity + xCorrection, targetYVelocity + yCorrection) * 180 / PI) + (direction - 1) * 180;
                target_position[2] %= 360;
                if (target_position[2] > 180) {
                    target_position[2] -= 360;
                }
                if (target_position[2] < -180) {
                    target_position[2] += 360;
                }
                error = currentPosition[2] - target_position[2];
                error %= 360;
                if (error > 180) {
                    error -= 360;
                }
                if (error < -180) {
                    error += 360;
                }
                op.telemetry.addData("error", error);
                double error2 = currentPosition[2] - atan2(x, y) * 180 / PI + (direction - 1) * 180;
                error2 %= 360;
                if (error2 > 180) {
                    error2 -= 360;
                }
                if (error2 < -180) {
                    error2 += 360;
                }
                double controlconst = pow(tt, 1.5);
                op.telemetry.addData("erro2r", error2);
                error = controlconst * error2 + ((1 - controlconst) * error);
                error %= 360;
                if (error > 180) {
                    error -= 360;
                }
                if (error < -180) {
                    error += 360;
                }
                targetaVelocity -= 2 * (error);
                op.telemetry.addData("targetAVelocity", targetaVelocity);
                anglecorrection = (error * 2 + (-targetaVelocity + aVelocity) * .2) / 216;
                double powernum = pow(E, -10 * (tan((abs(error / 12) % 15) * PI / 180)));
                if (powernum == -1) {
                    powernum = -1.0001;
                }
                if (Double.isNaN(powernum)) {
                    powernum = 99999;
                }
                if (error < 180 && error > -180) {
                    power = startpower * (3 - (1 / (1 + powernum)) * 4);
                } else {
                    power = startpower * (-2.75 + (1 / (1 + powernum)) * 4);
                }
                if (direction == 0) {
                    power *= -1;
                }
                if(difference<10){
                    powerconst=pow(difference,0.5);
                }
                op.telemetry.addData("t", t);
                op.telemetry.addData("error", error);
                op.telemetry.addData("targetXPosition", target_position[0]);
                op.telemetry.addData("targetYPosition", target_position[1]);
                drivetrain.setRightMotorPowers((powerconst) * power + anglecorrection);
                drivetrain.setLeftMotorPowers((powerconst) * power - anglecorrection);
                op.telemetry.addData("t", t);
                op.telemetry.addData("error", error);
                op.telemetry.addData("targetXPosition", target_position[0]);
                op.telemetry.addData("targetYPosition", target_position[1]);

                difference = sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2));
            }
        }
        stopAllMotors();
        double nowTime = op.getRuntime();
        while (op.opModeIsActive() && nowTime < 29.8 && starterTimes[1] < 500 && !faked || !rotated) {
            nowTime = op.getRuntime();
            turret.updateTurretPositions();
            if (!resetten && !autoAiming && starterTimes[0] == 100) {
                turret.TurretReset(0.5);
                resetting = true;
            } else if (resetten && resetting) {
                resetting = false;
                turret.stopExtend();
                turret.stopTurn();
                turret.FlipBasketToPosition(.9);
                turret.FlipBasketArmToPosition(0.03);
            }
            if (resetten && starterTimes[0] == 100) {
                starterTimes[0] = nowTime;
            }
            if (nowTime - starterTimes[0] > .2 && resetten && nowTime - starterTimes[4] > 0.74) {
                intake.reverseIntake(0.77);
                starterTimes[0] = 500;
                reversing = true;
                starterTimes[1] = nowTime;
            }
            if (reversing) {
                intake.reverseIntake(0.6);
            }
            if (nowTime - starterTimes[1] > 0.4) {
                intake.flipIntakeToPosition(0.2);
                turret.FlipBasketToPosition(.58);
                reversing = false;
                starterTimes[1] = 100;
                starterTimes[2] = nowTime;
            }
            if (nowTime - starterTimes[2] > .1) {
                turret.FlipBasketArmToPosition(.55);
                intake.stopIntake();
                starterTimes[2] = 100;
                autoAiming = true;
            }
            if (autoAiming) {
                fakeAutoAim();
                stopIntake();
                if (faked && rotated) {
                    turret.stopExtend();
                    turret.stopTurn();
                }
            }
            op.telemetry.addData("autoAim", autoAiming);
            op.telemetry.addData("faked", faked);
            op.telemetry.addData("resetten", resetten);
            op.telemetry.addData("start0", rotated);
            op.telemetry.addData("start1", starterTimes[1]);
            op.telemetry.update();
        }
    }

    public void autoPark(double x, double y, double power) {
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double truestartpower = power;
        target_position[0] = x;
        target_position[1] = y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        intake.flipIntakeToPosition(0.0);
        drivetrain.turnInPlace(0, power);
        while ((abs(difference) >= 4) && op.getRuntime() < 29.8) {
            turret.TurretReset(0.5);
            currentPosition = track();
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            drivetrain.setMotorPowers(power);
            difference = abs(sqrt(x * x + y * y));
        }
        stopAllMotors();
        while (op.getRuntime() < 29.95) {
            turret.TurretReset(1.0);
        }
        turret.stopTurn();
        turret.stopExtend();
    }

    /**
     * LEDs
     **/
    public void ledAmber(int led_number) {
        led_bank.LedAmber(led_number);
    }

    public void ledGreen(int led_number) {
        led_bank.LedGreen(led_number);
    }

    public void ledRed(int led_number) {
        led_bank.LedRed(led_number);
    }

    public void ledOff(int led_number) {
        led_bank.LedOff(led_number);
    }
}