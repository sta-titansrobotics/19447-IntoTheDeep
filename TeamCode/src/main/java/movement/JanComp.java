package movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class JanComp extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 0;
    int buttonY = 0;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;
    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    double prevtime;
    
    static double dir;
    static double mag;
    static double pi = Math.PI;

    int HLexttarg;
    int HRexttarg;
    int HLexttargfine;
    int HRexttargfine;
    double HRexterr;
    double HLexterr;
    double HLextpower;
    double HRextpower;
    double HRextpreverr;
    double HLextpreverr;

    double HrKp = 0.011;
    double HrKd = 0.001;
    double HlKp = 0.0125;
    double HlKd = 0.0015;

    int VLexttarg;
    int VRexttarg;
    int VLexttargfine;
    int VRexttargfine;
    double VRexterr;
    double VLexterr;
    double VLextpower;
    double VRextpower;
    double VRextpreverr;
    double VLextpreverr;

    double VKp = 0.01;
    double VKd = 0.0015;

    double rot;

    double offset = 0;
    double imureset = 0;

    double toepos = 0.16;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angle;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub 
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub 
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub 
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

        DcMotor HRext = hardwareMap.get(DcMotor.class, "hrext"); // Expantion hub
        DcMotor HLext = hardwareMap.get(DcMotor.class, "hlext"); // Expantion hub

        DcMotor VRext = hardwareMap.get(DcMotor.class, "vrext"); // Expantion hub
        DcMotor VLext = hardwareMap.get(DcMotor.class, "vlext"); // Expantion hub

        Servo claw = hardwareMap.get(Servo.class, "Claw");
        Servo toel = hardwareMap.get(Servo.class, "toel");
        Servo toer = hardwareMap.get(Servo.class, "toer");
        Servo Rint = hardwareMap.get(Servo.class, "rint");
        Servo Lint = hardwareMap.get(Servo.class, "lint");

        CRServo rrol = hardwareMap.get(CRServo.class, "rrol");
        CRServo lrol = hardwareMap.get(CRServo.class, "lrol");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        HRext.setDirection(DcMotorSimple.Direction.REVERSE);
        VRext.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rrol.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // ------------------PID-POWER---------------------------------

            // 4 stage sliders
            // limiting the motors movement so that it does not try to over extend the sliders
            HRexttarg = (int) clamp(HRexttarg, 10, 1950);
            HLexttarg = (int) clamp(HLexttarg, 10, 1950);

            HRexterr = HRexttarg - HRext.getCurrentPosition();
            HLexterr = HLexttarg - HLext.getCurrentPosition();

            // actual pd calculations
            HRextpower = HRexterr*HrKp+(HRexterr - HRextpreverr)*HrKd;
            HLextpower = HLexterr*HlKp+(HLexterr - HLextpreverr)*HlKd;

            // getting the previous error
            HRextpreverr = (HRexttarg - HRext.getCurrentPosition());
            HLextpreverr = (HLexttarg - HLext.getCurrentPosition());

            //to fix starting jitter
            if (HRext.getCurrentPosition() < 10 && HLext.getCurrentPosition() < 10 && HRexttarg == 10){
                HRextpower = 0;
                HLextpower = 0;
            }

            // actually setting the motor power
            HLext.setPower(clamp(HLextpower, -1, 1));
            HRext.setPower(clamp(HRextpower, -1, 1));

            if (gamepad2.y && !butYcheck) {
                buttonY += 1;
                butYcheck = true;
            }

            if (!gamepad2.y) {
                butYcheck = false;
            }

            if (butYcheck) {
                if (buttonY % 2 == 1) {
                    HRexttarg = 1900;
                    HLexttarg = 1900;
                } else {
                    HRexttarg = 10;
                    HLexttarg = 10;
                }
            }

            // 4 stage sliders
            // limiting the motors movement so that it does not try to over extend the sliders
            VRexttarg = (int) clamp(VRexttarg, 10, 3250);
            VLexttarg = (int) clamp(VLexttarg, 10, 3250);

            VRexterr = VRexttarg - VRext.getCurrentPosition();
            VLexterr = VLexttarg - VLext.getCurrentPosition();

            // actual pd calculations
            VRextpower = VRexterr*VKp+(VRexterr - VRextpreverr)*VKd;
            VLextpower = VLexterr*VKp+(VLexterr - VLextpreverr)*VKd;

            // getting the previous error
            VRextpreverr = (VRexttarg - VRext.getCurrentPosition());
            VLextpreverr = (VLexttarg - VLext.getCurrentPosition());

            //to fix starting jitter
            if (VRext.getCurrentPosition() < 10 && VLext.getCurrentPosition() < 10 && VRexttarg == 10){
                VRextpower = 0;
                VLextpower = 0;
            }

            // actually setting the motor power
            VLext.setPower(clamp(VLextpower, -1, 1));
            VRext.setPower(clamp(VRextpower, -1, 1));

            if (gamepad1.x && !butXcheck) {
                buttonX += 1;
                butXcheck = true;
            }

            if (!gamepad1.x) {
                butXcheck = false;
            }

            if (butXcheck) {
                if (buttonX % 2 == 1) {
                    VRexttarg = 3200;
                    VLexttarg = 3200;
                } else {
                    VRexttarg = 10;
                    VLexttarg = 10;
                }
            }

            if (gamepad2.a && !but2Acheck) {
                button2A += 1;
                but2Acheck = true;
            }

            if (!gamepad2.a) {
                but2Acheck = false;
            }

            if (but2Acheck && VRext.getCurrentPosition() > 1000) {
                if (button2A % 2 == 1) {
                    toepos = 0.5;
                } else {
                    toepos = 0.16;
                }
            }



            toer.setPosition(1-toepos);
            toel.setPosition(toepos);

            if (gamepad2.b && !but2Bcheck) {
                button2B += 1;
                but2Bcheck = true;
            }

            if (!gamepad2.b) {
                but2Bcheck = false;
            }

            if (but2Bcheck) {
                if (button2B % 2 == 1) {
                    Rint.setPosition(0.7);
                    Lint.setPosition(0.7);
                } else {
                    Rint.setPosition(0.025);
                    Lint.setPosition(0.025);
                }
            }

            if (gamepad2.x && !but2Xcheck) {
                button2X += 1;
                but2Xcheck = true;
            }

            if (!gamepad2.x) {
                but2Xcheck = false;
            }

            if (but2Xcheck) {
                if (button2X % 2 == 1) {
                    claw.setPosition(0.55);
                } else {
                    claw.setPosition(0);
                }
            }

            rrol.setPower((gamepad2.right_trigger)-gamepad2.left_trigger);
            lrol.setPower((gamepad2.right_trigger)-gamepad2.left_trigger);


            // ------------------DRIVE TRAIN---------------------------------

            //will cause the offset to be set back to 0
            if (gamepad2.x)
                imureset = getAngle();

            offset = getAngle() - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated
            
            //the dir variable is the variable that determines where we want to be on the sine wave

            dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)-offset;
            mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= Math.sqrt(2);
            if (mag > Math.sqrt(2))
                mag = Math.sqrt(2);

            rot = gamepad1.left_stick_x;

            //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
            if (gamepad1.b)
                rot *= 0.5;
            if (gamepad1.b)
                mag *= 0.5;


            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0){
                FR.setPower((Math.sin(dir-(pi/4))*mag) - rot);
                FL.setPower((Math.sin(dir+(pi/4))*mag) + rot);
                BR.setPower((Math.sin(dir+(pi/4))*mag) - rot);
                BL.setPower((Math.sin(dir-(pi/4))*mag) + rot);
            } else {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
            }

            telemetry.addLine("Drivetrain");
            telemetry.addLine("");
            telemetry.addData("dir", dir);
            telemetry.addData("offset", offset);
            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BL Power", BL.getPower());
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.addData("Ry", gamepad1.right_stick_y);

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Hor Sliders");
            telemetry.addLine("");
            telemetry.addData("Hor Left Extender Pwr", HLext.getPower());
            telemetry.addData("Hor Left Extender Enc", HLext.getCurrentPosition());
            telemetry.addData("Hor Left Targ", HLexttarg);
            telemetry.addData("Hor Left Err", (HLexttarg - HLext.getCurrentPosition()));
            telemetry.addData("Hor Left Err Prev Diff", (HLexterr - HLextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("Hor Right Extender Pwr", HRext.getPower());
            telemetry.addData("Hor Right Extender Enc", HRext.getCurrentPosition());
            telemetry.addData("Hor Right Targ", HRexttarg);
            telemetry.addData("Hor Right Err", (HRexttarg - HRext.getCurrentPosition()));
            telemetry.addData("Hor Right Err Prev Diff", (HRexterr - HRextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Vert Sliders");
            telemetry.addLine("");
            telemetry.addData("vert Left Extender Pwr", VLext.getPower());
            telemetry.addData("vert Left Extender Enc", VLext.getCurrentPosition());
            telemetry.addData("vert Left Targ", VLexttarg);
            telemetry.addData("vert Left Err", (VLexttarg - VLext.getCurrentPosition()));
            telemetry.addData("vert Left Err Prev Diff", (VLexterr - VLextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("vert Right Extender Pwr", VRext.getPower());
            telemetry.addData("vert Right Extender Enc", VRext.getCurrentPosition());
            telemetry.addData("vert Right Targ", VRexttarg);
            telemetry.addData("vert Right Err", (VRexttarg - VRext.getCurrentPosition()));
            telemetry.addData("vert Right Err Prev Diff", (VRexterr - VRextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addData("kp", VKd);
            telemetry.addLine("");

            telemetry.addLine("User Inputs");
            telemetry.addLine("");
            telemetry.addData("A", buttonA);
            telemetry.addData("B", buttonB);
            telemetry.addData("X", buttonX);
            telemetry.addData("Y", buttonY);
            telemetry.addData("A2", button2A);
            telemetry.addData("B2", button2B);
            telemetry.addData("X2", button2X);
            telemetry.addData("Y2", button2Y);

            telemetry.addData("toer", toer.getPosition());
            telemetry.addData("toel", toel.getPosition());
            telemetry.addData("rint", Rint.getPosition());
            telemetry.addData("lint", Lint.getPosition());
            telemetry.addData("rrol", rrol.getPower());
            telemetry.addData("lrol", lrol.getPower());
            telemetry.addData("righttrig", gamepad2.right_trigger);
            telemetry.addData("lefttrig", gamepad2.left_trigger);

            //telemetry.addData("", );

            telemetry.update();
        }
    }
    private double getAngle()
    {
        //this converts the imu's outputs from -180 to 180 into an output of 0 to 360

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle += deltaAngle;

        lastAngles = angles;

        return angle;
    }
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
