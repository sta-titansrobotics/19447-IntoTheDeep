package movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class movementtest extends LinearOpMode {

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

    int lexttarg;
    int rexttarg;
    int lexttargfine;
    int rexttargfine;
    double rexterr;
    double lexterr;
    double Lextpower;
    double Rextpower;
    double rextpreverr;
    double lextpreverr;

    double torquetarg;

    double Kp = 0.02;
    double Kd = 0.015;

    double rot;

    double offset = 0;
    double imureset = 0;

    double wristYaw;

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

        DcMotor Lext = hardwareMap.get(DcMotor.class, "Lext"); // Control hub 0
        DcMotor Rext = hardwareMap.get(DcMotor.class, "Rext"); // Control hub 1
        DcMotor Torque = hardwareMap.get(DcMotor.class, "Torque"); // Control hub 2

        Servo claw = hardwareMap.get(Servo.class, "Claw"); // Control hub 0
        Servo wristYawservo = hardwareMap.get(Servo.class, "WristYaw"); // Control hub 1

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
        Lext.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // ------------------PID-POWER---------------------------------

            // 4 stage sliders
            // limiting the motors movement so that it does not try to over extend the sliders
            rexttarg = (int) clamp(rexttarg, 50, 4100);
            lexttarg = (int) clamp(lexttarg, 50, 4100);

            rexterr = lexttarg - Rext.getCurrentPosition();
            lexterr = lexttarg - Lext.getCurrentPosition();

            // actual pd calculations
            Rextpower = Kp*rexterr+Kd*(rexterr - rextpreverr);
            Lextpower = Kp*lexterr+Kd*(lexterr - lextpreverr);

            // getting the previous error
            rextpreverr = (rexttarg - Rext.getCurrentPosition());
            lextpreverr = (lexttarg - Lext.getCurrentPosition());

            // actually setting the motor power
            Rext.setPower(clamp(Rextpower, -1, 1));
            Lext.setPower(clamp(Lextpower, -1, 1));

            // ------------------MACROS---------------------------------

            // 4 stage sliders
            if (gamepad2.y && !butYcheck) {
                buttonY += 1;
                butYcheck = true;
            }

            if (!gamepad2.y) {
                butYcheck = false;
            }

            if (!butYcheck) {
                if (buttonY % 2 == 1) {
                    rexttarg = 4100 + rexttargfine;
                    lexttarg = 4100 + rexttargfine;
                    } else {
                    rexttarg = 50 + rexttargfine;
                    lexttarg = 50 + rexttargfine;
                }
            }

            // ------------------TELEOP---------------------------------

            if (gamepad2.right_bumper)
                Torque.setPower(1);
            else if (gamepad2.left_bumper)
                 Torque.setPower(-1);
            else
                 Torque.setPower(0);

            //claw control
            if (gamepad1.a && !butAcheck) {
                buttonA += 1;
                butAcheck = true;
            }
            if (!gamepad1.a) {
                butAcheck = false;
            }

            if (!butAcheck) {
                if (buttonA % 2 == 1) {
                    claw.setPosition(0.25);
                    } else {
                    claw.setPosition(0);
                }
            }

            //slider control
            if (gamepad1.right_bumper){
                rexttargfine += 10;
                lexttargfine += 10;
            } else if (gamepad1.left_bumper){
                rexttargfine -= 10;
                lexttargfine -= 10;
            }


            //wrist yaw rot sync
            if (gamepad1.x && !butXcheck) {
                buttonX += 1;
                butXcheck = true;
            }
            if (!gamepad1.x) {
                butXcheck = false;
            }

            if (!butXcheck) {
                if (buttonX % 2 == 1) {
                    wristYawservo.setPosition(dir/pi);
                    wristYaw = (dir/pi);
                } else {
                    wristYawservo.setPosition(wristYaw);
                }
            }

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

            telemetry.addData("kp", Kp);
            telemetry.addData("kd", Kd);

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

            telemetry.addLine("4 Stage Sliders");
            telemetry.addLine("");
            telemetry.addData("Left Extender Pwr", 0.01*lexterr*0.1*(lextpreverr - lexterr));
            telemetry.addData("Left Extender Enc", Lext.getCurrentPosition());
            telemetry.addData("Left Targ", lexttarg);
            telemetry.addData("Left Err", (lexttarg - Lext.getCurrentPosition()));
            telemetry.addData("Left Err Prev Diff", (lexterr - lextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("Right Extender Pwr", 0.01*rexterr*0.1*(rextpreverr - rexterr));
            telemetry.addData("Right Extender Enc", Rext.getCurrentPosition());
            telemetry.addData("Right Targ", rexttarg);
            telemetry.addData("Right Err", (rexttarg - Rext.getCurrentPosition()));
            telemetry.addData("Right Err Prev Diff", (lexterr - lextpreverr));

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Intake");
            telemetry.addLine("");
            telemetry.addData("Claw Rot", claw.getPosition());
            telemetry.addData("Wrist Yaw", wristYawservo.getPosition());
            telemetry.addData("Torque2", torquetarg);
            telemetry.addData("Torque", Torque.getPower());

            telemetry.addLine("");
            telemetry.addLine("");
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
