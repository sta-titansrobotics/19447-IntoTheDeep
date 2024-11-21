package movement;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class AutoA extends LinearOpMode {

    // Lex
    int lex;
    int lexttarg;
    int lexttargfine;
    double lextpower;

    // Rex
    int rex;
    int rexttarg;
    int rexttargfine;
    double rexpower;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angle;
    private DcMotor Lext;
    private DcMotor Rext;
    private DcMotor Torque;
    private DcMotor Torque2;
    private CRServo george;
    private Servo curious;
    private Servo claw;
    private Servo wristYawservo;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

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
        DcMotor Torque2 = hardwareMap.get(DcMotor.class, "Torque2"); // Control hub 2

        CRServo george = hardwareMap.get(CRServo.class, "george");
        Servo curious = hardwareMap.get(Servo.class, "curious");
        Servo claw = hardwareMap.get(Servo.class, "Claw"); // Control hub 0
        Servo wristYawservo = hardwareMap.get(Servo.class, "WristYaw"); // Control hub 1

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Lext.setDirection(DcMotorSimple.Direction.REVERSE);
        Torque2.setDirection(DcMotorSimple.Direction.REVERSE);

        Lext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rext.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Sliders
    public void sliders_up(){
        Lext.setTargetPosition(3000);
        Rext.setTargetPosition(3000);
    }

    public void sliders_down(){
        Lext.setTargetPosition(50);
        Rext.setTargetPosition(50);
    }

    // Geroge Servo - up
    public void George_up(){
        george.setPower(0);
    }

    // Geroge Servo - down

    public void George_down(){
        george.setPower(-12950);
    }

    // Curious Servo-  intake
    public void curious_intake(){
        curious.setPosition(0.65);
    }

    // Curious Servo - outake
    public void curious_outake(){
        curious.setPosition(0.4);
    }

    }
