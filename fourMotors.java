package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "JPL_Autonomous_FOUR_MOTORS", group = "JPL")
public class fourMotors extends LinearOpMode {
    
    // when looking at the bot from the back (away from canal dir), motors are 0-3 from left to right
    private DcMotorEx myMotor0 = null;
    private DcMotorEx myMotor1 = null;
    private DcMotorEx myMotor2 = null;
    private DcMotorEx myMotor3 = null;

    @Override
    public void runOpMode() {

        int UP_POSITION = -710; // og -750
        int DOWN_POSITION = 0;
        double ratio_offset = 1.5;

        // 0 to 2940
        double LIFT_VEL = 400; // 2000 for water
        double LOWER_VEL = 220;
        
        
        // motor init stuff. the 0 and 3 motors are 40:1, 1 and 2 are 60:1
        myMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        myMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        
        
        // DISCONNECT THE BUCKET THEN TEST INDIVIDUAL ARMS TO FIND CORRECT DIRECTIONS
        myMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor3.setDirection(DcMotorSimple.Direction.FORWARD);
        
        myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // myMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        myMotor0.setTargetPosition(0);
        myMotor1.setTargetPosition(0);
        myMotor2.setTargetPosition(0);
        myMotor3.setTargetPosition(0);

        //pidf
        PIDFCoefficients pidf = myMotor0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double newP = pidf.p * 1;
        double newF = 10;
        PIDFCoefficients newPIDF = new PIDFCoefficients(newP, pidf.i, pidf.d, newF);
        myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        
        // up?
        myMotor0.setTargetPosition(UP_POSITION);
        myMotor1.setTargetPosition((int)(UP_POSITION * ratio_offset));
        myMotor2.setTargetPosition((int)(UP_POSITION * ratio_offset));
        myMotor3.setTargetPosition(UP_POSITION);
        
        myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        // setVel should have PID for speed built in 
        myMotor0.setVelocity(LIFT_VEL);
        myMotor1.setVelocity((int) (LIFT_VEL * ratio_offset));
        myMotor2.setVelocity((int) (LIFT_VEL * ratio_offset));
        myMotor3.setVelocity(LIFT_VEL);

        // Wait for BOTH motors to reach target
        while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy())) {
            telemetry.addData("M0 Pos", myMotor0.getCurrentPosition());
            telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
            telemetry.addData("M2 Pos", myMotor2.getCurrentPosition());
            telemetry.addData("M3 Pos", myMotor3.getCurrentPosition());
            telemetry.update();
            }
        
        // myMotor.setPower(0);
        // myMotor1.setPower(0);
        sleep(800); // og 1000
        
        myMotor0.setTargetPosition(DOWN_POSITION);
        myMotor1.setTargetPosition(DOWN_POSITION);
        myMotor2.setTargetPosition(DOWN_POSITION);
        myMotor3.setTargetPosition(DOWN_POSITION);
        
        myMotor0.setVelocity(LOWER_VEL);
        myMotor1.setVelocity((int) (LOWER_VEL * ratio_offset));
        myMotor2.setVelocity((int) (LOWER_VEL * ratio_offset));
        myMotor3.setVelocity(LOWER_VEL);
        
        while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy())) {
        telemetry.addData("M0 Pos", myMotor0.getCurrentPosition());
        telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
        telemetry.addData("M2 Pos", myMotor2.getCurrentPosition());
        telemetry.addData("M3 Pos", myMotor3.getCurrentPosition());
        telemetry.update();
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}
