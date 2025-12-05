package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "JPL_Autonomous_FOUR_MOTORS", group = "JPL")
public class fourMotors extends LinearOpMode {
    
    // when looking at the bot from the back (away from canal dir), motors are 0, 1, 2, 3
    private DcMotorEx myMotor0 = null;
    private DcMotorEx myMotor1 = null;
    private DcMotorEx myMotor2 = null;
    private DcMotorEx myMotor3 = null;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        int UP_POSITION = -545; // og -545
        int DOWN_POSITION = 50; //50
        int down_hold = 1800;
        
        double ratio_offset = 1.5;

        // 0 to 2940
        double LIFT_VEL = 1600; // og 600, max 1600 or else the motors will stall / twist. 300 for testing w/o weights
        double LOWER_VEL = 350;
        
        // motor init stuff. the 0 and 3 motors are 40:1, 1 and 2 are 60:1
        myMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        myMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        myMotor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        
        myMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.setDirection(DcMotorSimple.Direction.FORWARD); 
        myMotor2.setDirection(DcMotorSimple.Direction.REVERSE); 
        myMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        
        int tolerance = 15;
        myMotor0.setTargetPositionTolerance(tolerance);
        myMotor1.setTargetPositionTolerance(tolerance);
        myMotor2.setTargetPositionTolerance(tolerance);
        myMotor3.setTargetPositionTolerance(tolerance);
        
        myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        myMotor0.setTargetPosition(0);
        myMotor1.setTargetPosition(0);
        myMotor2.setTargetPosition(0);
        myMotor3.setTargetPosition(0);

        myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Get initial PIDF coefficients to modify later
        PIDFCoefficients pidf40 = myMotor0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf60 = myMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Create the modified coefficient objects
        // Stronger PIDF for going UP
        PIDFCoefficients liftPIDF40 = new PIDFCoefficients(pidf40.p, pidf40.i, pidf40.d, pidf40.f + 35);
        PIDFCoefficients liftPIDF60 = new PIDFCoefficients(pidf60.p, pidf60.i, pidf60.d, pidf60.f + 25);

        // Standard PIDF for going DOWN
        PIDFCoefficients downPIDF40 = new PIDFCoefficients(pidf40.p * .7, pidf40.i, pidf40.d, pidf40.f); // Math.max(0, pidf40.f -5)
        PIDFCoefficients downPIDF60 = new PIDFCoefficients(pidf60.p * .7, pidf60.i, pidf60.d, pidf60.f);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

        // --- LOOP STARTS HERE ---
        for(int i = 0; i < 7; i++) {
            tolerance = 15;
            telemetry.addData("Loop Count", i + 1);
            telemetry.update();

            // ------------------------------------
            // PHASE 1: GO UP
            // ------------------------------------
            // myMotor0.setTargetPosition(0);
            // myMotor1.setTargetPosition(0);
            // myMotor2.setTargetPosition(0);
            // myMotor3.setTargetPosition(0);
            myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // Set Stronger PIDF for lifting
            myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF40);
            myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF60);
            myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF60);
            myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPIDF40);

            // Set Targets
            myMotor0.setTargetPosition(UP_POSITION);
            myMotor1.setTargetPosition((int)(UP_POSITION * ratio_offset));
            myMotor2.setTargetPosition((int)(UP_POSITION * ratio_offset));
            myMotor3.setTargetPosition(UP_POSITION);
            
            // Set Velocity
            myMotor0.setVelocity(LIFT_VEL);
            myMotor1.setVelocity((int) (LIFT_VEL * ratio_offset));
            myMotor2.setVelocity((int) (LIFT_VEL * ratio_offset));
            myMotor3.setVelocity(LIFT_VEL);
            
            
            runtime.reset();
            // Wait for completion
            while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy()) && runtime.seconds() < 4.5) {
                // if(Math.abs(myMotor0.getCurrentPosition() - UP_POSITION) < 100)
                // {
                //     LIFT_VEL = 150;
                // }
                // else
                // {
                //     LIFT_VEL = 600;
                // }
                // myMotor0.setVelocity(LIFT_VEL);
                // myMotor1.setVelocity((int) (LIFT_VEL * ratio_offset));
                // myMotor2.setVelocity((int) (LIFT_VEL * ratio_offset));
                // myMotor3.setVelocity(LIFT_VEL);
                    
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Status", "Moving UP");
                telemetry.addData("M0 Pos", myMotor0.getCurrentPosition());
                telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
                telemetry.addData("M2 Pos", myMotor2.getCurrentPosition());
                telemetry.addData("M3 Pos", myMotor3.getCurrentPosition());
                telemetry.update();
            }
            
            myMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            double upHoldPower = -0.04;
            myMotor0.setPower(upHoldPower);
            myMotor1.setPower(upHoldPower);
            myMotor2.setPower(upHoldPower);
            myMotor3.setPower(upHoldPower);
            
            sleep(1200); 

            // ------------------------------------
            // PHASE 2: GO DOWN
            // ------------------------------------
            // tolerance = 0;
            // myMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // myMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // myMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // myMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // // Set Standard PIDF for lowering
            // myMotor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);
            // myMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
            // myMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF60);
            // myMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, downPIDF40);
            
            // // Set Targets
            // myMotor0.setTargetPosition(DOWN_POSITION);
            // myMotor1.setTargetPosition((int)(DOWN_POSITION * ratio_offset));
            // myMotor2.setTargetPosition((int)(DOWN_POSITION * ratio_offset));
            // myMotor3.setTargetPosition(DOWN_POSITION);
            
            // // Set Velocity
            // myMotor0.setVelocity(LOWER_VEL);
            // myMotor1.setVelocity((int) (LOWER_VEL * ratio_offset));
            // myMotor2.setVelocity((int) (LOWER_VEL * ratio_offset));
            // myMotor3.setVelocity(LOWER_VEL);
            
            // runtime.reset();
            // // Wait for completion
            // while (opModeIsActive() && (myMotor0.isBusy() || myMotor1.isBusy())) { // && runtime.seconds() < 6
            //     telemetry.addData("Loop", i + 1);
            //     telemetry.addData("Status", "Moving DOWN");
            //     telemetry.addData("M0 Pos", myMotor0.getCurrentPosition());
            //     telemetry.addData("M1 Pos", myMotor1.getCurrentPosition());
            //     telemetry.addData("M2 Pos", myMotor2.getCurrentPosition());
            //     telemetry.addData("M3 Pos", myMotor3.getCurrentPosition());
            //     telemetry.update();
            // }

            // // pushes the bucket firmly through the water to the B-stop.
            // // consider doing another downward movement with high feedforward or this below:
            myMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // tune these with loose collars
            myMotor0.setPower(0.3);
            myMotor1.setPower(0.3);
            myMotor2.setPower(0.3);
            myMotor3.setPower(0.3);
            
            telemetry.addData("Status", "pushing into bstop");
            telemetry.update();
            
            // push for variable time
            sleep(750);
            
            
            // brake
            myMotor0.setPower(-0.1);
            myMotor1.setPower(-0.1);
            myMotor2.setPower(-0.1);
            myMotor3.setPower(-0.1);
            
            telemetry.addData("Status", "pushing into bstop");
            telemetry.update();
            
            // push for variable time
            sleep(250);
            
            myMotor0.setPower(0.05);
            myMotor1.setPower(0.05);
            myMotor2.setPower(0.05);
            myMotor3.setPower(0.05);
            
            sleep(100);
            
            myMotor0.setPower(.4);
            myMotor1.setPower(.4);
            myMotor2.setPower(.4);
            myMotor3.setPower(.4);
            
            sleep(down_hold);
            
            myMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            // decrements/increments
            // UP_POSITION -= 5;
            // down_hold += 100;
    
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
    }
}
