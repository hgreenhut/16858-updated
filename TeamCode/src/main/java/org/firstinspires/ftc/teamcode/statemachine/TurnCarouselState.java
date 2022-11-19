package org.firstinspires.ftc.teamcode.statemachine;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class TurnCarouselState extends State{

    boolean on = false;

    DcMotor carouselMotor;
    String movement;

    double carouselPower;

    private int Time;
    ElapsedTime mRuntime = new ElapsedTime();




    public TurnCarouselState(StateMachine statemachine, int time, ArrayList<DcMotor> motors, double power){
        super(statemachine);
        carouselMotor = motors.get(5);
        carouselPower = power;
        Time = time;
        mRuntime.reset();
    }

    @Override
    void start() {
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.startTimeOut(this.Time);
        carouselMotor.setPower(carouselPower);
    }

    @Override
    void update() {
    }

    @Override
    void stop() {
        carouselMotor.setPower(0);
    }

    @Override
    void initialize() {

    }

    public boolean getOn(){
        return on;
    }
}
