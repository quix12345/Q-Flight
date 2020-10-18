using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlightControl : MonoBehaviour
{
    public GameObject Motor1;
    public GameObject Motor2;
    public GameObject Motor3;
    public GameObject Motor4;
    public GameObject Gyroscope;
    public GameObject model;

    public bool IsSuperControl = false;
    public float Throttle=0;
    public float pitch_expect=0;
    public float roll_expect=0;
    public float yaw_expect=0;

    public float AngleLimit = 5;
    private float yawOld = 0;
    private float pitchOld = 0;
    private float rollOld = 0;
    private Motor motor1;
    private Motor motor2;
    private Motor motor3;
    private Motor motor4;
    private Gyroscope gyroscope;
    public PID pid_out_Pitch;
    public PID pid_out_Roll;
    public PID pid_in_Pitch;
    public PID pid_in_Roll;
    public PID pid_in_Yaw;
    float pid_out_pitch = 0;
    float pid_out_roll = 0;
    float pid_in_pitch = 0;
    float pid_in_roll = 0;
    float pid_in_yaw = 0;
    private Vector3 euler_angles;
    private Vector3 gyro;
    public int MOTO1_PWM, MOTO2_PWM, MOTO3_PWM, MOTO4_PWM;

    public  float DEFAULT_PID_INTEGRATION_LIMIT = 100.0f;
    public  int YM_Dead = 1150;
    public  float PID_OUT_PITCH_KP = 30f;
    public  float PID_OUT_PITCH_KI = 0f;
    public  float PID_OUT_PITCH_KD = 0f;
    public  float PID_OUT_PITCH_INTEGRATION_LIMIT = 100.0f;

    public  float PID_OUT_ROLL_KP = 30f;
    public  float PID_OUT_ROLL_KI = 0f;
    public  float PID_OUT_ROLL_KD = 0f;
    public  float PID_OUT_ROLL_INTEGRATION_LIMIT = 100.0f;

    public  float PID_IN_PITCH_KP = 0.4f;
    public  float PID_IN_PITCH_KI = 0.00f;
    public  float PID_IN_PITCH_KD = 0f;
    public  float PID_IN_PITCH_INTEGRATION_LIMIT = 200.0f;

    public  float PID_IN_ROLL_KP = 0.4f;
    public  float PID_IN_ROLL_KI = 0.0f;
    public  float PID_IN_ROLL_KD = 0f;
    public  float PID_IN_ROLL_INTEGRATION_LIMIT = 200.0f;

    public  float PID_IN_YAW_KP = 1f;
    public  float PID_IN_YAW_KI = 0f;
    public  float PID_IN_YAW_KD = 0f;
    public  float PID_IN_YAW_INTEGRATION_LIMIT = 100.0f;

    private float timer = 0f;

    // Start is called before the first frame update
    void Start()
    {
        motor1 = Motor1.GetComponent<Motor>();
        motor2 = Motor2.GetComponent<Motor>();
        motor3 = Motor3.GetComponent<Motor>();
        motor4 = Motor4.GetComponent<Motor>();

        gyroscope = Gyroscope.GetComponent<Gyroscope>();

        pid_out_Pitch = new PID(PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
        pid_out_Roll = new PID(PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);

        pid_in_Pitch = new PID(PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
        pid_in_Roll = new PID(PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
        pid_in_Yaw = new PID(PID_IN_YAW_KP, PID_IN_YAW_KI, PID_IN_YAW_KD);

        pid_out_Pitch.pidSetIntegralLimit(PID_OUT_PITCH_INTEGRATION_LIMIT);
        pid_out_Roll.pidSetIntegralLimit(PID_OUT_ROLL_INTEGRATION_LIMIT);
        pid_in_Pitch.pidSetIntegralLimit(PID_IN_PITCH_INTEGRATION_LIMIT);
        pid_in_Roll.pidSetIntegralLimit(PID_IN_ROLL_INTEGRATION_LIMIT);
        pid_in_Yaw.pidSetIntegralLimit(PID_IN_YAW_INTEGRATION_LIMIT);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //if (timer <= 0.002)
        //{
        //    timer += Time.deltaTime;
        //}
        //else
        //{
        if (!IsSuperControl)
        {
            timer = 0;
            ImuUpdate();
            Control(Throttle, pitch_expect, roll_expect, yaw_expect);
        }
        else
        {
            SuperControl();
        }
        
        //}
    }

    float Convert2PercentagePWN(int Raw_PWN)
    {
        if (Raw_PWN < 1000)
        {
            Raw_PWN = 1000;
        }
        else if (Raw_PWN > 2000)
        {
            Raw_PWN = 2000;
        }
        return (float)(Raw_PWN - 1000.0f) / 10.0f;
    }

    void Control(float Throttle, float pitch_expect, float roll_expect, float yaw_expect)
    {
        if (Throttle < 0)
        {
            Throttle = 0;
        }
        else if (Throttle > 1)
        {
            Throttle = 1;
        }
        int RC_THROTTLE = (int)(1000.0f + Throttle * 1000.0f);
        if (RC_THROTTLE > YM_Dead)        //ÓÍÃÅ´óÓÚËÀÇø²Å½øÐÐ¿ØÖÆ
        {
            pid_out_pitch = pid_out_Pitch.PIDupdate(euler_angles.x, pitch_expect);
            //pid_out_pitch=pitch_expect;
            pid_in_pitch = pid_in_Pitch.PIDupdate(gyro.x, pid_out_pitch);

            pid_out_roll =pid_out_Roll.PIDupdate(euler_angles.z, roll_expect);
            //pid_out_roll=roll_expect;
            pid_in_roll = pid_in_Roll.PIDupdate(gyro.z, pid_out_roll);

            pid_in_yaw = pid_in_Yaw.PIDupdate(gyro.y, yaw_expect);
        }
        else if (RC_THROTTLE < YM_Dead)
        {
            pid_in_Pitch.integ = 0;
            pid_in_Roll.integ = 0;
            pid_in_Yaw.integ = 0;
        }
        MOTO1_PWM = (int)(RC_THROTTLE - pid_in_pitch + pid_in_roll - pid_in_yaw);
        MOTO2_PWM = (int)(RC_THROTTLE + pid_in_pitch + pid_in_roll + pid_in_yaw);
        MOTO3_PWM = (int)(RC_THROTTLE + pid_in_pitch - pid_in_roll - pid_in_yaw);
        MOTO4_PWM = (int)(RC_THROTTLE - pid_in_pitch - pid_in_roll + pid_in_yaw);
        if (RC_THROTTLE > YM_Dead)
        {
            if (MOTO1_PWM < YM_Dead) MOTO1_PWM = YM_Dead; else if (MOTO1_PWM > 2000) MOTO1_PWM = 2000;
            if (MOTO2_PWM < YM_Dead) MOTO2_PWM = YM_Dead; else if (MOTO2_PWM > 2000) MOTO2_PWM = 2000;
            if (MOTO3_PWM < YM_Dead) MOTO3_PWM = YM_Dead; else if (MOTO3_PWM > 2000) MOTO3_PWM = 2000;
            if (MOTO4_PWM < YM_Dead) MOTO4_PWM = YM_Dead; else if (MOTO4_PWM > 2000) MOTO4_PWM = 2000;
        }
        else
        {
            MOTO1_PWM = 1000;
            MOTO2_PWM = 1000;
            MOTO3_PWM = 1000;
            MOTO4_PWM = 1000;
        }

        PWN_Control_Motors(Convert2PercentagePWN(MOTO1_PWM),
        Convert2PercentagePWN(MOTO2_PWM),
        Convert2PercentagePWN(MOTO3_PWM),
        Convert2PercentagePWN(MOTO4_PWM));
    }

    private void ImuUpdate()
    {
        gyroscope.GyroscopeRead();
        euler_angles = gyroscope.eulerAngles;
        gyro = gyroscope.gyro;
    }

    private void PWN_Control_Motors(float pwn_motor1,float pwn_motor2,float pwn_motor3,float pwn_motor4)
    {
        motor1.Modulation = pwn_motor1;
        motor2.Modulation = pwn_motor2;
        motor3.Modulation = pwn_motor3;
        motor4.Modulation = pwn_motor4;
    }


    private void SuperControl()
    {
        if (Math.Abs(pitch_expect) <= 1)
        {
            pitch_expect = 0;
        }
        if (Math.Abs(yaw_expect) <= 1)
        {
            yaw_expect = 0;
        }
        if (Math.Abs(roll_expect) <= 1)
        {
            roll_expect = 0;
        }

        if (Math.Abs(rollOld) >= 50)
        {
            rollOld = Math.Sign(rollOld) * 50;
        }
        if (Math.Abs(pitchOld) >= 50)
        {
            pitchOld=Math.Sign(pitchOld) * 50;
        }

        yawOld += yaw_expect * Time.deltaTime;
        pitchOld += pitch_expect * Time.deltaTime;
        rollOld += roll_expect * Time.deltaTime;
        Debug.Log(yawOld);
        model.transform.rotation = Quaternion.Euler(new Vector3(pitchOld,yawOld,rollOld));
        int RC_THROTTLE = (int)(1000.0f + Throttle * 1000.0f);
        MOTO1_PWM = (int)RC_THROTTLE;
        MOTO2_PWM = (int)RC_THROTTLE;
        MOTO3_PWM = (int)RC_THROTTLE;
        MOTO4_PWM = (int)RC_THROTTLE;
        if (RC_THROTTLE > YM_Dead)
        {
            if (MOTO1_PWM < YM_Dead) MOTO1_PWM = YM_Dead; else if (MOTO1_PWM > 2000) MOTO1_PWM = 2000;
            if (MOTO2_PWM < YM_Dead) MOTO2_PWM = YM_Dead; else if (MOTO2_PWM > 2000) MOTO2_PWM = 2000;
            if (MOTO3_PWM < YM_Dead) MOTO3_PWM = YM_Dead; else if (MOTO3_PWM > 2000) MOTO3_PWM = 2000;
            if (MOTO4_PWM < YM_Dead) MOTO4_PWM = YM_Dead; else if (MOTO4_PWM > 2000) MOTO4_PWM = 2000;
        }
        else
        {
            MOTO1_PWM = 1000;
            MOTO2_PWM = 1000;
            MOTO3_PWM = 1000;
            MOTO4_PWM = 1000;
        }

        PWN_Control_Motors(Convert2PercentagePWN(MOTO1_PWM),
        Convert2PercentagePWN(MOTO2_PWM),
        Convert2PercentagePWN(MOTO3_PWM),
        Convert2PercentagePWN(MOTO4_PWM));
    }

    void OnMessageArrived(string msg)
    {
        string[] content = msg.Split(',');

        Throttle = (float)(Convert.ToInt32(content[0]) / 100.0);
        yaw_expect = (float)((Convert.ToInt32(content[1]) - 50) * AngleLimit / 50.0);
        pitch_expect = -(float)((Convert.ToInt32(content[2]) - 50) * AngleLimit / 50.0);
        roll_expect = -(float)((Convert.ToInt32(content[3]) - 50) * AngleLimit / 50.0);

    }

    // Invoked when a connect/disconnect event occurs. The parameter 'success'
    // will be 'true' upon connection, and 'false' upon disconnection or
    // failure to connect.
    void OnConnectionEvent(bool success)
    {
        if (success)
            Debug.Log("Connection established");
        else
            Debug.Log("Connection attempt failed or disconnection detected");
    }
}
