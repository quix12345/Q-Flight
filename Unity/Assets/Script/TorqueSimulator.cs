using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorqueSimulator : MonoBehaviour
{
    public GameObject Motor1;
    public GameObject Motor2;
    public GameObject Motor3;
    public GameObject Motor4;
    public float coefficient = 0.005f;
    private Motor motor1;
    private Motor motor2;
    private Motor motor3;
    private Motor motor4;
    private ConstantForce cf;
    private float angularVyOld=0;
    private Rigidbody rb;
    // Start is called before the first frame update
    void Start()
    {
        cf = gameObject.GetComponent<ConstantForce>();
        rb = gameObject.GetComponent<Rigidbody>();
        motor1 = Motor1.GetComponent<Motor>();
        motor2 = Motor2.GetComponent<Motor>();
        motor3 = Motor3.GetComponent<Motor>();
        motor4 = Motor4.GetComponent<Motor>();
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 torque = new Vector3(0,motor1.rpm+motor3.rpm-motor2.rpm-motor4.rpm,0);
        //cf.torque = torque;
        angularVyOld += (motor1.rpm + motor3.rpm - motor2.rpm - motor4.rpm) * coefficient * Time.deltaTime;

        if(Math.Abs(angularVyOld)-(float)Math.Pow(angularVyOld, 2) * 0.05f >= 0)
        {
            angularVyOld -= Math.Sign(angularVyOld) * (float)Math.Pow(angularVyOld, 2) * 0.05f;
        }

        if (Math.Abs(angularVyOld) >= 3)
        {
            angularVyOld = Math.Sign(angularVyOld) * 3;
        }

        rb.angularVelocity = new Vector3(rb.angularVelocity.x,angularVyOld, rb.angularVelocity.z);
    }
}
