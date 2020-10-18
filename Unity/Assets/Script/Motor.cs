using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Motor : MonoBehaviour
{
    public float Modulation=0f;
    public float Voltage = 12.4f;
    private Rigidbody rb;
    public float force=0f;
    public float rpm = 0f;
    public float coefficient = 0.005f;
    public int direction;
    public Vector3 torqueOld;

    void Start()
    {
        rb = gameObject.GetComponent<Rigidbody>();
    }
    void Update()
    {
        force = Calc_force();
        rpm = Calc_RPM();
        Vector3 v = gameObject.transform.Find("Cube").transform.position-gameObject.transform.position;
        Vector3 v2 = new Vector3(0, 0.01f, 0);
        rb.AddForce(v * force);
        torqueOld += direction * (v2*rpm*Time.deltaTime - new Vector3(0, 20f, 0) * Time.deltaTime);
        if (direction*torqueOld.y <= 0)
        {
            torqueOld.y = 0;
        }
        if(direction * torqueOld.y >= 30)
        {
            torqueOld.y = 30 * direction;
        }
        transform.Rotate(torqueOld);
        // rb.AddForce(v2 * rpm * coefficient);
    }

    private float Calc_force()
    {
        if (Modulation == 0)
        {
            return 0;
        }
        float force = (float)(2.24e-06 * Math.Pow(Modulation, 3) + 9.652e-06 * Math.Pow(Modulation, 2) + 0.03795 * Modulation + 0.2359) * 20;
        if (rb.velocity.y > 0)
        {
            force -= (float)Math.Pow(rb.velocity.y, 2)*coefficient;
        }
        if (force <= 0)
        {
            return 0f;
        }
        return force;
    }

    private float Calc_RPM()
    {
        if (force == 0)
        {
            return 0;
        }
        return (float)Math.Pow(force,0.5)*3200;
    }
}
