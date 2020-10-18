using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gyroscope : MonoBehaviour
{
    public Vector3 eulerAngles;
    public Vector3 gyro;
    public GameObject model;
    private Rigidbody rb;
    // Start is called before the first frame update
    void Start()
    {
        rb = model.GetComponent<Rigidbody>();
    }
    private void Update()
    {
        GyroscopeRead();
    }

    public void GyroscopeRead()
    {
        eulerAngles = model.transform.rotation.eulerAngles;
        eulerAngles.x = ConvertAngle(eulerAngles.x);
        eulerAngles.y = ConvertAngle(eulerAngles.y);
        eulerAngles.z = ConvertAngle(eulerAngles.z);
        gyro = rb.angularVelocity;
    }

    private float ConvertAngle(float angle)
    {
        while (angle > 180)
        {
            angle -= 360;
        }
        while (angle < -180)
        {
            angle += 360;
        }
        return angle;
    }
}
