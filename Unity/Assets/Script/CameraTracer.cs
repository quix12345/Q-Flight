using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTracer : MonoBehaviour
{
    public GameObject Target;
    private GameObject Camera;
    private GameObject realTarget;
    // Start is called before the first frame update
    void Start()
    {
        // Target = GameObject.Find("ZQ_Quadrotor").gameObject;
        Camera=gameObject.transform.Find("Main Camera").gameObject;
        realTarget = Target.transform.Find("model").gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 offset = new Vector3(0,10f,20f);
        Camera.transform.position = offset + realTarget.transform.position;
        float y = realTarget.transform.rotation.eulerAngles.y;
        Camera.transform.rotation = Quaternion.Euler(new Vector3(0f,y,0.0f));
       // Camera.transform.eulerAngles = Camera.transform.position - Target.transform.position;
    }
}
