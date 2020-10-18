using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GroundStation : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    
    void OnMessageArrived(string msg)
    {
        // int[] iNums;
        // iNums = Array.ConvertAll(iNums , int.Parse);
        // for(int i=0;i<iNums.Length;i++){
        //     Debug.Log(iNums[i]);
        // }
        Debug.Log(msg);
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
