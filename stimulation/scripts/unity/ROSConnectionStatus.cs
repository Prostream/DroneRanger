using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;


public class ROSConnectionStatus : MonoBehaviour
{
    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        Debug.Log($"=== ROS Connection Debug ===");
        Debug.Log($"Trying to connect to: {ros.RosIPAddress}:{ros.RosPort}");
        Debug.Log($"If this shows 192.168.1.70, change it to 'localhost' in Robotics->ROS Settings");

        // show the current setting
        if (ros.RosIPAddress == "192.168.1.70")
        {
            Debug.LogWarning("WRONG IP! Change to 'localhost' in ROS Settings!");
        }
        else if (ros.RosIPAddress == "localhost" || ros.RosIPAddress == "127.0.0.1")
        {
            Debug.Log("✓ Correct IP setting for mirrored networking!");
        }
    }

}
