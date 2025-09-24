using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Px4;

public class DronePX4Subscriber : MonoBehaviour
{
    private ROSConnection ros;
    private bool isReceivingData = false;
    private float lastMessageTime = 0f;
    private int messageCount = 0;

    private Vector3 targetPosition;
    private Vector3 currentVelocity;
    public float smoothTime = 0.1f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to position topic
        ros.Subscribe<VehicleLocalPositionMsg>("/unity/vehicle_local_position", PositionCallback);

        Debug.Log("=== DRONE SUBSCRIBER STARTED ===");
        Debug.Log($"ROS Connection IP: {ros.RosIPAddress}");
        Debug.Log($"ROS Connection Port: {ros.RosPort}");
        Debug.Log("Subscribed to: /unity/vehicle_local_position");

        // Initialize position
        targetPosition = transform.position;
    }

    void PositionCallback(VehicleLocalPositionMsg msg)
    {
        messageCount++;
        lastMessageTime = Time.time;
        isReceivingData = true;

        // Debug
        if (messageCount % 10 == 0)
        {
            Debug.Log($"[MSG #{messageCount}] PX4 Position - X: {msg.x:F2}, Y: {msg.y:F2}, Z: {msg.z:F2}");
            Debug.Log($"[MSG #{messageCount}] Velocity - VX: {msg.vx:F2}, VY: {msg.vy:F2}, VZ: {msg.vz:F2}");
        }

        // Convert NED to Unity coordinates
        // NED: X=North(forward), Y=East(right), Z=Down
        // Unity: X=Right, Y=Up, Z=Forward
        targetPosition = new Vector3(msg.y, -msg.z, msg.x);

        // Show position change
        float distance = Vector3.Distance(transform.position, targetPosition);
        if (distance > 0.01f)
        {
            Debug.Log($"<color=green>DRONE MOVING! Distance: {distance:F3}m</color>");
        }
    }

    void Update()
    {
        // Smooth movement
        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref currentVelocity,
            smoothTime
        );

        // Connection status indicator
        if (Time.time - lastMessageTime > 2f && isReceivingData)
        {
            Debug.LogWarning("No messages received for 2 seconds! Check connection.");
            isReceivingData = false;
        }
    }

    void OnGUI()
    {
        // Display connection status on screen
        GUIStyle style = new GUIStyle();
        style.fontSize = 16;
        style.normal.textColor = isReceivingData ? Color.green : Color.red;

        string status = isReceivingData ?
            $"RECEIVING DATA (#{messageCount})" :
            "NO DATA";

        GUI.Label(new Rect(10, 10, 300, 30), $"ROS Status: {status}", style);
        GUI.Label(new Rect(10, 40, 300, 30), $"Position: {transform.position}", style);
        GUI.Label(new Rect(10, 70, 300, 30), $"Target: {targetPosition}", style);
    }
}