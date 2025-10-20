using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class OdomSubscriberSafe : MonoBehaviour
{
    public enum PoseMode { PositionOnly, PositionPlusYaw, FullPose }

    [Header("ROS")]
    public string topicName = "/drone/odom";     // Odometry topic
    public bool odomIsNED = true;                // ✅ NED(Z down) -> tick this; ENU(Z up) -> untick

    [Header("Apply To")]
    public Transform target;                     // Drone root transform (the one with your visualizer)
    public PoseMode poseMode = PoseMode.PositionPlusYaw;

    [Header("Offsets (optional)")]
    public Vector3 positionOffset = Vector3.zero;
    public float yawOffsetDeg = 0f;

    private ROSConnection ros;

    void Awake()
    {
        if (target == null) target = transform;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, OnOdom);

        var rb = target.GetComponent<Rigidbody>();
        if (rb != null) { rb.useGravity = false; rb.isKinematic = true; }
    }

    void OnOdom(OdometryMsg msg)
    {
        var p = msg.pose.pose.position;
        var q = msg.pose.pose.orientation;

        Vector3 posUnity;
        Quaternion rotUnity;

        if (odomIsNED)
        {
            // PX4 NED -> Unity (X East, Y Up, Z North)
            // NED: X North, Y East, Z Down
            posUnity = new Vector3(
                (float)p.y,          // East  -> Unity X
                (float)(-p.z),       // Down  -> Unity Up (flip sign)
                (float)p.x           // North -> Unity Z
            );

            // Yaw-only is most robust; 
            Quaternion qRos = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            Vector3 eul = qRos.eulerAngles;       
            float yaw = eul.y + yawOffsetDeg;     
            rotUnity = Quaternion.Euler(0f, yaw, 0f);
        }
        else
        {
            // ENU -> Unity
            posUnity = new Vector3(
                (float)p.x,          // East  -> Unity X
                (float)p.z,          // Up    -> Unity Y
                (float)p.y           // North -> Unity Z
            );

            Quaternion qENU = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
            
            Quaternion qUnityFull = new Quaternion(qENU.x, qENU.z, qENU.y, qENU.w);

            if (poseMode == PoseMode.FullPose)
                rotUnity = qUnityFull * Quaternion.Euler(0f, yawOffsetDeg, 0f);
            else
                rotUnity = Quaternion.Euler(0f, (qUnityFull.eulerAngles).y + yawOffsetDeg, 0f);
        }

        target.position = posUnity + positionOffset;

        if (poseMode == PoseMode.PositionOnly)
            return;

        if (poseMode == PoseMode.PositionPlusYaw)
        {
            target.rotation = rotUnity;
        }
        else 
        {
            target.rotation = rotUnity;
        }
    }
}