using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public class ForwardObstacleGuard : MonoBehaviour
{
    [Header("Grid & Distance")]
    [Tooltip("Size of the smallest grid cell in Unity world units (meters).")]
    public float cellSizeMeters = 0.5f;

    [Tooltip("Safe distance (in cells). Effective meters = cellSizeMeters * safeCells.")]
    public int safeCells = 6;

    [Tooltip("Radius of the forward 'corridor' to check, in cells. Effective meters = cellSizeMeters * checkRadiusCells.")]
    public float checkRadiusCells = 1.0f;

    [Header("Detection Options")]
    [Tooltip("Layers that count as obstacles.")]
    public LayerMask obstacleLayers = ~0; // default: all layers

    [Tooltip("Whether to include trigger colliders in physics queries.")]
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Tooltip("If the host object has a collider, assign it here to ignore self-hits.")]
    public Collider selfCollider;

    [Header("Aim Source")]
    [Tooltip("Camera used for position/forward direction. If null, uses this object's Transform.")]
    public Camera cam;

    [Header("ROS2")]
    [Tooltip("ROS2 topic to publish std_msgs/Bool. True = blocked, False = clear.")]
    public string topicName = "/testObstacleSignal";

    [Tooltip("Publishing rate in Hz.")]
    [Range(1, 120)]
    public int publishHz = 30;

    [Header("Debug")]
    public bool drawGizmos = true;
    public Color gizmoColorClear = new Color(0f, 1f, 0f, 0.12f);
    public Color gizmoColorBlocked = new Color(1f, 0f, 0f, 0.12f);
    public Color gizmoLine = new Color(1f, 1f, 1f, 0.8f);

    // --- private state ---
    ROSConnection ros;
    float safeDistanceMeters;   // computed safe distance in meters
    float castRadiusMeters;     // computed cast radius in meters
    readonly BoolMsg cachedTrue = new BoolMsg(true);
    readonly BoolMsg cachedFalse = new BoolMsg(false);
    bool lastState = false;
    float publishInterval;
    float publishTimer = 0f;

    void Awake()
    {
        // Clamp/normalize inputs
        cellSizeMeters = Mathf.Max(0.001f, cellSizeMeters);
        safeCells = Mathf.Max(1, safeCells);
        checkRadiusCells = Mathf.Max(0.0f, checkRadiusCells);

        RecomputeDistances();

        // ROS setup
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>(topicName);

        publishInterval = 1f / Mathf.Max(1, publishHz);

        // Auto-assign self collider if available
        if (selfCollider == null)
            selfCollider = GetComponent<Collider>();
    }

    void RecomputeDistances()
    {
        safeDistanceMeters = cellSizeMeters * safeCells;
        castRadiusMeters = cellSizeMeters * checkRadiusCells;
    }

    void OnValidate()
    {
        // Keep editor-time values sane
        cellSizeMeters = Mathf.Max(0.001f, cellSizeMeters);
        safeCells = Mathf.Max(1, safeCells);
        checkRadiusCells = Mathf.Max(0.0f, checkRadiusCells);
        publishHz = Mathf.Clamp(publishHz, 1, 120);
        RecomputeDistances();
        publishInterval = 1f / Mathf.Max(1, publishHz);
    }

    void Update()
    {
        // Compute current blocked/clear state
        bool blocked = CheckBlocked();

        // Heartbeat publish at a fixed rate (even if unchanged)
        publishTimer += Time.deltaTime;
        if (publishTimer >= publishInterval)
        {
            ros.Publish(topicName, blocked ? cachedTrue : cachedFalse);
            publishTimer = 0f;
        }

        lastState = blocked;
    }

    bool CheckBlocked()
    {
        Transform aimTf = (cam != null) ? cam.transform : transform;
        Vector3 origin = aimTf.position;
        Vector3 dir = aimTf.forward;

        // Slightly nudge forward to reduce immediate self-hits
        const float startNudge = 0.01f;
        origin += dir * startNudge;

        RaycastHit hit;
        bool hasHit = Physics.SphereCast(
            origin,
            castRadiusMeters,
            dir,
            out hit,
            safeDistanceMeters,
            obstacleLayers,
            triggerInteraction
        );

        if (!hasHit) return false;

        // Ignore own collider if assigned
        if (selfCollider != null && hit.collider == selfCollider)
            return false;

        return true;
    }

    // Scene view visualization of the forward corridor
    void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        Transform aimTf = (cam != null) ? cam?.transform : transform;
        if (aimTf == null) return;

        Vector3 origin = aimTf.position;
        Vector3 dir = aimTf.forward;

        Gizmos.color = lastState ? gizmoColorBlocked : gizmoColorClear;

        // Draw a series of spheres to approximate the corridor volume
        float step = Mathf.Max(0.25f * cellSizeMeters, castRadiusMeters * 0.5f);
        float total = (cellSizeMeters > 0f && safeCells > 0) ? (cellSizeMeters * safeCells) : 0.0f;
        int steps = Mathf.Max(2, Mathf.CeilToInt(total / step));

        for (int i = 1; i <= steps; i++)
        {
            float d = (i / (float)steps) * total;
            Vector3 p = origin + dir * d;
            Gizmos.DrawSphere(p, Mathf.Max(0.02f, castRadiusMeters * 0.95f));
        }

        // Center line
        Gizmos.color = gizmoLine;
        Gizmos.DrawLine(origin, origin + dir * total);
    }
}
