using UnityEngine;

public class GroundTargetMover : MonoBehaviour
{
    public enum DriveMode { KeyboardWASD, CircleAuto, Waypoints }

    [Header("Motion")]
    public DriveMode mode = DriveMode.KeyboardWASD;
    public float moveSpeed = 3.0f;          // m/s on the ground plane
    public float turnSpeed = 360f;          // deg/s yaw rotation while moving

    [Header("Grounding")]
    public bool snapToGround = true;        // true = raycast down to ground; false = lock to fixedY
    public float fixedY = 0f;               // used when snapToGround = false (e.g., a Plane at y=0)
    public float groundOffset = 0.0f;       // raise a bit above ground to avoid z-fighting
    public float maxSnapDistance = 10f;     // how far down to raycast for ground
    public LayerMask groundLayers = ~0;     // which layers count as ground

    [Header("Circle Auto")]
    public float circleRadius = 5f;
    public float circleAngularSpeedDeg = 30f; // deg/s
    public Transform circleCenterTransform;   // optional: if set, use this as center
    public Vector3 circleCenterWorld;         // fallback world position center
    public bool useInitialPositionAsCenter = true; // if true and no transform, capture start position as center

    [Header("Waypoints")]
    public Transform[] waypoints;
    public float waypointArriveDistance = 0.2f;

    private int _wpIndex = 0;
    private float _circleAngleDeg = 0f;
    private bool _centerCaptured = false;

    void Start()
    {
        // Capture initial position as center if requested and no transform provided
        if (useInitialPositionAsCenter && circleCenterTransform == null)
        {
            circleCenterWorld = transform.position;
            _centerCaptured = true;
        }
    }

    void Update()
    {
        Vector3 desiredMoveXZ = Vector3.zero;
        float dt = Time.deltaTime;

        switch (mode)
        {
            case DriveMode.KeyboardWASD:
            {
                float h = Input.GetAxisRaw("Horizontal"); // A/D
                float v = Input.GetAxisRaw("Vertical");   // W/S
                Vector3 dir = new Vector3(h, 0f, v);
                if (dir.sqrMagnitude > 1e-4f)
                {
                    dir.Normalize();
                    desiredMoveXZ = dir * moveSpeed * dt;
                    Quaternion targetRot = Quaternion.LookRotation(dir, Vector3.up);
                    transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, turnSpeed * dt);
                }
                break;
            }

            case DriveMode.CircleAuto:
            {
                // Advance angle
                _circleAngleDeg += circleAngularSpeedDeg * dt;
                float rad = _circleAngleDeg * Mathf.Deg2Rad;
            
                // Choose center
                Vector3 center = circleCenterTransform
                    ? circleCenterTransform.position
                    : circleCenterWorld;
            
                // Parametric circle position on XZ plane
                Vector3 targetPosXZ = center + new Vector3(Mathf.Cos(rad), 0f, Mathf.Sin(rad)) * circleRadius;
            
                // Desired displacement this frame (XZ only)
                Vector3 flatCurrent = new Vector3(transform.position.x, 0f, transform.position.z);
                Vector3 to = targetPosXZ - flatCurrent;
            
                // Move directly to the parametric point with speed cap (optional)
                Vector3 step = Vector3.ClampMagnitude(to, moveSpeed * dt);
                desiredMoveXZ = new Vector3(step.x, 0f, step.z);
            
                // Face tangent direction (cos,sin) tangent is (-sin, cos)
                Vector3 tangent = new Vector3(-Mathf.Sin(rad), 0f, Mathf.Cos(rad));
                if (tangent.sqrMagnitude > 1e-6f)
                {
                    Quaternion targetRot = Quaternion.LookRotation(tangent, Vector3.up);
                    transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, turnSpeed * dt);
                }
                break;
            }

            case DriveMode.Waypoints:
            {
                if (waypoints != null && waypoints.Length > 0)
                {
                    Transform wp = waypoints[_wpIndex % waypoints.Length];
                    if (wp != null)
                    {
                        Vector3 to = new Vector3(wp.position.x - transform.position.x, 0f, wp.position.z - transform.position.z);
                        if (to.magnitude < waypointArriveDistance)
                        {
                            _wpIndex++;
                        }
                        else if (to.sqrMagnitude > 1e-6f)
                        {
                            Vector3 dir = to.normalized;
                            desiredMoveXZ = dir * moveSpeed * dt;
                            Quaternion targetRot = Quaternion.LookRotation(dir, Vector3.up);
                            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, turnSpeed * dt);
                        }
                    }
                }
                break;
            }
        }

        // Apply XZ movement (preserve current Y for now)
        Vector3 pos = transform.position + desiredMoveXZ;

        // Constrain to ground
        if (snapToGround)
        {
            Vector3 rayStart = pos + Vector3.up * (maxSnapDistance * 0.5f);
            float rayLen = maxSnapDistance;
            if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, rayLen, groundLayers, QueryTriggerInteraction.Ignore))
            {
                pos.y = hit.point.y + groundOffset;
            }
        }
        else
        {
            pos.y = fixedY + groundOffset;
        }

        transform.position = pos;
    }

    void OnDrawGizmosSelected()
    {
        if (mode == DriveMode.CircleAuto)
        {
            Gizmos.color = Color.cyan;
            Vector3 center = circleCenterTransform ? circleCenterTransform.position : circleCenterWorld;
            float y = snapToGround ? transform.position.y : fixedY;
            Gizmos.DrawWireSphere(new Vector3(center.x, y, center.z), circleRadius);
            Gizmos.DrawLine(new Vector3(center.x, y, center.z), new Vector3(transform.position.x, y, transform.position.z));
        }

        if (mode == DriveMode.Waypoints && waypoints != null)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < waypoints.Length; i++)
            {
                if (waypoints[i] == null) continue;
                Gizmos.DrawSphere(waypoints[i].position, 0.1f);
                if (i + 1 < waypoints.Length && waypoints[i + 1] != null)
                    Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
            }
        }
    }
}
