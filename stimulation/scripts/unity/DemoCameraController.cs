using UnityEngine;

public class SimpleCameraSetup : MonoBehaviour
{
    [Header("Cameras")]
    public Camera overviewCamera;
    public Camera chaseCamera;
    public Camera fpvCamera;

    [Header("Drone to Follow")]
    public Transform droneTransform;

    [Header("Chase Camera Settings")]
    public Vector3 chaseOffset = new Vector3(0, 8, -15);
    public float smoothSpeed = 5f;

    private int currentCameraIndex = 0;

    void Start()
    {
        SetActiveCamera(0);
        Debug.Log("SimpleCameraSetup initialized. Press 1, 2, or 3 to switch cameras.");
    }

    void Update()
    {
        // Camera switching with number keys
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            SetActiveCamera(0); // Overview
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            SetActiveCamera(1); // Chase
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            SetActiveCamera(2); // FPV
        }

        // Update chase camera position if it's active
        if (currentCameraIndex == 1 && chaseCamera != null && droneTransform != null)
        {
            Vector3 desiredPosition = droneTransform.position + chaseOffset;
            chaseCamera.transform.position = Vector3.Lerp(
                chaseCamera.transform.position,
                desiredPosition,
                smoothSpeed * Time.deltaTime
            );
            chaseCamera.transform.LookAt(droneTransform);
        }

        // Update FPV camera if it's active
        if (currentCameraIndex == 2 && fpvCamera != null && droneTransform != null)
        {
            fpvCamera.transform.position = droneTransform.position + droneTransform.forward * 0.5f;
            fpvCamera.transform.rotation = droneTransform.rotation;
        }
    }

    void SetActiveCamera(int index)
    {
        currentCameraIndex = index;

        // Disable all cameras first
        if (overviewCamera != null) overviewCamera.enabled = false;
        if (chaseCamera != null) chaseCamera.enabled = false;
        if (fpvCamera != null) fpvCamera.enabled = false;

        switch (index)
        {
            case 0:
                if (overviewCamera != null)
                {
                    overviewCamera.enabled = true;
                    Debug.Log("Switched to Overview Camera");
                }
                break;
            case 1:
                if (chaseCamera != null)
                {
                    chaseCamera.enabled = true;
                    Debug.Log("Switched to Chase Camera");
                }
                break;
            case 2:
                if (fpvCamera != null)
                {
                    fpvCamera.enabled = true;
                    Debug.Log("Switched to FPV Camera");
                }
                break;
        }
    }
}