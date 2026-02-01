using UnityEngine;

public class Gimbal : MonoBehaviour
{
    void Start()
    {
        DoGimbal(allowLarge: true);
    }

    void LateUpdate()
    {
        DoGimbal(allowLarge: false);
    }

    void DoGimbal(bool allowLarge)
    {
        if (transform.parent == null)
            return;

        // 1) Extract parent's yaw (rotation around world up)
        Vector3 parentForward = Vector3.ProjectOnPlane(transform.parent.forward, Vector3.up);
        Quaternion yawOnly = Quaternion.identity;

        if (parentForward.sqrMagnitude > 1e-6f)
        {
            yawOnly = Quaternion.LookRotation(parentForward.normalized, Vector3.up);
        }

        // 2) Define "down" forward and "up" vector based on parent's yaw
        Vector3 forward = Vector3.down;
        Vector3 up = yawOnly * Vector3.forward;

        // 3) Apply rotation with threshold check
        Quaternion newRotation = Quaternion.LookRotation(forward, up);
        
        // Could be useful to do once at the start
        if (allowLarge)
        {
            transform.rotation = newRotation;
            return;
        }

        // maximum allowed rotation change in degrees (adjust this variable as needed)
        float maxAngleDegrees = 30f;

        float angleDelta = Quaternion.Angle(transform.rotation, newRotation);

        if (angleDelta <= maxAngleDegrees)
        {
            transform.rotation = newRotation;
        }
        else
        {
            Debug.LogWarning($"Gimbal rotation change of {angleDelta}° exceeds the maximum allowed {maxAngleDegrees}°. Rotation not applied.");
        }

    }
}
