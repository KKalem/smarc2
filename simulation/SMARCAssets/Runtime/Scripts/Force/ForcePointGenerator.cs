using UnityEngine;


namespace Force
{
    public class ForcePointGenerator : MonoBehaviour
    {
        [Header("Placement")]
        [Tooltip("Number of ForcePoints to generate along each axis.")]
        public int CountX = 2;
        public int CountY = 2;
        public int CountZ = 10;
        public Vector3 Size = new(1f, 1f, 1f);

        public Vector3 CenterOffset = new(0f, 0f, 0f);

        [Header("ForcePoint Settings")]
        [Tooltip("The ForcePoint to copy settings from. Configure this ForcePoint how you want all the generated ForcePoints to be.")]
        public GameObject ExampleToCopy;

        [Header("Gizmos")]
        public float GizmoSize = 0.1f;

        public void Generate()
        {
            if (ExampleToCopy == null)
            {
                Debug.LogError("No ExampleToCopy set for ForcePointGenerator");
                return;
            }

            var exampleForcePoint = ExampleToCopy.GetComponent<ForcePoint>();
            if (exampleForcePoint == null)
            {
                Debug.LogError("ExampleToCopy does not have a ForcePoint component");
                return;
            }

            DestroyForcePoints();
            var points = GetPoints();
            foreach (var point in points)
            {
                var newFP = Instantiate(ExampleToCopy, transform.position + point, Quaternion.identity, transform);
                newFP.name = $"ForcePoint_{point}";
                newFP.SetActive(true);
            }
        }

        public void DestroyForcePoints()
        {
            var existingForcePoints = GetComponentsInChildren<ForcePoint>();
            foreach (var forcePoint in existingForcePoints)
            {
                if (forcePoint.gameObject.activeSelf)
                {
                    if (Application.isPlaying)
                        Destroy(forcePoint.gameObject);
                    else
                        DestroyImmediate(forcePoint.gameObject);
                }
            }
        }


        Vector3[] GetPoints()
        {
            Vector3[] points = new Vector3[CountX * CountY * CountZ];
            int index = 0;
            for (int x = 0; x < CountX; x++)
            {
                for (int y = 0; y < CountY; y++)
                {
                    for (int z = 0; z < CountZ; z++)
                    {
                        float posX = (x - (CountX - 1) / 2f) * (Size.x / Mathf.Max(1, CountX - 1));
                        float posY = (y - (CountY - 1) / 2f) * (Size.y / Mathf.Max(1, CountY - 1));
                        float posZ = (z - (CountZ - 1) / 2f) * (Size.z / Mathf.Max(1, CountZ - 1));
                        points[index++] = new Vector3(posX, posY, posZ) + CenterOffset;
                    }
                }
            }
            return points;
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.cyan;
            var points = GetPoints();
            foreach (var point in points)
            {
                Gizmos.DrawSphere(transform.position + point, GizmoSize);
            }
        }
    }
}