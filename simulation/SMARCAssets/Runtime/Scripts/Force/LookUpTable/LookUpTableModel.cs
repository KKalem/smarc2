using Force.LookUpTable;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace DefaultNamespace.LookUpTable
{
    public class LookUpTableModel : MonoBehaviour
    {
        private Rigidbody rb;

        public void Start()
        {
            rb = GetComponent<Rigidbody>();
            var tablesFromJson = JsonUtils.TablesFromJson("lookupTable");
            DampingForceEquations.LookupTables = tablesFromJson;
        }

        public void FixedUpdate()
        {
            var (forces, moments) = DampingForceEquations.CalculateDamping(rb, transform);

            rb.AddRelativeForce(forces, ForceMode.Force);
           rb.AddRelativeTorque(moments, ForceMode.Force);
        }
    }
}