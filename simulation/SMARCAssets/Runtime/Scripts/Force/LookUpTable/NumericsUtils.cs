using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DefaultNamespace.LookUpTable
{
    public static class NumericsUtils
    {
        public static Vector<double> ToDense(this Vector3 vec)
        {
            return Vector.Build.Dense(new[] { (double)vec.x, vec.y, vec.z });
        }
        
        public static Vector<double> ToDense(this Vector3<NED> vec)
        {
            return Vector.Build.Dense(new[] { (double)vec.x, vec.y, vec.z });
        }
        
        public static Vector<double> ToDense(this Vector3<ENU> vec)
        {
            return Vector.Build.Dense(new[] { (double)vec.x, vec.y, vec.z });
        }
    }
}