using UnityEngine;

namespace Force
{
    public interface IForceModel
    {
        public Vector3 GetTorqueDamping();
        public Vector3 GetForceDamping();

       

    }
}
