using UnityEngine;

namespace DefaultNamespace.Water
{
    public abstract class WaterQueryModel: MonoBehaviour
    {

        public abstract float GetWaterLevelAt(Vector3 position);
    }
}
