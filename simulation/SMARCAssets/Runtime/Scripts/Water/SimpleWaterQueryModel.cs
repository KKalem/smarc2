using UnityEngine;

namespace DefaultNamespace.Water
{
    public class SimpleWaterQueryModel : WaterQueryModel
    {
        public float water_level_z = 0.0f;
        public override float GetWaterLevelAt(Vector3 position)
        {
            return water_level_z;
        }
    }
}
