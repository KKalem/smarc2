using UnityEngine;

namespace DefaultNamespace.Water
{
    public class ObjectWaterQueryModel : WaterQueryModel
    {
        public override float GetWaterLevelAt(Vector3 position)
        {
            return transform.position.y;
        }
    }
}
