using UnityEditor;
using UnityEngine;

using Force;

namespace Editor.Scripts
{
    [CustomEditor(typeof(ForcePointGenerator))]
    public class ForcePointGeneratorEditor : UnityEditor.Editor
    {
        ForcePointGenerator container;

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            container = (ForcePointGenerator)target;

            if (GUILayout.Button("(Re)Generate Force Points"))
            {
                container.DestroyForcePoints();
                container.Generate();
            }

            if (GUILayout.Button("Destroy Generated Force Points"))
            {
                container.DestroyForcePoints();
            }
        }
        
    }
}