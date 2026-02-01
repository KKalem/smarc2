using UnityEditor;
using UnityEngine;


namespace Editor.Scripts
{
    [CustomEditor(typeof(DomainRandomization))]
    public class DomainRandomizationEditor : UnityEditor.Editor
    {
        DomainRandomization container;

        public override void OnInspectorGUI()
        {
            container = (DomainRandomization) target;
            DrawDefaultInspector();
            
            if (GUILayout.Button("Randomize All"))
            {
                container.RandomizeAll();
            }

            if (GUILayout.Button("Randomize Sun"))
            {
                container.RandomizeSun();
            }

            if (GUILayout.Button("Randomize SkyAndFog"))
            {
                container.RandomizeSkyAndFog();
            }

            if (GUILayout.Button("Randomize Cameras"))
            {
                container.RandomizeCameras();
            }


            if (GUILayout.Button("Randomize Water Surface"))
            {
                container.RandomizeWaterSurface();
            }
        }
    }
}