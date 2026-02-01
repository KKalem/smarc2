using System.IO;
using UnityEngine;

namespace Force.LookUpTable
{
    public static class JsonUtils
    {
        public static LookUpTables TablesFromJson(string filename)
        {
            return LookUpTables.CreateFromJSON(ReadJsonFromFile(filename));
        }

        public static string ReadJsonFromFile(string filename)
        {
            string textContent = "";

            if (filename != null && filename.Length > 0)
            {
                string path = Application.dataPath + "/Resources/Text/" + filename + ".json";
                textContent = File.ReadAllText(path);
            }

            if (textContent.Length == 0)
            {
                throw new FileNotFoundException("No file found trying to load text from file (" + filename + ")... - please check the configuration");
            }

            return textContent;
        }
    }
}