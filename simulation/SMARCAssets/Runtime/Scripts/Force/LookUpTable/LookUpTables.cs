using System;

using UnityEngine;

namespace Force.LookUpTable
{
    [Serializable]
    public class LookUpTables
    {
        public Double[][] xcp;
        public Double[][] cz;
        public Double[][] cy;
        public Double[][] cx;

        public static LookUpTables CreateFromJSON(string jsonString)
        {
            return new LookUpTables(); // TODO FIX JsonConvert.DeserializeObject<LookUpTables>(jsonString);
        }
    }
}