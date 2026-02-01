using SmarcGUI.MissionPlanning.Params;

namespace SmarcGUI.MissionPlanning.Tasks
{

    public class AlarsBT : Task
    {
        public override void SetParams()
        {
            Name = "alars-bt";
            Description = "Run the entire ALARS system";
            Params.Add("initial_travel_alt", 25.0f);
            Params.Add("search_position", new GeoPoint());
            Params.Add("delivery_position", new GeoPoint());
            Params.Add("forward_distance", 2.0f);
            Params.Add("forward_altitude", 3.0f);
            Params.Add("dipping_altitude", 7.0f);
            Params.Add("raising_altitude", 15.0f);
        }
    }

    public class AlarsSearch : Task
    {
        public override void SetParams()
        {
            Name = "alars-search";
            Description = "Search for an AUV in the water";
            Params.Add("search_position", new GeoPoint());
        }
    }

    public class AlarsRecover : Task
    {
        public override void SetParams()
        {
            Name = "alars-recover";
            Description = "Hook a rope in the water";
            Params.Add("object_position", new GeoPoint());
            Params.Add("buoy_position", new GeoPoint());
            Params.Add("forward_distance", 2.0f);
            Params.Add("forward_altitude", 3.0f);
            Params.Add("dipping_altitude", 7.0f);
            Params.Add("raising_altitude", 15.0f);
        }
    }

    public class AlarsLocalize : Task
    {
        public override void SetParams()
        {
            Name = "alars-localize";
            Description = "Localize an AUV in the water by moving above the camera detection points";
            Params.Add("localize_auv", true);
            Params.Add("localize_buoy", false);
        }
    }

}