using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;

using GPSRef = GeoRef.GlobalReferencePoint;
using ROS.Core;



namespace ROS.Publishers
{
    public class UTMtoUnityOriginPublisher: ROSPublisher<TFMessageMsg>
    {
        GPSRef gpsRef;
        TransformStampedMsg utmToUnityMsg, utmZBToUtmMsg;


        protected override void InitPublisher()
        {
            topic = "/tf";
            var utmpubs = FindObjectsByType<UTMtoUnityOriginPublisher>(FindObjectsSortMode.None);
            if(utmpubs.Length > 1)
            {
                Debug.LogWarning("Found too many UTM->unity_origin publishers in the scene, there should only be one!");
            }

            var gpsRefs = FindObjectsByType<GPSRef>(FindObjectsSortMode.None);
            if(gpsRefs.Length < 1)
            {
                Debug.LogWarning("[UTM->Map pub] No Global Reference Point found in the scene. There must be at least one! Disabling UTM->Map publisher.");
                enabled = false;
                return;
            }
            if(gpsRefs.Length > 1)
            {
                Debug.LogWarning("[UTM->Map pub] Found too many Global Reference Points in the scene, there should only be one! Using the first!");
            }
            
            gpsRef = gpsRefs[0];

            // make sure this is in the origin
            // why origin? so that we can tell all other tf publishers
            // in the scene to publish a "global" frame that is unity_origin
            // and they wont need to do any origin shenanigans that way
            transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

            // this is the position of unity-world in utm coordinates
            var (originEasting, originNorthing, _, _) = gpsRef.GetUTMLatLonOfObject(gameObject);
            var utm_zone_band = $"utm_{gpsRef.UTMZone}_{gpsRef.UTMBand}";

            var unity_origin_in_utm = new TransformMsg();
            unity_origin_in_utm.translation.x = originEasting;
            unity_origin_in_utm.translation.y = originNorthing;

            utmToUnityMsg = new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), "utm"), //header
                "unity_origin", //child frame_id
                unity_origin_in_utm
            );

            // also create a dummy utm_Z_B -> utm tf for people
            // that do not care about actual global location...
            utmZBToUtmMsg = new TransformStampedMsg
            (
                new HeaderMsg(new TimeStamp(Clock.time), utm_zone_band), //header
                "utm", //child frame_id
                new TransformMsg() // 0-transform
            );

            List<TransformStampedMsg> tfMessageList = new List<TransformStampedMsg>
            {
                utmZBToUtmMsg,
                utmToUnityMsg
            };
            // These transforms never change during play mode
            // so we can publish the same message all the time
            ROSMsg = new TFMessageMsg(tfMessageList.ToArray());
        }

        protected override void UpdateMessage()
        {
            // these are static transforms, they just change stamps...
            var stamp = new TimeStamp(Clock.time);
            utmToUnityMsg.header.stamp = stamp;
            utmZBToUtmMsg.header.stamp = stamp;

            List<TransformStampedMsg> tfMessageList = new List<TransformStampedMsg>
            {
                utmZBToUtmMsg,
                utmToUnityMsg
            };
            // These transforms never change during play mode
            // so we can publish the same message all the time
            ROSMsg = new TFMessageMsg(tfMessageList.ToArray());
        }
    }
}