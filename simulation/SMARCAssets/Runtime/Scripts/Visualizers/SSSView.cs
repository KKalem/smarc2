using System;
using UnityEngine;

using SSS = VehicleComponents.Sensors.Sonar;

namespace Visualizers
{
    public class SSSView : MonoBehaviour
    {
        int textureWidth = 64;
        int textureHeight = 64;
        Texture2D texture;

        SSS sss;

        [Header("Sidescan viewer")]
        public int viewX=0;
        public int viewY=30;
        public int viewHeight = 500;
        public int viewWidth = 1000;

        [Tooltip("If true, then new lines will be from the top, otherwise from the bottom")]
        public bool flip=true;
        
        // the raw data to copy into texture
        byte[] line;
        byte[] reversePort;
        byte[] image;
        byte[] flippedImage;

        void CreateNewTexture ()
        {
            // R8 is a 1byte format, all-red, good for SSS data
            // Alternatively Alpha8 is the same, for alpha channel instead
            texture = new Texture2D(
                width:textureWidth,
                height:textureHeight,
                textureFormat:TextureFormat.R8, 
                mipCount:1,
                linear:true
            );
            texture.Apply();
        }

        void Start()
        {
            sss = GetComponent<SSS>();
            textureWidth = sss.NumBucketsPerBeam * 2;
            textureHeight = 500;
            CreateNewTexture();
            line = new byte[textureWidth];
            image = new byte[textureWidth*textureHeight];
            flippedImage = new byte[image.Length];
            reversePort = new byte[sss.NumBucketsPerBeam];
        }

        void OnGUI()
        {
            GUI.DrawTexture(
                position:new Rect(viewX, viewY, width:viewWidth, height:viewHeight),
                image:texture,
                scaleMode:ScaleMode.ScaleToFit,
                alphaBlend:false
            );
        }

        void FixedUpdate()
        {
            // First, create the current line
            // We want port(left) to be reversed so the nadir is in the middle
            // Buffer.BlockCopy(sss.portBuckets, 0, reversePort, 0, sss.numBucketsPerSide);
            Buffer.BlockCopy(sss.Buckets, 0, reversePort, 0, sss.NumBucketsPerBeam);
            Array.Reverse(reversePort, 0, reversePort.Length);

            // Finally put the two sides together into one line
            Buffer.BlockCopy(reversePort, 0, line, 0, reversePort.Length);
            // Buffer.BlockCopy(sss.strbBuckets, 0, line, sss.numBucketsPerSide, sss.numBucketsPerSide);
            Buffer.BlockCopy(sss.Buckets, sss.NumBucketsPerBeam, line, sss.NumBucketsPerBeam, sss.NumBucketsPerBeam);

            // Draw a dotted line in the middle
            byte t = 0;
            if((int)Time.time %2 == 0) t = (byte)255;
            line[sss.NumBucketsPerBeam] = t;
            line[sss.NumBucketsPerBeam+1] = t;

            // Scroll the current image by 1 pixel down
            // Since image is a flattened array, that means
            // shifting by width and copying one width-worth fewer pixels
            // to leave room at the top
            // src, srcOff, dst, dstOff, count
            Buffer.BlockCopy(image, 0, image, textureWidth, textureWidth*(textureHeight-1));

            // Copy the new line at the top
            Buffer.BlockCopy(line, 0, image, 0, textureWidth);

            if(flip)
            {
                // Flip it upside-down
                for(int i=0; i<textureHeight; i++)
                {
                    Buffer.BlockCopy(image, i*textureWidth, flippedImage, (textureHeight-i-1)*textureWidth, textureWidth);
                }
                // Straight up set the texture data to our bytes. 
                texture.SetPixelData(flippedImage, 0);
            }
            else
            {
                texture.SetPixelData(image, 0);
            }
            texture.Apply();
        }
    }
}