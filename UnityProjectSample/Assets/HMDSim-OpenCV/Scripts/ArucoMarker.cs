using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// This class is responsible for showing an ArucoMarker on Unity Editor / Game
/// so that it can be tracked by locatable cameras in the simulator
/// </summary>
public class ArucoMarker : MonoBehaviour
{

    public int resolution = 512;
    //public bool border = true; (disabling this for now)
    public HMDSimOpenCV.ARUCO_PREDEFINED_DICTIONARY MarkerDictionary;
    public int markerId;

    private byte[] textureBuffer;
    Texture2D markerTexture;

    private void OnEnable()
    {
        // limit resolution to avoid crashing opencv / unity
        // a minimum resolution is required per Aruco
        if (resolution < 100 || resolution > 4096)
            resolution = 512;


        textureBuffer = new byte[resolution * resolution * 3];

        // draw the image
        HMDSimOpenCV.Aruco_DrawMarker((int)MarkerDictionary, markerId, resolution, true, textureBuffer);

        // update the texture in the picture
        markerTexture = new Texture2D(resolution, resolution, TextureFormat.RGB24, false);
        markerTexture.LoadRawTextureData(textureBuffer);
        markerTexture.Apply();

        // update material on the plane
        Renderer r = this.GetComponent<Renderer>();
        r.material.mainTexture = markerTexture;
    }

}
