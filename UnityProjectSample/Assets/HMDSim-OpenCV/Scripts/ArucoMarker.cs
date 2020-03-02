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
    public HMDSimOpenCV.ARUCO_PREDEFINED_DICTIONARY MarkerDictionary = HMDSimOpenCV.ARUCO_PREDEFINED_DICTIONARY.DICT_5X5_250;
    public int markerId = 42;

    private byte[] textureBuffer;
    Texture2D markerTexture;


    private int _markerId = -1;
    private HMDSimOpenCV.ARUCO_PREDEFINED_DICTIONARY _oldDictionary;

    private void OnEnable()
    {
        // limit resolution to avoid crashing opencv / unity
        // a minimum resolution is required per Aruco
        if (resolution < 25 || resolution > 4096)
            resolution = 512;


        UpdateMarkerSettings();
    }

    public void UpdateMarkerSettings()
    {
        bool needToGenerateAgain = false;

        // do we need to allocate again?
        if (textureBuffer == null || textureBuffer.Length != (resolution * resolution * 3))
        {
            // allocate a new buffer
            textureBuffer = new byte[resolution * resolution * 3];

            // update the texture in the picture
            markerTexture = new Texture2D(resolution, resolution, TextureFormat.RGB24, false);

            // update material on the plane
            Renderer r = this.GetComponent<Renderer>();
            r.material.mainTexture = markerTexture;

            // generate texture again
            needToGenerateAgain = true;
        }

        if (markerId != _markerId)
        {
            _markerId = markerId;
            needToGenerateAgain = true;
        }

        if (MarkerDictionary != _oldDictionary)
        {
            _oldDictionary = MarkerDictionary;
            needToGenerateAgain = true;
        }

        if (needToGenerateAgain)
        {
            // draw the image
            bool generatedWell = HMDSimOpenCV.Aruco_DrawMarker((int)_oldDictionary, _markerId, resolution, true, textureBuffer);

            // update the texture
            markerTexture.LoadRawTextureData(textureBuffer);
            markerTexture.Apply();

            if (generatedWell)
            {
                Debug.Log(string.Format("[ArucoMarker] Generated marker id={0} ({1}) with {2}x{2}", _markerId, _oldDictionary.ToString(), resolution));
            } else
            {
                Debug.LogError(string.Format("[ArucoMarker] Error generating marker id={0} ({1}) with {2}x{2}", _markerId, _oldDictionary.ToString(), resolution));
            }
        }

    }

}
