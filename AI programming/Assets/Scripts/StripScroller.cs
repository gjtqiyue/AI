using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/* ********************************************************* *
 * Strip scroller combined with normal background scroller   *
 * and offset scroller. Can only be used on quad since it    *
 * needs to manipulate the material. Add a 0.25 offset on x  *
 * every time pass through a cycle .                         *
 * ********************************************************* */
public class StripScroller : MonoBehaviour {

    public float scrollSpeed;
    public float tileSizeZ;

    private Vector3 startPosition;
    private new Renderer renderer;
    private Vector2 presetOffset;

    void Start () {
        startPosition = transform.position;
        renderer = GetComponent<Renderer>();

        // get the preset offset
        presetOffset = renderer.sharedMaterial.GetTextureOffset("_MainTex");
    }
	
	void Update () {
        Debug.Log(Time.time);
        float x = Mathf.Repeat(Time.time * scrollSpeed, tileSizeZ * 4);
        x = x / tileSizeZ;
        x = Mathf.Floor(x);
        x = x / 4;
        Vector2 offset = new Vector2(x, presetOffset.y);
        renderer.sharedMaterial.SetTextureOffset("_MainTex", offset);

        float newPosition = Mathf.Repeat(Time.time * scrollSpeed, tileSizeZ);
        transform.position = startPosition + Vector3.back * newPosition;
	}

    private void OnDisable()
    {
        renderer.sharedMaterial.SetTextureOffset("_MainTex", presetOffset);
    }
}
