using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OffsetScroller : MonoBehaviour {

    public float scrollSpeed;

    private new Renderer renderer;
    Vector2 presetOffset;

    // Use this for initialization
    void Start () {
        renderer = GetComponent<Renderer>();

        // get the preset offset
        presetOffset = renderer.sharedMaterial.GetTextureOffset("_MainTex");
	}
	
	
	void Update () {
        float y = Mathf.Repeat(Time.time * scrollSpeed, 1);
        Vector2 offset = new Vector2(presetOffset.x, y);
        renderer.sharedMaterial.SetTextureOffset("_MainTex", offset);
	}

    private void OnDisable()
    {
        renderer.sharedMaterial.SetTextureOffset("_MainTex", presetOffset);
    }
}
