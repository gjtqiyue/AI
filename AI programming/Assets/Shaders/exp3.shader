Shader "Custom/exp3" {

	properties{
		_Tint("Tint", Color) = (1, 1, 1, 1)
	}

	SubShader {

		// having more than one pass means that the object gets rendered multiple times
		Pass {

		CGPROGRAM

		#pragma vertex vert
	    #pragma fragment frag

        #include "UnityCG.cginc" 

		// vertex info
		float4 vert(float4 position : POSITION) : SV_POSITION { // SV stands for system value
			return UnityObjectToClipPos(position);
		}

		float4 _Tint;

		// color for each pixel to render
		float4 frag(float4 ver : SV_POSITION, float3 localPos : TEXCOORD0) : SV_TARGET { // SV_TARGET is the default shader target, which needs to use the output from vertex function
			
			return float4(localPosition, 1);
		}

		ENDCG

		}
	}
}
