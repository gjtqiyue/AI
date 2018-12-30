Shader "Custom/exp3" {

	properties{
		_Tint("Tint", Color) = (1, 1, 1, 1)
		_MainTex("Texture", 2D) = "White" {}
	}

	SubShader {

		// having more than one pass means that the object gets rendered multiple times
		Pass {

		CGPROGRAM

		#pragma vertex vert
	    #pragma fragment frag

        #include "UnityCG.cginc" 

		float4 _Tint;
	    sampler2D _MainTex;
		float4 _MainTex_ST;

	    // using data structure to pass more data to the vertex program
		struct VertexData {
			float4 position : POSITION;
			float2 uv : TEXCOORD0;
		};

		// using data structure to pass more data to the frag program
		struct Interpolators {
			float4 position : SV_POSITION;
			float2 uv : TEXCOORD0;
		};

		// vertex program
		Interpolators vert(VertexData v)  {
			Interpolators i;
		    
			// _MainTex_ST.xy controls tiling and zw controls offset
			//i.uv = v.uv * _MainTex_ST.xy + _MainTex_ST.zw;
			i.uv = TRANSFORM_TEX(v.uv, _MainTex);
	        i.position = UnityObjectToClipPos(v.position);

			return i;
		}

		// color for each pixel to render
		float4 frag(Interpolators i) : SV_TARGET { // SV_TARGET is the default shader target, which needs to use the output from vertex function
			return tex2D(_MainTex, i.uv) * _Tint;
		}

		ENDCG

		}
	}
}
