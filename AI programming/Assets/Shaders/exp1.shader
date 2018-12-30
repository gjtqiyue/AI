Shader "Custom/exp1"
{
	Properties{
		_MainTex("Texture", 2D) = "white"
	}

	SubShader
	{
		Tags { "RenderType"="Opaque" }
		LOD 100

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			
			#include "UnityCG.cginc"

			struct appdata
			{
				float4 vertex : POSITION;	
				float4 uv : TEXCOORD0;
			};

			struct v2f
			{		
				float4 vertex : SV_POSITION;
				float4 uv : TEXCOORD0;
			};

			v2f vert (appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = v.uv;
				return o;
			}
			
			fixed4 frag (v2f i) : SV_Target
			{
				return float4(i.uv.x, i.uv.y, 1, 1);
			}
			ENDCG
		}
	}
}
