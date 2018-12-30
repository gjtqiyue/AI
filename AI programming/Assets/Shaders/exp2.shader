Shader "Custom/exp2"
{
	Properties{
		_MainTex("Texture", 2D) = "white" {}
		_Color("Color", Color) = (1, 1, 1, 1)
	}

	SubShader
	{
		Tags{ "Queue" = "Transparent" }
		Pass
		{
			
		    //ZWrite Off
		    Blend SrcAlpha OneMinusSrcAlpha

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
				o.uv = v.uv * 2; // tiling by 2
				return o;
			}

			sampler2D _MainTex;
			float4 _Color;
			
			fixed4 frag (v2f i) : SV_Target
			{
				float4 color = tex2D(_MainTex, i.uv)  * float4(i.uv.r, i.uv.g, 0, 1);
				return color;
			}
			ENDCG
		}
	}
}
