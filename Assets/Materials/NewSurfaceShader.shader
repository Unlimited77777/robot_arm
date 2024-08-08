Shader "Custom/DepthShader"
{
    Properties { }
    SubShader
    {
        Pass
        {
            ZWrite On
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float depth : TEXCOORD0; // 使用 TEXCOORD0 来传递深度值
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.depth = o.pos.z / o.pos.w; // 正确计算深度值
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float depth = Linear01Depth(i.depth); // 使用正确的深度值
                return fixed4(depth, depth, depth, 1); // 输出灰度值
            }
            ENDCG
        }
    }
}
