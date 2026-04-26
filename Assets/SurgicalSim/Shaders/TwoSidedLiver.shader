Shader "SurgicalSim/TwoSidedLiver"
{
    Properties
    {
        _Color ("Liver Surface Color", Color) = (0.85, 0.25, 0.15, 1)
        _InteriorColor ("Cut Interior Color", Color) = (0.85, 0.45, 0.45, 1)
        _Smoothness ("Smoothness", Range(0, 1)) = 0.3
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" "Queue"="Geometry" }

        // 单 Pass 双面渲染：用 VFACE 判断正反面
        Pass
        {
            Name "TwoSidedForward"
            Tags { "LightMode"="UniversalForward" }
            Cull Off  // 关闭背面剔除 = 双面渲染

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _ADDITIONAL_LIGHTS

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS   : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS  : SV_POSITION;
                float3 normalWS    : TEXCOORD0;
                float3 positionWS  : TEXCOORD1;
            };

            CBUFFER_START(UnityPerMaterial)
                half4 _Color;
                half4 _InteriorColor;
                half  _Smoothness;
            CBUFFER_END

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs vpi = GetVertexPositionInputs(input.positionOS.xyz);
                output.positionCS  = vpi.positionCS;
                output.positionWS  = vpi.positionWS;
                output.normalWS    = TransformObjectToWorldNormal(input.normalOS);
                return output;
            }

            // VFACE: 正面 = +1, 背面 = -1
            half4 frag(Varyings input, half facing : VFACE) : SV_Target
            {
                // 背面翻转法线
                float3 normal = normalize(input.normalWS) * facing;

                // 根据正反面选颜色
                half3 albedo = (facing > 0) ? _Color.rgb : _InteriorColor.rgb;

                // 简单 PBR 光照
                Light mainLight = GetMainLight();
                float NdotL = saturate(dot(normal, mainLight.direction));

                // 漫反射 + 环境光
                float3 ambient = albedo * 0.3;
                float3 diffuse = albedo * mainLight.color.rgb * NdotL * 0.7;

                // 视角相关高光（简单 Blinn-Phong）
                float3 viewDir = GetWorldSpaceNormalizeViewDir(input.positionWS);
                float3 halfDir = normalize(mainLight.direction + viewDir);
                float spec = pow(saturate(dot(normal, halfDir)), lerp(4.0, 64.0, _Smoothness));
                float3 specular = mainLight.color.rgb * spec * 0.15;

                return half4(ambient + diffuse + specular, 1.0);
            }
            ENDHLSL
        }

        // ShadowCaster pass（阴影投射）
        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode"="ShadowCaster" }
            Cull Off

            HLSLPROGRAM
            #pragma vertex ShadowVert
            #pragma fragment ShadowFrag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS   : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
            };

            float3 _LightDirection;

            Varyings ShadowVert(Attributes input)
            {
                Varyings output;
                float3 posWS = TransformObjectToWorld(input.positionOS.xyz);
                float3 normalWS = TransformObjectToWorldNormal(input.normalOS);
                // 法线偏移防止自阴影
                posWS = posWS + normalWS * 0.001;
                output.positionCS = TransformWorldToHClip(posWS);
                return output;
            }

            half4 ShadowFrag(Varyings input) : SV_Target
            {
                return 0;
            }
            ENDHLSL
        }
    }
    FallBack "Universal Render Pipeline/Lit"
}
