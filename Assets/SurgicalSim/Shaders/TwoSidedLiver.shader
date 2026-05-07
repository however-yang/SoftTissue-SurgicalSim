Shader "SurgicalSim/TwoSidedLiver"
{
    Properties
    {
        _MainTex ("Liver Texture", 2D) = "white" {}
        _Color ("Liver Surface Color", Color) = (0.85, 0.25, 0.15, 1)
        _InteriorColor ("Cut Interior Color", Color) = (0.85, 0.45, 0.45, 1)
        _TextureStrength ("Texture Strength", Range(0, 1)) = 0.65
        _TriplanarScale ("Triplanar Scale", Float) = 2.5
        _TriplanarBlend ("Triplanar Blend", Range(0.1, 16)) = 4
        _Smoothness ("Smoothness", Range(0, 1)) = 0.3
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" "Queue"="Geometry" }

        Pass
        {
            Name "TwoSidedForward"
            Tags { "LightMode"="UniversalForward" }
            Cull Off

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
                float3 restOS     : TEXCOORD1;
            };

            struct Varyings
            {
                float4 positionCS  : SV_POSITION;
                float3 normalWS    : TEXCOORD0;
                float3 positionWS  : TEXCOORD1;
                float3 positionOS  : TEXCOORD2;
                float3 normalOS    : TEXCOORD3;
                float3 restOS      : TEXCOORD4;
            };

            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);

            CBUFFER_START(UnityPerMaterial)
                half4 _Color;
                half4 _InteriorColor;
                float4 _MainTex_ST;
                half  _TextureStrength;
                half  _Smoothness;
                float _TriplanarScale;
                float _TriplanarBlend;
            CBUFFER_END

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs vpi = GetVertexPositionInputs(input.positionOS.xyz);
                output.positionCS = vpi.positionCS;
                output.positionWS = vpi.positionWS;
                output.positionOS = input.positionOS.xyz;
                output.normalOS = normalize(input.normalOS);
                output.restOS = input.restOS;
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                return output;
            }

            half3 SampleLiverTriplanar(float3 positionOS, float3 normalOS)
            {
                float3 weights = abs(normalize(normalOS));
                weights = max(weights, float3(1e-4, 1e-4, 1e-4));
                float blend = max(_TriplanarBlend, 1e-3);
                weights = pow(weights, float3(blend, blend, blend));
                weights /= max(weights.x + weights.y + weights.z, 1e-5);

                float scale = max(_TriplanarScale, 1e-5);
                float2 texScale = max(abs(_MainTex_ST.xy), float2(1e-5, 1e-5));
                float2 texOffset = _MainTex_ST.zw;

                float2 uvX = positionOS.zy * scale * texScale + texOffset;
                float2 uvY = positionOS.xz * scale * texScale + texOffset;
                float2 uvZ = positionOS.xy * scale * texScale + texOffset;

                half3 sampleX = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvX).rgb;
                half3 sampleY = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvY).rgb;
                half3 sampleZ = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvZ).rgb;

                return sampleX * weights.x + sampleY * weights.y + sampleZ * weights.z;
            }

            half4 frag(Varyings input, half facing : VFACE) : SV_Target
            {
                float3 normal = normalize(input.normalWS) * facing;

                half3 texColor = SampleLiverTriplanar(input.restOS, input.normalOS);
                half3 surfaceAlbedo = lerp(_Color.rgb, texColor, saturate(_TextureStrength));
                half3 albedo = (facing > 0) ? surfaceAlbedo : _InteriorColor.rgb;

                Light mainLight = GetMainLight();
                float NdotL = saturate(dot(normal, mainLight.direction));

                float3 ambient = albedo * 0.3;
                float3 diffuse = albedo * mainLight.color.rgb * NdotL * 0.7;

                float3 viewDir = GetWorldSpaceNormalizeViewDir(input.positionWS);
                float3 halfDir = normalize(mainLight.direction + viewDir);
                float spec = pow(saturate(dot(normal, halfDir)), lerp(4.0, 64.0, _Smoothness));
                float3 specular = mainLight.color.rgb * spec * 0.15;

                return half4(ambient + diffuse + specular, 1.0);
            }
            ENDHLSL
        }

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
                posWS += normalWS * 0.001;
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
