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

            float3 SafeNormalize3(float3 v, float3 fallback)
            {
                float lenSq = dot(v, v);
                return lenSq > 1e-12 ? v * rsqrt(lenSq) : fallback;
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs vpi = GetVertexPositionInputs(input.positionOS.xyz);
                float3 normalOS = SafeNormalize3(input.normalOS, float3(0.0, 1.0, 0.0));
                output.positionCS = vpi.positionCS;
                output.positionWS = vpi.positionWS;
                output.positionOS = input.positionOS.xyz;
                output.normalOS = normalOS;
                output.restOS = input.restOS;
                output.normalWS = SafeNormalize3(TransformObjectToWorldNormal(normalOS), float3(0.0, 1.0, 0.0));
                return output;
            }

            half3 SampleLiverTriplanar(float3 positionOS, float3 normalOS)
            {
                float3 weights = abs(SafeNormalize3(normalOS, float3(0.0, 1.0, 0.0)));
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
                float3 geometricNormal = SafeNormalize3(
                    cross(ddx(input.positionWS), ddy(input.positionWS)),
                    float3(0.0, 1.0, 0.0));
                float3 vertexNormal = SafeNormalize3(input.normalWS, geometricNormal);
                if (dot(vertexNormal, geometricNormal) < -0.25)
                    vertexNormal = -vertexNormal;
                float3 normal = SafeNormalize3(vertexNormal * facing, geometricNormal);

                half3 texColor = SampleLiverTriplanar(input.restOS, input.normalOS);
                half3 minSurface = saturate(_Color.rgb * 0.35 + 0.04);
                texColor = max(texColor, minSurface);
                half3 surfaceAlbedo = max(
                    lerp(_Color.rgb, texColor, saturate(_TextureStrength)),
                    minSurface);
                // Main liver mesh renders only the original exterior surface.
                // Explicit cut faces are drawn by CutSurfaceRenderer, so backfaces
                // here must stay liver-colored instead of becoming interior patches.
                half3 albedo = surfaceAlbedo;

                Light mainLight = GetMainLight();
                float NdotL = saturate(dot(normal, mainLight.direction));
                NdotL = max(NdotL, 0.18);

                float ambientStrength = 0.50;
                float diffuseStrength = 0.50;
                float3 ambient = albedo * ambientStrength;
                float3 diffuse = albedo * mainLight.color.rgb * NdotL * diffuseStrength;

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
