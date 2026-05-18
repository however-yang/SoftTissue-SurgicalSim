Shader "SurgicalSim/LiverCutSurfaceGPU"
{
    Properties
    {
        [MainColor] _CutColor ("Cut Tissue Color", Color) = (0.30, 0.025, 0.018, 1)
        _Color ("Color Alias", Color) = (0.30, 0.025, 0.018, 1)
        _BaseColor ("URP Base Color Alias", Color) = (0.30, 0.025, 0.018, 1)
        _InteriorColor ("Interior Color Alias", Color) = (0.30, 0.025, 0.018, 1)
        _Roughness ("Roughness", Range(0, 1)) = 0.34
        _Smoothness ("Smoothness Alias", Range(0, 1)) = 0.66
        _Wetness ("Wetness", Range(0, 1)) = 0.78
        _SpecularStrength ("Wet Specular Strength", Range(0, 2)) = 0.85
        _SpecularColor ("Specular Tint", Color) = (0.9, 0.76, 0.65, 1)
        _FiberIntensity ("Fiber Intensity", Range(0, 1)) = 0.34
        _FiberScale ("Fiber Scale", Float) = 36.0
        _SSSColor ("SSS Color", Color) = (0.55, 0.08, 0.035, 1)
        _SSSStrength ("SSS Strength", Range(0, 2)) = 0.24
        _SSSWrap ("Diffuse Wrap", Range(0, 1)) = 0.24
        _RimDarkening ("Rim Darkening", Range(0, 1)) = 0.18
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Opaque"
            "RenderPipeline" = "UniversalPipeline"
            "Queue" = "Geometry"
        }
        LOD 250

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }
            Cull Off
            ZWrite On

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
            #pragma multi_compile _ _SHADOWS_SOFT

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _CutColor;
                half4 _Color;
                half4 _BaseColor;
                half4 _InteriorColor;
                half4 _SpecularColor;
                half4 _SSSColor;
                half _Roughness;
                half _Smoothness;
                half _Wetness;
                half _SpecularStrength;
                half _FiberIntensity;
                float _FiberScale;
                half _SSSStrength;
                half _SSSWrap;
                half _RimDarkening;
            CBUFFER_END

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 positionOS : TEXCOORD1;
                float3 normalWS : TEXCOORD2;
            };

            float3 SafeNorm(float3 v)
            {
                float lenSq = dot(v, v);
                return lenSq > 1e-12 ? v * rsqrt(lenSq) : float3(0.0, 1.0, 0.0);
            }

            float Hash31(float3 p)
            {
                p = frac(p * 0.1031);
                p += dot(p, p.yzx + 33.33);
                return frac((p.x + p.y) * p.z);
            }

            float Noise3(float3 p)
            {
                float3 i = floor(p);
                float3 f = frac(p);
                float3 u = f * f * (3.0 - 2.0 * f);
                float n000 = Hash31(i + float3(0, 0, 0));
                float n100 = Hash31(i + float3(1, 0, 0));
                float n010 = Hash31(i + float3(0, 1, 0));
                float n110 = Hash31(i + float3(1, 1, 0));
                float n001 = Hash31(i + float3(0, 0, 1));
                float n101 = Hash31(i + float3(1, 0, 1));
                float n011 = Hash31(i + float3(0, 1, 1));
                float n111 = Hash31(i + float3(1, 1, 1));
                float nx00 = lerp(n000, n100, u.x);
                float nx10 = lerp(n010, n110, u.x);
                float nx01 = lerp(n001, n101, u.x);
                float nx11 = lerp(n011, n111, u.x);
                return lerp(lerp(nx00, nx10, u.y), lerp(nx01, nx11, u.y), u.z);
            }

            half3 Cut_FresnelSchlick(float cosTheta, half3 f0)
            {
                return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
            }

            float Cut_D_GGX(float ndh, float a2)
            {
                float d = ndh * ndh * (a2 - 1.0) + 1.0;
                return a2 / max(PI * d * d, 1e-6);
            }

            float Cut_G_Schlick(float ndx, float k)
            {
                return ndx / max(ndx * (1.0 - k) + k, 1e-6);
            }

            half3 Cut_GGXSpecular(float3 n, float3 l, float3 v, half3 f0, float roughness)
            {
                float3 h = SafeNorm(l + v);
                float ndl = saturate(dot(n, l));
                float ndv = max(dot(n, v), 1e-4);
                float ndh = saturate(dot(n, h));
                float vdh = saturate(dot(v, h));
                float a = max(roughness * roughness, 0.04);
                float a2 = a * a;
                float k = (roughness + 1.0);
                k = k * k * 0.125;
                return (Cut_D_GGX(ndh, a2) * Cut_G_Schlick(ndl, k) * Cut_G_Schlick(ndv, k) * Cut_FresnelSchlick(vdh, f0)) / max(4.0 * ndl * ndv, 1e-5);
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs pos = GetVertexPositionInputs(input.positionOS.xyz);
                output.positionCS = pos.positionCS;
                output.positionWS = pos.positionWS;
                output.positionOS = input.positionOS.xyz;
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                return output;
            }

            half4 frag(Varyings input, half facing : VFACE) : SV_Target
            {
                float side = facing > 0.0 ? 1.0 : -1.0;
                float3 n = SafeNorm(input.normalWS * side);
                float3 v = SafeNorm(GetWorldSpaceViewDir(input.positionWS));
                float scale = max(_FiberScale, 0.001);
                float noiseA = Noise3(input.positionOS * scale);
                float noiseB = Noise3(input.positionOS * scale * 2.13 + 9.7);
                float fiber = sin((input.positionOS.x * 19.0 + input.positionOS.z * 11.0 + noiseA * 7.0) * scale * 0.08) * 0.5 + 0.5;
                float tissuePattern = saturate(noiseA * 0.45 + noiseB * 0.25 + fiber * 0.30);
                half fiberGain = lerp(1.0 - _FiberIntensity, 1.0 + _FiberIntensity, tissuePattern);

                half3 baseColor = _CutColor.rgb;
                half3 darkBlood = half3(0.12h, 0.008h, 0.006h);
                half3 albedo = lerp(darkBlood, baseColor, 0.72h) * fiberGain;
                albedo = saturate(albedo);

                #if defined(_MAIN_LIGHT_SHADOWS) || defined(_MAIN_LIGHT_SHADOWS_CASCADE) || defined(_MAIN_LIGHT_SHADOWS_SCREEN)
                    Light mainLight = GetMainLight(TransformWorldToShadowCoord(input.positionWS));
                #else
                    Light mainLight = GetMainLight();
                #endif

                float3 l = SafeNorm(mainLight.direction);
                half3 lightColor = mainLight.color;
                half shadow = mainLight.shadowAttenuation;
                float ndl = saturate(dot(n, l));
                half wrap = saturate(_SSSWrap);
                half wrapped = saturate((dot(n, l) + wrap) / max(1.0h + wrap, 1e-4h));
                half roughness = saturate(lerp(_Roughness, max(0.08h, _Roughness * 0.55h), _Wetness));
                half3 f0 = lerp(half3(0.03h, 0.03h, 0.03h), _SpecularColor.rgb * 0.09h, _Wetness);

                half rim = pow(1.0 - saturate(dot(n, v)), 2.5);
                albedo *= lerp(1.0h, 1.0h - _RimDarkening, rim);

                half3 diffuse = albedo * lightColor * wrapped * shadow;
                half3 specular = Cut_GGXSpecular(n, l, v, f0, roughness) * lightColor * ndl * shadow * _SpecularStrength;
                half3 sss = lerp(albedo, _SSSColor.rgb, 0.45h) * wrapped * _SSSStrength * lightColor * shadow;
                half3 ambient = SampleSH(n) * albedo * 0.45h;

                #ifdef _ADDITIONAL_LIGHTS
                uint count = GetAdditionalLightsCount();
                for (uint i = 0u; i < count; ++i)
                {
                    Light addLight = GetAdditionalLight(i, input.positionWS, half4(1, 1, 1, 1));
                    float3 al = SafeNorm(addLight.direction);
                    half3 ac = addLight.color * addLight.distanceAttenuation * addLight.shadowAttenuation;
                    float andl = saturate(dot(n, al));
                    half awrap = saturate((dot(n, al) + wrap) / max(1.0h + wrap, 1e-4h));
                    diffuse += albedo * ac * awrap;
                    specular += Cut_GGXSpecular(n, al, v, f0, roughness) * ac * andl * _SpecularStrength;
                }
                #endif

                return half4(ambient + diffuse + specular + sss, 1.0);
            }
            ENDHLSL
        }
    }

    FallBack "Universal Render Pipeline/Lit"
}
