Shader "SurgicalSim/LiverTissueGPU"
{
    Properties
    {
        [MainTexture] _MainTex ("Liver Albedo", 2D) = "white" {}
        [MainColor] _Color ("Tissue Base Color", Color) = (0.62, 0.18, 0.10, 1)
        _BaseColor ("URP Base Color Alias", Color) = (0.62, 0.18, 0.10, 1)
        _TextureStrength ("Texture Strength", Range(0, 1)) = 1.0
        _TextureContrast ("Texture Contrast", Range(0.25, 2.5)) = 1.35
        _TextureColorBlend ("Texture Native Color Blend", Range(0, 1)) = 1.0
        _UvTextureWeight ("Planar UV Texture Weight", Range(0, 1)) = 0.0
        _AlbedoBrightness ("Albedo Brightness", Range(0.2, 1.5)) = 0.82
        _TriplanarScale ("Triplanar Scale", Float) = 5.5
        _TriplanarBlend ("Triplanar Blend", Range(0.1, 16)) = 5.0

        _NormalMap ("Normal Map", 2D) = "bump" {}
        _HeightMap ("Sofa Liver Height Detail", 2D) = "gray" {}
        _SpecMap ("Sofa Liver Specular Detail", 2D) = "black" {}
        _NormalStrength ("Normal Strength", Range(0, 2)) = 0.12
        _ProceduralNormalStrength ("Procedural Detail Normal Strength", Range(0, 2)) = 0.05

        _Roughness ("Roughness", Range(0, 1)) = 0.70
        _Smoothness ("URP Smoothness Alias", Range(0, 1)) = 0.24
        _SpecularStrength ("Wet Specular Strength", Range(0, 2)) = 0.18
        _SpecularColor ("Specular Tint", Color) = (0.9, 0.82, 0.72, 1)
        _Wetness ("Wetness", Range(0, 1)) = 0.18

        _SSSColor ("SSS Color", Color) = (0.76, 0.13, 0.055, 1)
        _SSSStrength ("SSS Strength", Range(0, 2)) = 0.06
        _SSSDirect ("Backlight Strength", Range(0, 2)) = 0.04
        _SSSPower ("Backlight Focus", Range(1, 24)) = 7.0
        _SSSWrap ("Diffuse Wrap", Range(0, 1)) = 0.32

        _FresnelStrength ("Wet Rim Strength", Range(0, 2)) = 0.08
        _FresnelPow ("Wet Rim Falloff", Range(1, 8)) = 4.2
        _MicroMottleStrength ("Micro Mottle Strength", Range(0, 1)) = 0.16
        _MicroMottleScale ("Micro Mottle Scale", Float) = 20.0
        _VeinStrength ("Dark Vein Strength", Range(0, 1)) = 0.10
        _VeinScale ("Dark Vein Scale", Float) = 22.0
        _DebugTextureOnly ("Debug Texture Only", Range(0, 1)) = 0.0
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Opaque"
            "RenderPipeline" = "UniversalPipeline"
            "Queue" = "Geometry"
        }
        LOD 300

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

            TEXTURE2D(_MainTex);   SAMPLER(sampler_MainTex);
            TEXTURE2D(_NormalMap); SAMPLER(sampler_NormalMap);
            TEXTURE2D(_HeightMap); SAMPLER(sampler_HeightMap);
            TEXTURE2D(_SpecMap);   SAMPLER(sampler_SpecMap);

            CBUFFER_START(UnityPerMaterial)
                half4 _Color;
                half4 _BaseColor;
                half4 _SpecularColor;
                half4 _SSSColor;
                float4 _MainTex_ST;
                float4 _NormalMap_ST;
                float4 _HeightMap_ST;
                float4 _SpecMap_ST;
                half _TextureStrength;
                half _TextureContrast;
                half _TextureColorBlend;
                half _UvTextureWeight;
                half _AlbedoBrightness;
                float _TriplanarScale;
                float _TriplanarBlend;
                half _NormalStrength;
                half _ProceduralNormalStrength;
                half _Roughness;
                half _Smoothness;
                half _SpecularStrength;
                half _Wetness;
                half _SSSStrength;
                half _SSSDirect;
                half _SSSPower;
                half _SSSWrap;
                half _FresnelStrength;
                half _FresnelPow;
                half _MicroMottleStrength;
                float _MicroMottleScale;
                half _VeinStrength;
                float _VeinScale;
                half _DebugTextureOnly;
            CBUFFER_END

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float2 uv : TEXCOORD0;
                float3 restOS : TEXCOORD1;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float3 normalOS : TEXCOORD2;
                float3 restOS : TEXCOORD3;
                float2 uv : TEXCOORD4;
            };

            float3 SafeNorm(float3 v)
            {
                float lenSq = dot(v, v);
                return lenSq > 1e-12 ? v * rsqrt(lenSq) : float3(0.0, 1.0, 0.0);
            }

            float2 ApplyST(float2 uv, float4 st)
            {
                return uv * st.xy + st.zw;
            }

            float3 TriplanarWeights(float3 nOS)
            {
                float3 w = pow(max(abs(nOS), 1e-4), max(_TriplanarBlend, 1e-3));
                return w / max(w.x + w.y + w.z, 1e-5);
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
                float nxy0 = lerp(nx00, nx10, u.y);
                float nxy1 = lerp(nx01, nx11, u.y);
                return lerp(nxy0, nxy1, u.z);
            }

            half3 SampleTriplanarAlbedo(float3 restOS, float3 nOS)
            {
                float3 w = TriplanarWeights(nOS);
                float s = max(_TriplanarScale, 1e-5);
                half3 cx = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, ApplyST(restOS.zy * s, _MainTex_ST)).rgb;
                half3 cy = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, ApplyST(restOS.xz * s, _MainTex_ST)).rgb;
                half3 cz = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, ApplyST(restOS.xy * s, _MainTex_ST)).rgb;
                half3 tex = cx * w.x + cy * w.y + cz * w.z;
                return saturate((tex - 0.5h) * _TextureContrast + 0.5h);
            }

            half3 SampleUvAlbedo(float2 uv)
            {
                half3 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, ApplyST(uv, _MainTex_ST)).rgb;
                return saturate((tex - 0.5h) * _TextureContrast + 0.5h);
            }

            half SampleHeightDetail(float2 uv)
            {
                return SAMPLE_TEXTURE2D(_HeightMap, sampler_HeightMap, ApplyST(uv, _HeightMap_ST)).r;
            }

            half SampleTriplanarHeight(float3 restOS, float3 nOS)
            {
                float3 w = TriplanarWeights(nOS);
                float s = max(_TriplanarScale, 1e-5);
                half hx = SAMPLE_TEXTURE2D(_HeightMap, sampler_HeightMap, ApplyST(restOS.zy * s, _HeightMap_ST)).r;
                half hy = SAMPLE_TEXTURE2D(_HeightMap, sampler_HeightMap, ApplyST(restOS.xz * s, _HeightMap_ST)).r;
                half hz = SAMPLE_TEXTURE2D(_HeightMap, sampler_HeightMap, ApplyST(restOS.xy * s, _HeightMap_ST)).r;
                return hx * w.x + hy * w.y + hz * w.z;
            }

            half SampleSpecDetail(float2 uv)
            {
                return SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, ApplyST(uv, _SpecMap_ST)).r;
            }

            half SampleTriplanarSpec(float3 restOS, float3 nOS)
            {
                float3 w = TriplanarWeights(nOS);
                float s = max(_TriplanarScale, 1e-5);
                half sx = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, ApplyST(restOS.zy * s, _SpecMap_ST)).r;
                half sy = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, ApplyST(restOS.xz * s, _SpecMap_ST)).r;
                half sz = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, ApplyST(restOS.xy * s, _SpecMap_ST)).r;
                return sx * w.x + sy * w.y + sz * w.z;
            }

            float3 SampleTriplanarNormal(float3 restOS, float3 nOS)
            {
                float s = max(_TriplanarScale, 1e-5);
                float eps = 0.004 / s;
                half hx = SampleTriplanarHeight(restOS + float3(eps, 0.0, 0.0), nOS)
                        - SampleTriplanarHeight(restOS - float3(eps, 0.0, 0.0), nOS);
                half hy = SampleTriplanarHeight(restOS + float3(0.0, eps, 0.0), nOS)
                        - SampleTriplanarHeight(restOS - float3(0.0, eps, 0.0), nOS);
                half hz = SampleTriplanarHeight(restOS + float3(0.0, 0.0, eps), nOS)
                        - SampleTriplanarHeight(restOS - float3(0.0, 0.0, eps), nOS);
                return SafeNorm(nOS - float3(hx, hy, hz) * _NormalStrength * 2.5);
            }

            float3 ProceduralFineNormal(float3 restOS, float3 nOS)
            {
                float scale = max(_MicroMottleScale, 0.001);
                float3 p = restOS * scale;
                float eps = 0.075;
                float h = Noise3(p);
                float hx = Noise3(p + float3(eps, 0.0, 0.0)) - h;
                float hy = Noise3(p + float3(0.0, eps, 0.0)) - h;
                float hz = Noise3(p + float3(0.0, 0.0, eps)) - h;
                return SafeNorm(nOS + float3(hx, hy, hz) * _ProceduralNormalStrength * 2.0);
            }

            half3 TextureDrivenAlbedo(half3 baseColor, half3 texColor, float3 restOS)
            {
                half3 tissueTint = half3(1.04h, 0.68h, 0.52h);
                half3 nativeTexture = saturate(texColor * tissueTint + baseColor * 0.08h);
                half texLum = dot(nativeTexture, half3(0.299h, 0.587h, 0.114h));
                half3 shadowedTexture = nativeTexture * 0.52h + half3(0.065h, 0.006h, 0.003h);
                half3 brightTexture = nativeTexture * 1.12h + half3(0.045h, 0.010h, 0.004h);
                half3 stainedTexture = lerp(shadowedTexture, brightTexture, smoothstep(0.10h, 0.70h, texLum));
                stainedTexture = lerp(stainedTexture, nativeTexture, saturate(_TextureColorBlend));

                half veinA = Noise3(restOS * max(_VeinScale, 0.001) + 13.7);
                half veinB = Noise3(restOS * max(_VeinScale * 2.17, 0.001) + 41.3);
                half veinMask = smoothstep(0.58h, 0.88h, veinA * 0.68h + veinB * 0.32h);
                half3 darkVein = stainedTexture * 0.42h + half3(0.055h, 0.001h, 0.001h);
                stainedTexture = lerp(stainedTexture, darkVein, veinMask * _VeinStrength);

                return lerp(baseColor, stainedTexture, saturate(_TextureStrength));
            }

            half3 Liver_FresnelSchlick(float cosTheta, half3 f0)
            {
                return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
            }

            float Liver_D_GGX(float ndh, float a2)
            {
                float d = ndh * ndh * (a2 - 1.0) + 1.0;
                return a2 / max(PI * d * d, 1e-6);
            }

            float Liver_G_Schlick(float ndx, float k)
            {
                return ndx / max(ndx * (1.0 - k) + k, 1e-6);
            }

            half3 Liver_GGXSpecular(float3 n, float3 l, float3 v, half3 f0, float roughness)
            {
                float3 h = SafeNorm(l + v);
                float ndl = saturate(dot(n, l));
                float ndv = max(dot(n, v), 1e-4);
                float ndh = saturate(dot(n, h));
                float vdh = saturate(dot(v, h));
                float a = max(roughness * roughness, 0.045);
                float a2 = a * a;
                float k = (roughness + 1.0);
                k = k * k * 0.125;
                half3 f = Liver_FresnelSchlick(vdh, f0);
                return (Liver_D_GGX(ndh, a2) * Liver_G_Schlick(ndl, k) * Liver_G_Schlick(ndv, k) * f) / max(4.0 * ndl * ndv, 1e-5);
            }

            half3 TissueSSS(float3 n, float3 l, float3 v, half3 albedo)
            {
                float wrap = saturate(_SSSWrap);
                float wrapped = saturate((dot(n, l) + wrap) / max(1.0 + wrap, 1e-4));
                float back = pow(saturate(dot(v, -l)), _SSSPower);
                half3 scatterTint = lerp(albedo, _SSSColor.rgb, 0.45);
                return scatterTint * (wrapped * _SSSStrength + back * _SSSDirect);
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs pos = GetVertexPositionInputs(input.positionOS.xyz);
                output.positionCS = pos.positionCS;
                output.positionWS = pos.positionWS;
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                output.normalOS = SafeNorm(input.normalOS);
                output.restOS = input.restOS;
                output.uv = input.uv;
                return output;
            }

            half4 frag(Varyings input, half facing : VFACE) : SV_Target
            {
                float side = facing > 0.0 ? 1.0 : -1.0;
                float3 baseOS = SafeNorm(input.normalOS * side);
                float3 baseWS = SafeNorm(input.normalWS * side);
                float3 mappedOS = SampleTriplanarNormal(input.restOS, baseOS);
                float3 proceduralOS = ProceduralFineNormal(input.restOS, baseOS);
                mappedOS = SafeNorm(lerp(mappedOS, proceduralOS, saturate(_ProceduralNormalStrength)));
                float3 mappedWS = SafeNorm(TransformObjectToWorldNormal(mappedOS));
                float3 n = SafeNorm(lerp(baseWS, mappedWS, saturate(_NormalStrength)));
                float3 v = SafeNorm(GetWorldSpaceViewDir(input.positionWS));

                half3 triTex = SampleTriplanarAlbedo(input.restOS, baseOS);
                half3 uvTex = SampleUvAlbedo(input.uv);
                half uvWeight = saturate(_UvTextureWeight);
                half3 texColor = lerp(triTex, uvTex, uvWeight);
                half heightDetail = lerp(SampleTriplanarHeight(input.restOS, baseOS), SampleHeightDetail(input.uv), uvWeight);
                half specDetail = lerp(SampleTriplanarSpec(input.restOS, baseOS), SampleSpecDetail(input.uv), uvWeight);
                half3 baseColor = _Color.rgb;
                half mottle = Noise3(input.restOS * max(_MicroMottleScale, 0.001));
                half fine = Noise3(input.restOS * max(_MicroMottleScale * 2.37, 0.001) + 17.0);
                half mottleMix = saturate(mottle * 0.65 + fine * 0.35);
                half heightGain = lerp(0.88h, 1.10h, heightDetail);
                half mottleGain = lerp(1.0 - _MicroMottleStrength, 1.0 + _MicroMottleStrength, mottleMix) * heightGain;
                half3 albedo = saturate(TextureDrivenAlbedo(baseColor, texColor, input.restOS) * _AlbedoBrightness * mottleGain);

                #if defined(_MAIN_LIGHT_SHADOWS) || defined(_MAIN_LIGHT_SHADOWS_CASCADE) || defined(_MAIN_LIGHT_SHADOWS_SCREEN)
                    Light mainLight = GetMainLight(TransformWorldToShadowCoord(input.positionWS));
                #else
                    Light mainLight = GetMainLight();
                #endif

                float3 l = SafeNorm(mainLight.direction);
                half3 lightColor = mainLight.color;
                half shadow = mainLight.shadowAttenuation;
                float ndl = saturate(dot(n, l));
                float wrap = saturate(_SSSWrap);
                float wrapped = saturate((dot(n, l) + wrap) / max(1.0 + wrap, 1e-4));

                half roughness = saturate(lerp(_Roughness, max(0.16h, _Roughness * 0.72h), _Wetness * specDetail));
                half3 f0 = lerp(half3(0.028h, 0.028h, 0.028h), _SpecularColor.rgb * 0.08h, _Wetness);
                half3 diffuse = albedo * lightColor * wrapped * shadow;
                half specMask = lerp(0.45h, 1.15h, specDetail);
                half3 specular = Liver_GGXSpecular(n, l, v, f0, roughness) * lightColor * ndl * shadow * _SpecularStrength * specMask;
                half3 sss = TissueSSS(n, l, v, albedo) * lightColor * shadow;
                half rim = pow(1.0 - saturate(dot(n, v)), _FresnelPow);
                half3 fresnel = rim * _SpecularColor.rgb * _FresnelStrength * _Wetness * lightColor;
                half3 ambient = SampleSH(n) * albedo * 0.65h;

                #ifdef _ADDITIONAL_LIGHTS
                uint count = GetAdditionalLightsCount();
                for (uint i = 0u; i < count; ++i)
                {
                    Light addLight = GetAdditionalLight(i, input.positionWS, half4(1, 1, 1, 1));
                    float3 al = SafeNorm(addLight.direction);
                    half3 ac = addLight.color * addLight.distanceAttenuation * addLight.shadowAttenuation;
                    float andl = saturate(dot(n, al));
                    float awrap = saturate((dot(n, al) + wrap) / max(1.0 + wrap, 1e-4));
                    diffuse += albedo * ac * awrap;
                    specular += Liver_GGXSpecular(n, al, v, f0, roughness) * ac * andl * _SpecularStrength * specMask;
                    sss += TissueSSS(n, al, v, albedo) * ac * 0.35h;
                }
                #endif

                half3 litColor = ambient + diffuse + specular + sss + fresnel;
                return half4(lerp(litColor, texColor, saturate(_DebugTextureOnly)), 1.0);
            }
            ENDHLSL
        }

        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode" = "ShadowCaster" }
            Cull Off
            ZWrite On
            ZTest LEqual
            ColorMask 0

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_shadowcaster
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            float3 _LightDirection;

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
            };

            Varyings vert(Attributes input)
            {
                Varyings output;
                float3 posWS = TransformObjectToWorld(input.positionOS.xyz);
                float3 normalWS = TransformObjectToWorldNormal(input.normalOS);
                posWS = ApplyShadowBias(posWS, normalWS, _LightDirection);
                output.positionCS = TransformWorldToHClip(posWS);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                return 0;
            }
            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }
            Cull Off
            ZWrite On
            ColorMask R

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes { float4 positionOS : POSITION; };
            struct Varyings { float4 positionCS : SV_POSITION; };

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                return 0;
            }
            ENDHLSL
        }
    }

    FallBack "Universal Render Pipeline/Lit"
}
