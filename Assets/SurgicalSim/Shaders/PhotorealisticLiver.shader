// PhotorealisticLiver.shader — 真实感肝脏着色器 (URP 12.x 兼容)
//
// 核心算法:
//   1. Triplanar Albedo + Normal Map (Ben Golus Whiteout Blend)
//   2. Liver_GGX Cook-Torrance 微表面高光 (避免与 URP BRDF.hlsl 冲突)
//   3. Wrapped Diffuse + 背光 SSS 次表面散射近似
//   4. Schlick Fresnel 边缘湿润光
//   5. URP SH 环境光 + Additional Lights
//
// 修复:
//   - 自定义函数加 Liver_ 前缀, 避免与 URP BRDF.hlsl 的 D_GGX 等冲突
//   - ShadowCaster 不再 include Shadows.hlsl (接收侧库), 避免 LerpWhiteTo 错误
//   - 使用 ApplyShadowBias() 正确处理阴影偏移
//
// 参考:
//   - GPU Gems Ch.16: "Real-Time Approximations to SSS" (Jensen et al.)
//   - Ben Golus: "Triplanar Normal Mapping" blog
//   - SofaUnity: 利用 Unity PBR + 高质量法线贴图实现手术仿真渲染质量

Shader "SurgicalSim/PhotorealisticLiver"
{
    Properties
    {
        // ── 基础颜色 (兼容 SoftBody.cs SetColor 调用) ─────────────
        [Header(Base Appearance)]
        _MainTex        ("Liver Albedo Texture",  2D)           = "white" {}
        _Color          ("Surface Base Color",    Color)         = (0.42, 0.045, 0.030, 1)
        _InteriorColor  ("Cut Interior Color",    Color)         = (0.24, 0.018, 0.014, 1)
        _TextureStrength("Texture Blend",         Range(0,1))    = 0.55
        _TriplanarScale ("Triplanar Scale",        Float)         = 6.0
        _TriplanarBlend ("Triplanar Blend Power",  Range(0.1,16)) = 5.0

        // ── 法线贴图 ────────────────────────────────────────────────
        [Header(Normal Map)]
        _NormalMap      ("Normal Map (Bump)",  2D)          = "bump" {}
        _NormalStrength ("Normal Strength",    Range(0,3))  = 0.8

        // ── 高光/粗糙度贴图 ──────────────────────────────────────────
        [Header(Specular Map)]
        _SpecMap        ("Specular Map (liver2_spec)", 2D)  = "gray" {}
        _SpecMapStrength("Spec Map Strength",  Range(0,2))  = 0.8

        // ── PBR 高光 ────────────────────────────────────────────────
        [Header(Surface PBR)]
        _Roughness       ("Roughness",          Range(0,1)) = 0.40
        _SpecularStrength("Specular Strength",  Range(0,3)) = 0.8
        _SpecularColor   ("Specular Tint",      Color)      = (0.95, 0.90, 0.85, 1)

        // ── 次表面散射 SSS ──────────────────────────────────────────
        [Header(Subsurface Scattering SSS)]
        _SSSColor    ("SSS Scatter Color",   Color)      = (0.80, 0.15, 0.05, 1)
        _SSSStrength ("SSS Wrap Strength",   Range(0,3)) = 0.35
        _SSSDirect   ("Backlight Strength",  Range(0,2)) = 0.30
        _SSSPower    ("Backlight Focus",     Range(1,24)) = 6.0
        _SSSWrap     ("Diffuse Wrap Factor", Range(0,1))  = 0.35

        // ── Fresnel 边缘 ────────────────────────────────────────────
        [Header(Fresnel Rim Wetness)]
        _FresnelStrength("Fresnel Strength", Range(0,3)) = 0.20
        _FresnelPow     ("Fresnel Falloff",  Range(1,8)) = 4.0
    }

    SubShader
    {
        Tags
        {
            "RenderType"="Opaque"
            "RenderPipeline"="UniversalPipeline"
            "Queue"="Geometry"
        }
        LOD 300

        // ════════════════════════════════════════════════════════════
        // Pass 1: Forward Lit
        // ════════════════════════════════════════════════════════════
        Pass
        {
            Name "FORWARD"
            Tags { "LightMode"="UniversalForward" }
            Cull Off   // 双面 — 切割内表面可见

            HLSLPROGRAM
            #pragma vertex   vert
            #pragma fragment frag

            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
            #pragma multi_compile _ _SHADOWS_SOFT
            #pragma multi_compile _ LIGHTMAP_ON DIRLIGHTMAP_COMBINED

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            // ── Textures ──────────────────────────────────────────────
            TEXTURE2D(_MainTex);   SAMPLER(sampler_MainTex);
            TEXTURE2D(_NormalMap); SAMPLER(sampler_NormalMap);
            TEXTURE2D(_SpecMap);   SAMPLER(sampler_SpecMap);

            // ── CBUFFER (SRP Batcher 兼容) ───────────────────────────
            CBUFFER_START(UnityPerMaterial)
                half4  _Color;
                half4  _InteriorColor;
                half4  _SpecularColor;
                half4  _SSSColor;
                float4 _MainTex_ST;
                float4 _NormalMap_ST;
                float4 _SpecMap_ST;
                half   _TextureStrength;
                float  _TriplanarScale;
                float  _TriplanarBlend;
                half   _NormalStrength;
                half   _Roughness;
                half   _SpecularStrength;
                half   _SpecMapStrength;
                half   _SSSStrength;
                half   _SSSDirect;
                half   _SSSPower;
                half   _SSSWrap;
                half   _FresnelStrength;
                half   _FresnelPow;
            CBUFFER_END

            // ── Structs ───────────────────────────────────────────────
            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS   : NORMAL;
                float4 tangentOS  : TANGENT;
                float3 restOS     : TEXCOORD1; // 静止位置 (SoftBody 写入 UV1)
            };

            struct Varyings
            {
                float4 positionCS  : SV_POSITION;
                float3 positionWS  : TEXCOORD0;
                float3 normalWS    : TEXCOORD1;
                float3 positionOS  : TEXCOORD2;
                float3 normalOS    : TEXCOORD3;
                float3 restOS      : TEXCOORD4;
            };

            // ── 工具函数 ──────────────────────────────────────────────
            float3 Liver_SafeNorm(float3 v)
            {
                float l = dot(v, v);
                return l > 1e-12 ? v * rsqrt(l) : float3(0, 1, 0);
            }

            // ── Triplanar Albedo ──────────────────────────────────────
            // 修复: 正确使用 _MainTex_ST 的 tiling (.xy) 和 offset (.zw)
            half3 Liver_TriplanarAlbedo(float3 restPos, float3 normalOS)
            {
                float3 w = pow(max(abs(normalOS), 1e-4), _TriplanarBlend);
                w /= max(w.x + w.y + w.z, 1e-5);
                // _MainTex_ST.xy = tiling, _MainTex_ST.zw = offset
                float2 tiling = max(abs(_MainTex_ST.xy), float2(1e-5, 1e-5));
                float  s      = max(_TriplanarScale, 1e-5);
                float2 uvX = restPos.zy * s * tiling + _MainTex_ST.zw;
                float2 uvY = restPos.xz * s * tiling + _MainTex_ST.zw;
                float2 uvZ = restPos.xy * s * tiling + _MainTex_ST.zw;
                half3 cx = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvX).rgb;
                half3 cy = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvY).rgb;
                half3 cz = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uvZ).rgb;
                return cx * w.x + cy * w.y + cz * w.z;
            }

            // ── Triplanar SpecMap ─────────────────────────────────────
            half Liver_TriplanarSpec(float3 restPos, float3 normalOS)
            {
                float3 w = pow(max(abs(normalOS), 1e-4), _TriplanarBlend);
                w /= max(w.x + w.y + w.z, 1e-5);
                float s = max(_TriplanarScale, 1e-5);
                half sx = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, restPos.zy * s).r;
                half sy = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, restPos.xz * s).r;
                half sz = SAMPLE_TEXTURE2D(_SpecMap, sampler_SpecMap, restPos.xy * s).r;
                return sx * w.x + sy * w.y + sz * w.z;
            }

            // ── Triplanar Normal (Ben Golus Whiteout Blend) ───────────
            float3 Liver_TriplanarNormal(float3 restPos, float3 nOS, float3 nWS)
            {
                float3 w = pow(max(abs(nOS), 1e-4), _TriplanarBlend);
                w /= max(w.x + w.y + w.z, 1e-5);
                float s = max(_TriplanarScale, 1e-5);

                half3 tnX = UnpackNormalScale(
                    SAMPLE_TEXTURE2D(_NormalMap, sampler_NormalMap, restPos.zy * s), _NormalStrength);
                half3 tnY = UnpackNormalScale(
                    SAMPLE_TEXTURE2D(_NormalMap, sampler_NormalMap, restPos.xz * s), _NormalStrength);
                half3 tnZ = UnpackNormalScale(
                    SAMPLE_TEXTURE2D(_NormalMap, sampler_NormalMap, restPos.xy * s), _NormalStrength);

                // Whiteout blend: 在世界空间各面构造法线
                float3 nX = float3(tnX.xy + nWS.zy, abs(nWS.x)); nX = nX.zyx;
                float3 nY = float3(tnY.xy + nWS.xz, abs(nWS.y)); nY = nY.xzy;
                float3 nZ = float3(tnZ.xy + nWS.xy, abs(nWS.z));

                return Liver_SafeNorm(nX * w.x + nY * w.y + nZ * w.z);
            }

            // ── GGX 法线分布函数 D ────────────────────────────────────
            // 前缀 Liver_ 避免与 URP BRDF.hlsl 中的 D_GGX 冲突
            float Liver_D_GGX(float NdotH, float a2)
            {
                float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
                return a2 / max(PI * d * d, 1e-7);
            }

            // ── GGX Smith 几何函数 G ──────────────────────────────────
            float Liver_G_Schlick(float NdotX, float k)
            {
                return NdotX / max(NdotX * (1.0 - k) + k, 1e-7);
            }

            // ── Schlick Fresnel F ─────────────────────────────────────
            half3 Liver_F_Schlick(float cosTheta, half3 F0)
            {
                return F0 + (1.0 - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
            }

            // ── Cook-Torrance GGX 高光 ────────────────────────────────
            half3 Liver_GGXSpecular(float3 N, float3 L, float3 V, half3 F0, float roughness)
            {
                float3 H    = Liver_SafeNorm(L + V);
                float NdotH = saturate(dot(N, H));
                float NdotL = saturate(dot(N, L));
                float NdotV = max(dot(N, V), 1e-4);
                float HdotV = saturate(dot(H, V));

                float a  = max(roughness * roughness, 0.001);
                float a2 = a * a;
                float D  = Liver_D_GGX(NdotH, a2);
                float k  = (roughness + 1.0); k = k * k / 8.0;
                float G  = Liver_G_Schlick(NdotL, k) * Liver_G_Schlick(NdotV, k);
                half3 F  = Liver_F_Schlick(HdotV, F0);

                return (D * G * F) / max(4.0 * NdotL * NdotV, 1e-7);
            }

            // ── SSS: Wrapped Diffuse + 背光透射 ──────────────────────
            // 参考: GPU Gems Ch.16 "Real-Time Approximations to SSS"
            half3 Liver_SSS(float3 N, float3 L, float3 V)
            {
                // Wrapped diffuse: 光绕到阴影面, 模拟组织内散射
                float wrap   = _SSSWrap;
                float wDiff  = saturate((dot(N, L) + wrap) / ((1.0 + wrap) * (1.0 + wrap)));
                half3 sssWrap = wDiff * _SSSColor.rgb * _SSSStrength;

                // 背光透射: dot(V, -L) 在逆光时最大, 模拟薄组织透光
                float back  = pow(saturate(dot(V, -L)), _SSSPower);
                half3 sssBack = back * _SSSColor.rgb * _SSSDirect;

                return sssWrap + sssBack;
            }

            // ── Fresnel 边缘湿润感 ────────────────────────────────────
            half3 Liver_FresnelRim(float3 N, float3 V)
            {
                float rim = pow(1.0 - saturate(dot(N, V)), _FresnelPow);
                return rim * _SpecularColor.rgb * _FresnelStrength;
            }

            // ═════════════════════════════════════════════════════════
            // Vertex Shader
            // ═════════════════════════════════════════════════════════
            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                VertexPositionInputs vpi = GetVertexPositionInputs(IN.positionOS.xyz);
                VertexNormalInputs   vni = GetVertexNormalInputs(IN.normalOS, IN.tangentOS);

                OUT.positionCS = vpi.positionCS;
                OUT.positionWS = vpi.positionWS;
                OUT.normalWS   = vni.normalWS;
                OUT.positionOS = IN.positionOS.xyz;
                OUT.normalOS   = Liver_SafeNorm(IN.normalOS);
                OUT.restOS     = IN.restOS;
                return OUT;
            }

            // ═════════════════════════════════════════════════════════
            // Fragment Shader
            // ═════════════════════════════════════════════════════════
            half4 frag(Varyings IN, half facing : VFACE) : SV_Target
            {
                // ── 几何法线 ────────────────────────────────────────────
                float3 geoN  = Liver_SafeNorm(cross(ddx(IN.positionWS), ddy(IN.positionWS)));
                float3 vertN = Liver_SafeNorm(IN.normalWS);
                if (dot(vertN, geoN) < -0.25) vertN = -vertN;
                float3 baseNWS = vertN * (facing > 0 ? 1.0 : -1.0);

                // ── Triplanar 法线 ───────────────────────────────────────
                float3 N = Liver_TriplanarNormal(IN.restOS, IN.normalOS, baseNWS);
                N = Liver_SafeNorm(lerp(baseNWS, N, saturate(_NormalStrength)));

                // ── Albedo 正确合成 ──────────────────────────────────────
                // 修复: 去掉 * 1.8 boost (导致纹理被冲白)
                // 正确做法: 纹理在生物色相范围内调制 base color
                half3 texColor   = Liver_TriplanarAlbedo(IN.restOS, IN.normalOS);
                half3 baseColor  = _Color.rgb;   // 深红棕, 约(0.42, 0.045, 0.03)

                // texColor 通常偏亮/偏暖, 用它调制 baseColor 以保留生物色相
                // lerp: TextureStrength=0 时纯 baseColor, =1 时 texColor * baseColor
                half3 tinted = texColor * baseColor * 3.5; // 3.5 把暗纹理提亮到可见范围
                tinted = saturate(tinted);                 // 钳位防止溢出
                half3 albedo = lerp(baseColor, tinted, saturate(_TextureStrength));
                // 确保不低于 base color 的 30% (避免全黑)
                albedo = max(albedo, baseColor * 0.30);

                // plan 规定: 主 mesh 不用 VFACE 决定 Interior
                // VFACE 只用于法线翻转 (已在上方处理)
                // 切面颜色由 CutSurfaceRenderer 单独渲染

                // ── 主光源 ───────────────────────────────────────────────
                #if defined(_MAIN_LIGHT_SHADOWS) || defined(_MAIN_LIGHT_SHADOWS_CASCADE) || defined(_MAIN_LIGHT_SHADOWS_SCREEN)
                    float4 shadowCoord = TransformWorldToShadowCoord(IN.positionWS);
                    Light  mainLight   = GetMainLight(shadowCoord);
                #else
                    Light mainLight = GetMainLight();
                #endif

                float3 L      = Liver_SafeNorm(mainLight.direction);
                float3 V      = Liver_SafeNorm(GetWorldSpaceViewDir(IN.positionWS));
                float  shadow = mainLight.shadowAttenuation;
                half3  lightC = mainLight.color;

                // ── Wrapped Diffuse ──────────────────────────────────────
                float wrap    = _SSSWrap;
                float wDiff   = saturate((dot(N, L) + wrap) / ((1.0 + wrap) * (1.0 + wrap)));
                half3 diffuse = albedo * lightC * wDiff * shadow;

                // ── GGX Specular (加入 SpecMap 调制粗糙度) ────────────────
                // liver2_spec.png 的高值区域 = 更湿润/更光滑
                half specFromMap = Liver_TriplanarSpec(IN.restOS, IN.normalOS);
                // specMap 高 → roughness 低 (更光滑)
                float roughness = max(
                    lerp(_Roughness, _Roughness * (1.0 - specFromMap * _SpecMapStrength), 1.0),
                    0.04);
                half3 F0      = lerp(half3(0.04, 0.04, 0.04), _SpecularColor.rgb, 0.35);
                float NdotL   = saturate(dot(N, L));
                // specMap 高 → specular 强 (更湿润高光)
                half  specBoost = 1.0 + specFromMap * _SpecMapStrength;
                half3 specular  = Liver_GGXSpecular(N, L, V, F0, roughness)
                                * lightC * shadow * NdotL * _SpecularStrength * specBoost;

                // ── SSS ──────────────────────────────────────────────────
                half3 sss = Liver_SSS(N, L, V) * lightC * shadow;

                // ── Fresnel 边缘 ─────────────────────────────────────────
                half3 fresnel = Liver_FresnelRim(N, V) * lightC;

                // ── 环境光 (URP Spherical Harmonics) ─────────────────────
                half3 ambient = SampleSH(N) * albedo * 0.8;

                // ── 附加光源 ─────────────────────────────────────────────
                // 注: URP 12.x LIGHT_LOOP_BEGIN 宏内部变量为 lightIndex 而非 i,
                //     直接写 for 循环避免歧义
                #ifdef _ADDITIONAL_LIGHTS
                {
                    uint addCount = GetAdditionalLightsCount();
                    for (uint li = 0u; li < addCount; ++li)
                    {
                        Light addL  = GetAdditionalLight(li, IN.positionWS, half4(1,1,1,1));
                        float3 aL   = Liver_SafeNorm(addL.direction);
                        float  aNdL = saturate(dot(N, aL));
                        float  aAtt = addL.distanceAttenuation * addL.shadowAttenuation;
                        half3  aC   = addL.color * aAtt;
                        float  aWrap = saturate((dot(N,aL)+wrap)/((1.0+wrap)*(1.0+wrap)));

                        diffuse  += albedo * aC * aWrap;
                        specular += Liver_GGXSpecular(N,aL,V,F0,roughness) * aC * aNdL * _SpecularStrength;
                        sss      += Liver_SSS(N, aL, V) * aC * 0.5;
                    }
                }
                #endif

                // ── 合成 ─────────────────────────────────────────────────
                return half4(ambient + diffuse + specular + sss + fresnel, 1.0);
            }
            ENDHLSL
        }

        // ════════════════════════════════════════════════════════════
        // Pass 2: Shadow Caster
        //
        // 修复说明 (URP 12.x):
        //   直接 #include Shadows.hlsl 会报 LerpWhiteTo 未声明，因为
        //   Shadows.hlsl 内部依赖 LerpWhiteTo (定义在 CommonMaterial.hlsl)，
        //   该函数必须由 Lighting.hlsl 的完整 include 链来建立。
        //   改用 Lighting.hlsl 即可同时获得:
        //     - LerpWhiteTo (via Core → CommonMaterial.hlsl)
        //     - ApplyShadowBias (via Shadows.hlsl)
        // ════════════════════════════════════════════════════════════
        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode"="ShadowCaster" }
            Cull Off
            ZWrite On
            ZTest LEqual
            ColorMask 0

            HLSLPROGRAM
            #pragma vertex   ShadowVert
            #pragma fragment ShadowFrag
            #pragma multi_compile_shadowcaster

            // Lighting.hlsl 通过正确的依赖顺序包含 Shadows.hlsl，
            // 确保 LerpWhiteTo 和 ApplyShadowBias 都可用
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4  _Color; half4 _InteriorColor; half4 _SpecularColor; half4 _SSSColor;
                float4 _MainTex_ST; float4 _NormalMap_ST; float4 _SpecMap_ST;
                half   _TextureStrength; float _TriplanarScale; float _TriplanarBlend;
                half   _NormalStrength;  half  _Roughness;      half  _SpecularStrength;
                half   _SpecMapStrength; half  _SSSStrength;    half  _SSSDirect;
                half   _SSSPower;        half  _SSSWrap;        half  _FresnelStrength;
                half   _FresnelPow;
            CBUFFER_END

            float3 _LightDirection;

            struct Attributes { float4 positionOS : POSITION; float3 normalOS : NORMAL; };
            struct Varyings   { float4 positionCS : SV_POSITION; };

            Varyings ShadowVert(Attributes IN)
            {
                Varyings OUT;
                float3 posWS  = TransformObjectToWorld(IN.positionOS.xyz);
                float3 normWS = TransformObjectToWorldNormal(IN.normalOS);
                // ApplyShadowBias: 法线偏移 + 深度偏移, 防止 shadow acne
                posWS = ApplyShadowBias(posWS, normWS, _LightDirection);
                OUT.positionCS = TransformWorldToHClip(posWS);
                return OUT;
            }

            half4 ShadowFrag(Varyings IN) : SV_Target { return 0; }
            ENDHLSL
        }

        // ════════════════════════════════════════════════════════════
        // Pass 3: Depth Only (URP depth prepass, 深度图生成)
        // ════════════════════════════════════════════════════════════
        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode"="DepthOnly" }
            Cull Off
            ZWrite On
            ColorMask R

            HLSLPROGRAM
            #pragma vertex   DepthVert
            #pragma fragment DepthFrag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4  _Color; half4 _InteriorColor; half4 _SpecularColor; half4 _SSSColor;
                float4 _MainTex_ST; float4 _NormalMap_ST; float4 _SpecMap_ST;
                half   _TextureStrength; float _TriplanarScale; float _TriplanarBlend;
                half   _NormalStrength;  half  _Roughness;      half  _SpecularStrength;
                half   _SpecMapStrength; half  _SSSStrength;    half  _SSSDirect;
                half   _SSSPower;        half  _SSSWrap;        half  _FresnelStrength;
                half   _FresnelPow;
            CBUFFER_END

            struct Attributes { float4 positionOS : POSITION; };
            struct Varyings   { float4 positionCS : SV_POSITION; };

            Varyings DepthVert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionCS = TransformObjectToHClip(IN.positionOS.xyz);
                return OUT;
            }

            half DepthFrag(Varyings IN) : SV_Target { return 0; }
            ENDHLSL
        }
    }

    FallBack "Universal Render Pipeline/Lit"
}
