"""
generate_liver_tet_mesh.py
生成肝臟四面體網格，輸出 Muller Ten-Minute-Physics JSON 格式
依賴: numpy, scipy
"""

import numpy as np
from scipy.spatial import Delaunay
import json
import time
import os
import sys

# 強制使用 UTF-8 輸出（修復 Windows GBK 問題）
sys.stdout.reconfigure(encoding='utf-8')

# ── 配置 ────────────────────────────────────────────────────
# GPU 版可以處理 25k+ tet；尺寸改為 20cm￠視覺清晰
TARGET_TETS = 25000
OUTPUT_PATH = r"D:\Desktop\Tissue_Simulation\Simulation\Assets\StreamingAssets\liver.json"
MESH_SCALE  = 0.20  # 20cm 肝臟，視覺清晰
N_SURFACE   = 3000  # 表面點數量

# 肝臟形狀超橢球半徑（歸一化，判斷點是否在內部時用）
LIVER_A = 1.0   # 左右
LIVER_B = 0.55  # 上下
LIVER_C = 0.65  # 前後

def inside_liver(pts):
    """判斷點 (N,3) 是否在肝臟形內部"""
    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    p = 2.2  # 超橢球冪次（>2 比球更方正）

    dist = (np.abs(x / LIVER_A)**p +
            np.abs(y / LIVER_B)**p +
            np.abs(z / LIVER_C)**p)

    # 左葉收窄（肝臟左側尖細）
    taper = np.ones(len(pts))
    right_mask = x > 0.2
    taper[right_mask] = 1.0 - 0.4 * (x[right_mask] - 0.2)**2
    taper = np.clip(taper, 0.1, 1.0)

    return dist < taper

def sample_liver_points(n_target):
    """均勻採樣 n_target 個肝臟內部點"""
    print(f"[採樣] 在肝臟形狀內部採樣 {n_target} 個點...")
    pts = []
    batch = 60000
    while len(pts) < n_target:
        cands = np.random.uniform(
            [-LIVER_A - 0.1, -LIVER_B - 0.1, -LIVER_C - 0.1],
            [ LIVER_A + 0.1,  LIVER_B + 0.1,  LIVER_C + 0.1],
            (batch, 3)
        )
        mask = inside_liver(cands)
        pts.extend(cands[mask])
        print(f"  已採樣: {min(len(pts), n_target)}/{n_target}", end='\r')
    pts = np.array(pts[:n_target])
    print(f"\n[採樣] 完成: {len(pts)} 個點")
    return pts

def sample_surface_points(n_surface):
    """在肝臟表面採樣點（用於提升表面三角形質量）"""
    print(f"[表面] 採樣 {n_surface} 個表面點...")
    pts = []
    # 用球面均勻分布，縮放到超橢球表面，再微調到肝臟形狀
    while len(pts) < n_surface:
        # 球面均勻分布
        phi   = np.random.uniform(0, 2*np.pi, n_surface * 4)
        theta = np.arccos(np.random.uniform(-1, 1, n_surface * 4))
        sx = np.sin(theta)*np.cos(phi)
        sy = np.sin(theta)*np.sin(phi)
        sz = np.cos(theta)

        # 縮放到超橢球表面（0.995 盡量靠近邊界，確保形成凸包）
        scale = 0.995
        cands = np.stack([sx*LIVER_A*scale,
                          sy*LIVER_B*scale,
                          sz*LIVER_C*scale], axis=1)

        # 只保留在肝臟形狀內的點（去掉形狀修剪後超出的角落）
        mask = inside_liver(cands)
        pts.extend(cands[mask])

    pts = np.array(pts[:n_surface])
    print(f"[表面] 完成: {len(pts)} 個表面點")
    return pts


def extract_boundary_triangles(tets, verts, n_verts):
    """提取邊界三角形並確保法線一致向外"""
    print("[邊界] 提取邊界三角形...")
    # 4 個面的局部索引
    face_local = [(0,2,1),(0,1,3),(0,3,2),(1,2,3)]
    face_count = {}
    face_tri   = {}

    for t in tets:
        for fi in face_local:
            tri = t[list(fi)]
            key = tuple(sorted(tri))
            if key in face_count:
                face_count[key] += 1
            else:
                face_count[key]  = 1
                face_tri[key]    = tri

    boundary = [face_tri[k] for k, v in face_count.items() if v == 1]
    boundary = np.array(boundary, dtype=np.int32)
    print(f"[邊界] 找到 {len(boundary)} 個邊界三角形")

    # ── 修復法線方向（確保全部向外）──────────────────────────────
    print("[法線] 修復邊界三角形法線方向...")
    mesh_centroid = verts.mean(axis=0)  # 網格幾何中心
    flipped = 0
    for i, tri in enumerate(boundary):
        a = verts[tri[0]]
        b = verts[tri[1]]
        c = verts[tri[2]]
        face_center = (a + b + c) / 3.0
        # 叉積計算法線
        n = np.cross(b - a, c - a)
        # 面中心到網格中心的向量（應與法線反向，即法線朝外）
        outward = face_center - mesh_centroid
        if np.dot(n, outward) < 0:
            # 法線朝內 → 翻轉頂點順序
            boundary[i] = [tri[0], tri[2], tri[1]]
            flipped += 1

    print(f"[法線] 修復完成，翻轉了 {flipped}/{len(boundary)} 個三角形")
    return boundary


def main():
    np.random.seed(42)
    t0 = time.time()

    # Step 1: 採樣
    # n_pts ≈ TARGET_TETS / 5.5 (Delaunay 三維規律)
    n_interior = int(TARGET_TETS / 6.5)
    n_surface  = N_SURFACE
    print(f"[Step 1] 目標 {TARGET_TETS} tet -> 內部 {n_interior} + 表面 {n_surface} 點")

    interior_pts = sample_liver_points(n_interior)
    surface_pts  = sample_surface_points(n_surface)
    pts_norm = np.vstack([interior_pts, surface_pts])
    print(f"[Step 1] 合計採樣: {len(pts_norm)} 個點")


    # Step 2: 縮放到米單位
    pts_m = pts_norm * MESH_SCALE

    # Step 3: Delaunay 四面體化（在米單位上進行）
    print(f"[Step 2] Delaunay 四面體化 ({len(pts_m)} 點)...")
    t_d = time.time()
    delaunay = Delaunay(pts_m)
    raw_tets = delaunay.simplices
    print(f"[Step 2] 原始 tet: {len(raw_tets)}, 耗時 {time.time()-t_d:.2f}s")

    # Step 4: 過濾 tet（tet 中心必須在肝臟形內，使用歸一化坐標判斷）
    # tet 中心先轉回歸一化坐標
    tet_centers_norm = pts_norm[raw_tets].mean(axis=1)  # 用原始歸一化坐標
    inside_mask = inside_liver(tet_centers_norm)
    tets = raw_tets[inside_mask]
    print(f"[Step 3] 過濾後 tet: {len(tets)} （移除 {(~inside_mask).sum()} 個外部 tet）")

    if len(tets) < TARGET_TETS * 0.6:
        print(f"[警告] tet 數不足，建議調整 TARGET_TETS 或採樣密度")

    # Step 5: 重新索引（只保留被 active tet 使用的頂點）
    print("[Step 4] 重新索引頂點...")
    used_ids = np.unique(tets)
    old2new  = np.full(len(pts_m), -1, dtype=np.int64)
    old2new[used_ids] = np.arange(len(used_ids))
    new_pts  = pts_m[used_ids]
    new_tets = old2new[tets].astype(np.int32)
    print(f"[Step 4] 最終: {len(new_pts)} 頂點, {len(new_tets)} 四面體")

    # Step 6: 提取邊界三角形（傳入頂點坐標修復法線）
    surf_tris = extract_boundary_triangles(new_tets, new_pts, len(new_pts))

    # Step 6b: 修復四面體頂點順序（確保體積為正，XPBD 需要）
    print("[Tet] 修復四面體頂點順序確保正體積...")
    fixed = 0
    for i, tet in enumerate(new_tets):
        p0, p1, p2, p3 = new_pts[tet[0]], new_pts[tet[1]], new_pts[tet[2]], new_pts[tet[3]]
        vol = np.dot(np.cross(p1 - p0, p2 - p0), p3 - p0) / 6.0
        if vol < 0:
            # 交換 p2 和 p3 使體積為正
            new_tets[i] = [tet[0], tet[1], tet[3], tet[2]]
            fixed += 1
    print(f"[Tet] 修復完成，翻轉了 {fixed}/{len(new_tets)} 個四面體")

    # Step 6c: 過濾退化四面體（體積過小 → XPBD 數值不穩定）
    print("[Tet] 過濾退化四面體...")
    MIN_VOL = 1e-10  # 最小體積閾值（m³）
    vols = np.array([
        abs(np.dot(np.cross(new_pts[t[1]]-new_pts[t[0]],
                            new_pts[t[2]]-new_pts[t[0]]),
                   new_pts[t[3]]-new_pts[t[0]])) / 6.0
        for t in new_tets
    ])
    valid_mask = vols > MIN_VOL
    removed_count = (~valid_mask).sum()
    new_tets = new_tets[valid_mask]
    print(f"[Tet] 移除退化 tet: {removed_count} 個, 剩餘: {len(new_tets)} 個")
    print(f"[Tet] 體積範圍: min={vols[valid_mask].min():.2e}, "
          f"max={vols[valid_mask].max():.2e}, "
          f"mean={vols[valid_mask].mean():.2e}")


    # Step 7: 構建並保存 JSON
    print("[Step 5] 構建 JSON...")
    data = {
        "verts":            new_pts.flatten().tolist(),
        "tetIds":           new_tets.flatten().tolist(),
        "tetSurfaceTriIds": surf_tris.flatten().tolist(),
        "tetEdgeIds":       []
    }

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    print(f"[Step 6] 保存到: {OUTPUT_PATH}")
    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        json.dump(data, f, separators=(',', ':'))

    fsize = os.path.getsize(OUTPUT_PATH) / 1024 / 1024
    total = time.time() - t0

    print()
    print("=" * 52)
    print("  肝臟四面體網格生成完成！")
    print(f"  頂點數:     {len(new_pts):>8,}")
    print(f"  四面體數:   {len(new_tets):>8,}")
    print(f"  表面三角形: {len(surf_tris):>8,}")
    print(f"  網格尺寸:   {MESH_SCALE*2*100:.0f}cm x {MESH_SCALE*LIVER_B*2*100:.0f}cm x {MESH_SCALE*LIVER_C*2*100:.0f}cm")
    print(f"  文件大小:   {fsize:.1f} MB")
    print(f"  總耗時:     {total:.1f}s")
    print("=" * 52)
    print()
    print("下一步:")
    print("  Unity TetMeshLoader.jsonFileName = \"liver.json\"")

if __name__ == "__main__":
    main()
