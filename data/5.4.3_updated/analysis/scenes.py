"""Frozen mixed-forest geometry, voxel map, and deterministic odometry noise."""
from pathlib import Path
import hashlib
import json
import math
import random

import numpy as np
from scipy.ndimage import label

RADIUS = 0.322
START = (-15.0, 0.0, 0.1)
GOAL = (15.0, 0.0, 1.0)
NOISE = {"none": (0., 0.), "low": (.015, .025), "medium": (.035, .060)}
RES = .1
HEIGHT = 1.9
ENTRANCE = 5.
THICKNESS = .5


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wall_nodes(width, length):
    x = np.linspace(-length, length, 2 * math.ceil(length / RES) + 1)
    y = width / 2 + (ENTRANCE - width) / 2 * np.sin(np.pi * x / (2 * length)) ** 2
    return np.column_stack((x, y))


def forest(seed, length, density, gap=0.15):
    rng = random.Random(seed)
    objects = []
    for lo, hi in [(-24.5, -length), (length, 24.5)]:
        side = []
        for shape in ("cylinder", "ellipsoid"):
            count = 0
            for _ in range(density * 50):
                if count == density:
                    break
                x, y = rng.uniform(lo, hi), rng.uniform(-12, 12)
                if shape == "cylinder":
                    r, h = rng.uniform(.5, .7), rng.uniform(1, HEIGHT)
                    item = dict(shape=shape, x=x, y=y, r=r, h=h)
                else:
                    a, b, c = rng.uniform(.5, .7), rng.uniform(.5, .7), rng.uniform(.7, .8)
                    r = max(a, b)
                    item = dict(shape=shape, x=x, y=y, z=rng.uniform(c, HEIGHT-c),
                                a=a, b=b, c=c, yaw=rng.uniform(-.5, .5))
                if any(math.hypot(x - endpoint, y) < r + 2 for endpoint in (-15, 15)):
                    continue
                if abs(x) - r <= length + .2 and abs(y) - r <= ENTRANCE / 2 + THICKNESS + .3:
                    continue
                if any(math.hypot(x - ox, y - oy) < r + footprint + gap
                       for ox, oy, footprint in side):
                    continue
                side.append((x, y, r))
                objects.append(item)
                count += 1
    return objects


def geometry(config, seed):
    return dict(wall_nodes=wall_nodes(config["width_m"], config["contraction_m"]).tolist(),
                wall_height_m=HEIGHT, wall_thickness_m=THICKNESS,
                obstacles=forest(seed, config["contraction_m"], config["forest_density"],
                                 config.get("forest_gap_m", 0.15)))


def points_for_map(scene):
    arrays = []
    for item in scene["obstacles"]:
        if item["shape"] == "cylinder":
            r = item["r"]
            v = np.arange(-math.floor(r/RES), math.floor(r/RES)+1) * RES
            x, y, z = np.meshgrid(v, v, np.arange(0, item["h"]+1e-8, RES), indexing="ij")
            keep = x*x + y*y <= r*r
        else:
            a, b, c = (item[k] for k in ("a", "b", "c"))
            v = np.arange(-math.ceil(max(a,b)/RES), math.ceil(max(a,b)/RES)+1)*RES
            w = np.arange(-math.floor(c/RES), math.floor(c/RES)+1)*RES
            x, y, z = np.meshgrid(v, v, w, indexing="ij")
            co, si = math.cos(item["yaw"]), math.sin(item["yaw"])
            keep = ((co*x+si*y)/a)**2 + ((-si*x+co*y)/b)**2 + (z/c)**2 <= 1
        arrays.append(np.column_stack((x[keep]+item["x"], y[keep]+item["y"],
                                       z[keep]+item.get("z", 0))))
    nodes = np.asarray(scene["wall_nodes"])
    x, j, z = np.meshgrid(nodes[:,0], np.arange(0, THICKNESS+1e-8, RES),
                          np.arange(0, HEIGHT+1e-8, RES), indexing="ij")
    y = j + nodes[:,1,None,None]
    for sign in (-1, 1):
        arrays.append(np.column_stack((x.ravel(), sign*y.ravel(), z.ravel())))
    return np.ascontiguousarray(np.concatenate(arrays), dtype="<f4")


def distance_to_solids(position, scene):
    """Unsigned distance to continuous solids, zero for an interior center."""
    p = np.asarray(position, dtype=float)
    px, py, pz = p.T
    nodes = np.asarray(scene["wall_nodes"])
    dz = np.maximum(np.maximum(-pz, pz-HEIGHT), 0)
    best = np.full(len(p), np.inf)
    for sign in (-1, 1):
        xy = np.column_stack((px, sign*py))
        outer = nodes + [0, THICKNESS]
        polygon = np.vstack((nodes, outer[::-1]))
        wall = np.full(len(p), np.inf)
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            ab = b - a
            ap = xy - a
            proj = np.clip((ap @ ab) / (ab @ ab), 0, 1)
            wall = np.minimum(wall, np.linalg.norm(ap - proj[:,None]*ab, axis=1))
        wall_y = np.interp(px, nodes[:,0], nodes[:,1])
        inside = ((px >= nodes[0,0]) & (px <= nodes[-1,0]) &
                  (sign*py >= wall_y) & (sign*py <= wall_y+THICKNESS))
        best = np.minimum(best, np.hypot(np.where(inside, 0., wall), dz))
    for item in scene["obstacles"]:
        dx, dy = px-item["x"], py-item["y"]
        if item["shape"] == "cylinder":
            radial = np.maximum(np.hypot(dx,dy)-item["r"], 0)
            vertical = np.maximum(np.maximum(-pz,pz-item["h"]), 0)
            best = np.minimum(best, np.hypot(radial, vertical))
            continue
        axes = np.array([item[k] for k in ("a","b","c")])
        dz = pz-item["z"]
        candidate = np.flatnonzero(np.sqrt(dx*dx+dy*dy+dz*dz)-max(axes) < best)
        if not len(candidate):
            continue
        co, si = math.cos(item["yaw"]), math.sin(item["yaw"])
        q = np.column_stack((co*dx[candidate]+si*dy[candidate],
                             -si*dx[candidate]+co*dy[candidate], dz[candidate]))
        out = np.linalg.norm(q/axes, axis=1) > 1.
        d = np.zeros(len(q))
        outside = q[out]
        if len(outside):
            a2 = axes**2
            lo = np.zeros(len(outside))
            hi = np.linalg.norm(outside*axes, axis=1)
            for _ in range(36):
                mid = (lo+hi)/2
                high = np.sum((outside*axes/(a2+mid[:,None]))**2, axis=1)>1
                lo = np.where(high, mid, lo)
                hi = np.where(high, hi, mid)
            d[out] = np.linalg.norm(outside-outside*a2/(a2+hi[:,None]), axis=1)
        best[candidate] = np.minimum(best[candidate], d)
    return best


def valid_map(scene, width, route_margin=0.1):
    """Six-neighbor center route, with a half-edge clearance reserve."""
    x = np.arange(-75, 76) * .2
    y = np.arange(-55, 56) * .2
    z = np.arange(2, 9) * .2
    margin = RADIUS + route_margin
    xx, yy = np.meshgrid(x, y, indexing="ij")
    p = np.column_stack((xx.ravel(), yy.ravel(), np.ones(xx.size)))
    walls = dict(scene, obstacles=[])
    mask = np.repeat((distance_to_solids(p, walls).reshape(xx.shape) > margin)[:,:,None],
                     len(z), axis=2)
    for item in scene["obstacles"]:
        if item["shape"] == "cylinder":
            footprint = item["r"] + margin
        else:
            axes = np.array([item[k] for k in ("a", "b", "c")])
            # Scaling by the smallest axis conservatively includes the sphere dilation.
            axes *= 1 + margin / axes.min()
            footprint = max(axes[:2])
        ix = np.flatnonzero(abs(x-item["x"]) <= footprint)
        iy = np.flatnonzero(abs(y-item["y"]) <= footprint)
        dx = x[ix,None,None] - item["x"]
        dy = y[None,iy,None] - item["y"]
        if item["shape"] == "cylinder":
            dr = np.maximum(np.hypot(dx,dy)-item["r"],0.)
            dz = np.maximum(z[None,None,:]-item["h"],0.)
            blocked = np.hypot(dr,dz) <= margin
        else:
            a,b,c = axes
            co,si = math.cos(item["yaw"]),math.sin(item["yaw"])
            dz = z[None,None,:]-item["z"]
            blocked = ((co*dx+si*dy)/a)**2 + ((-si*dx+co*dy)/b)**2 + (dz/c)**2 <= 1
        mask[ix[:,None],iy[None,:],:] &= ~blocked
    # The x=0 plane can be crossed only inside the opening, at every layer.
    mask[len(x)//2] &= (abs(y) < width/2-RADIUS)[:,None]
    groups, _ = label(mask)
    j,k = len(y)//2, int(np.where(z==1.)[0][0])
    return bool(groups[0,j,k] and groups[0,j,k] == groups[-1,j,k])


def materialize(directory, config, seed, densities=(120,)):
    """First feasible draw under a predeclared geometry-only rejection rule."""
    if (directory/"scenario.json").exists():
        raise RuntimeError("Scene already frozen")
    rejected = {}
    for density in densities:
        config = dict(config, forest_density=density)
        rejected[density] = []
        for offset in range(100):
            scene = geometry(config, seed+offset)
            if valid_map(scene, config["width_m"], config.get("route_margin_m", 0.1)):
                break
            rejected[density].append(seed+offset)
        else:
            print(f"No 3-D route in 100 candidates at density {density}", flush=True)
            continue
        print(f"3-D route verified: seed {seed+offset}, density {density}", flush=True)
        break
    else:
        raise RuntimeError("No feasible 3-D map within the prespecified candidate ranges")
    directory.mkdir(parents=True, exist_ok=True)
    config = dict(config, seed=seed+offset, rejected_map_seeds=rejected, start=START, goal=GOAL)
    ps, vs = NOISE[config["noise"]]
    config.update(position_sigma_m=ps, velocity_sigma_m_s=vs,
                  noise_seed=900000+seed, noise_dt_s=.01)
    (directory/"geometry.json").write_text(json.dumps(scene, indent=2))
    points_for_map(scene).tofile(directory/"map.xyz")
    noise = np.random.default_rng(config["noise_seed"]).normal(size=(12501,6))
    np.savetxt(directory/"noise.txt", noise*np.array([ps]*3+[vs]*3), fmt="%.17g")
    config.update(map_sha256=sha(directory/"map.xyz"), noise_sha256=sha(directory/"noise.txt"),
                  geometry_sha256=sha(directory/"geometry.json"))
    (directory/"scenario.json").write_text(json.dumps(config,indent=2))
    return config
