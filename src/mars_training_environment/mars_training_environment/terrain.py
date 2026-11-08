"""Height fields for placement, the picture, and the labeler.

Historical profiles retain separate coarse collision and fine visual fields.
mixed_reachable v1 uses one triangle surface for all consumers; grain is texture.
"""
import math
from pathlib import Path

import numpy as np

from .common import png, write_json
from .meshes import write_dae


class Terrain:
    def __init__(self, heights, size):
        self.h = np.asarray(heights, dtype=float)
        self.size = float(size)
        self.n = len(self.h)
        self.step = self.size / (self.n - 1)

    def height(self, x, y):
        """Interpolate the same diagonal as the explicit collision triangles."""
        u = np.clip((np.asarray(x)+self.size/2)/self.step, 0, self.n-1-1e-9)
        v = np.clip((np.asarray(y)+self.size/2)/self.step, 0, self.n-1-1e-9)
        i, j = u.astype(int), v.astype(int)
        a, b = u-i, v-j
        z00, z10 = self.h[j, i], self.h[j, i+1]
        z01, z11 = self.h[j+1, i], self.h[j+1, i+1]
        return np.where(a+b <= 1, z00 + a*(z10-z00)+b*(z01-z00),
                        z11+(1-a)*(z01-z11)+(1-b)*(z10-z11))

    def gradient(self, x, y):
        d = self.step * .2
        return np.array([(self.height(x+d,y)-self.height(x-d,y))/(2*d),
                         (self.height(x,y+d)-self.height(x,y-d))/(2*d)])

    def mesh(self):
        coords = np.linspace(-self.size/2, self.size/2, self.n)
        x, y = np.meshgrid(coords, coords)
        vertices = np.column_stack([x.ravel(), y.ravel(), self.h.ravel()])
        faces = []
        for j in range(self.n-1):
            for i in range(self.n-1):
                a = j*self.n+i
                faces.extend([[a, a+1, a+self.n], [a+1, a+self.n+1, a+self.n]])
        return vertices, np.asarray(faces, dtype=int)

    def maximum_slope(self):
        v, f = self.mesh()
        normals = np.cross(v[f[:,1]]-v[f[:,0]], v[f[:,2]]-v[f[:,0]])
        return float(np.max(np.arctan2(np.linalg.norm(normals[:, :2], axis=1), normals[:, 2])))


def _spawn_blend(x, y, clearance):
    """Zero inside the spawn pad, smoothstep to 1 over the next 1.2 m."""
    distance = np.hypot(x, y)
    blend = np.clip((distance-clearance)/1.2, 0, 1)
    return blend*blend*(3-2*blend)


def _ripples(rng, x, y, blend, config):
    """Centimetre wind ripples and low ridges. Zero on the spawn pad."""
    spec = config['ripples']
    out = np.zeros(x.shape, dtype=float)
    for _ in range(spec['waves']):
        angle = rng.uniform(0, 2*math.pi)
        along = x*math.cos(angle)+y*math.sin(angle)
        wavelength = rng.uniform(*spec['wavelength_m'])
        wave = np.sin(along*(2*math.pi/wavelength)+rng.uniform(0, 2*math.pi))
        out += rng.uniform(*spec['height_m'])*np.tanh(1.8*wave)/math.tanh(1.8)
    for _ in range(spec['ridges']):
        angle = rng.uniform(0, 2*math.pi)
        along = x*math.cos(angle)+y*math.sin(angle)
        wavelength = rng.uniform(*spec['ridge_wavelength_m'])
        out += rng.uniform(*spec['ridge_height_m'])*np.sin(along*(2*math.pi/wavelength)+rng.uniform(0, 2*math.pi))
    # Wind ripples are patchy. A full-strength sine across the whole plain
    # reads as corduroy.
    envelope = np.full(x.shape, 0.25)
    for _ in range(3):
        angle = rng.uniform(0, 2*math.pi)
        along = x*math.cos(angle)+y*math.sin(angle)
        envelope += rng.uniform(.25, .45)*np.sin(along*(2*math.pi/rng.uniform(2.5, 6.0))+rng.uniform(0, 2*math.pi))
    out *= np.clip(envelope, 0, 1)
    return np.clip(out, -0.04, 0.04)*blend


def _value_noise(rng, x, y, spacing, extent):
    u,v=(x+extent)/spacing,(y+extent)/spacing
    grid=rng.uniform(-1,1,(math.ceil(float(v.max()))+2,math.ceil(float(u.max()))+2))
    i,j=np.floor(u).astype(int),np.floor(v).astype(int)
    a,b=u-i,v-j
    a=a*a*a*(a*(a*6-15)+10);b=b*b*b*(b*(b*6-15)+10)
    return ((1-a)*(1-b)*grid[j,i]+a*(1-b)*grid[j,i+1]
            +(1-a)*b*grid[j+1,i]+a*b*grid[j+1,i+1])


def _mixed_heights(rng, x, y, size):
    extent=size/2+4
    wx=x+1.2*_value_noise(rng,x,y,5.5,extent)
    wy=y+1.2*_value_noise(rng,x,y,5.5,extent)
    h=sum(weight*_value_noise(rng,wx,wy,scale,extent)
          for scale,weight in [(5.,.36),(2.5,.16),(1.2,.07)])
    for index in range(12):
        cx,cy=rng.uniform(-size*.43,size*.43,2)
        sx,sy=rng.uniform(.55,1.8,2)
        angle=rng.uniform(0,math.tau)
        dx,dy=x-cx,y-cy
        a,b=dx*math.cos(angle)+dy*math.sin(angle),-dx*math.sin(angle)+dy*math.cos(angle)
        amplitude=rng.uniform(.16,.42)*(1 if index%3 else -.7)
        h+=amplitude*np.exp(-.5*((a/sx)**2+(b/sy)**2))
    return h


def generate_terrain(seed, config, folder):
    from .reachability import PROFILE, validate_profile
    validate_profile(config)
    shared=config.get('profile')==PROFILE
    rng = np.random.RandomState(seed)
    n, size = config['grid_size'], config['size_m']
    coords = np.linspace(-size/2, size/2, n)
    x, y = np.meshgrid(coords, coords)
    h = np.zeros_like(x)
    if shared:
        h=_mixed_heights(rng,x,y,size)
    else:
        for _ in range(9):
            angle, phase = rng.uniform(0, 2*math.pi, 2)
            frequency = rng.uniform(.2, 1.5)
            h += rng.uniform(.03, .14)*np.sin((x*math.cos(angle)+y*math.sin(angle))*frequency+phase)
        for _ in range(7):
            cx, cy = rng.uniform(-size*.4, size*.4, 2)
            sx, sy = rng.uniform(.9, 2.8, 2)
            h += rng.uniform(-.4, .4)*np.exp(-((x-cx)/sx)**2-((y-cy)/sy)**2)
    # Flat vertices cover every triangle intersecting the entire clear disk.
    pad=config['spawn_clearance_m']+(math.sqrt(2)*size/(n-1) if shared else 0)
    blend = _spawn_blend(x, y, pad)
    h = (h-h[n//2, n//2])*blend
    h *= config['relief_m']/max(float(np.ptp(h)), 1e-8)
    field = Terrain(h, size)
    slope = field.maximum_slope()
    h *= min(1, math.tan(math.radians(config['max_slope_deg']))/max(math.tan(slope), 1e-8)) * .93
    # The collision mesh is quantized to 8 bits. Ripples are not: an 8-bit step
    # across 0.8 m of relief is already 3 mm, which would erase them.
    low, high = float(h.min()), float(h.max())
    encoded = np.round((h-low)/(high-low)*255).astype(np.uint8)
    h = low + encoded.astype(float)/255*(high-low)
    zero = float(h[n//2, n//2])
    h -= zero
    low -= zero
    base = Terrain(h, size)
    visual_n = config['visual_grid_size']
    fine = np.linspace(-size/2, size/2, visual_n)
    fx, fy = np.meshgrid(fine, fine)
    if shared:
        visual_h=h.copy();visual=base
    else:
        visual_h = base.height(fx, fy) + _ripples(rng, fx, fy, _spawn_blend(fx, fy, config['spawn_clearance_m']), config)
        visual_h -= float(visual_h[visual_n//2, visual_n//2])
        visual = Terrain(visual_h, size)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    png(folder/'heightmap.png', np.flipud(encoded))
    texture_n = config['texture_size']
    yy, xx = np.mgrid[0:texture_n, 0:texture_n]/texture_n
    phase = rng.uniform(0, 6.28, 4)
    patches = (np.sin(xx*19+phase[0]) * np.cos(yy*15+phase[1]) + np.sin((xx+yy)*31+phase[2]))*.5
    dune = np.sin(xx*300+np.sin(yy*70)*2)*1.8
    noise = rng.normal(0, 5, (texture_n, texture_n))
    red = np.array([139, 83, 58])+rng.uniform(-8, 8, 3)
    tan = np.array([173, 143, 106])
    gray = np.array([111, 109, 105])
    texture = red + np.maximum(patches, 0)[:, :, None]*(tan-red) + np.maximum(-patches, 0)[:, :, None]*(gray-red)
    texture += (noise+dune)[:, :, None]
    png(folder/'ground.png', np.clip(texture, 0, 255))
    png(folder/'normal.png', np.tile([128, 128, 255], (8, 8, 1)))
    collision_v, collision_f = base.mesh()
    write_dae(folder/'terrain_collision.dae', collision_v, collision_f)
    visual_v, visual_f = visual.mesh()
    # Scanned ground covers the patch once. Unpainted procedural ground repeats
    # every texture_tile_m; generation uses world UVs when painting candidates.
    if config['ground_textures']:
        uv = np.column_stack([(visual_v[:, 0]+size/2)/size, (visual_v[:, 1]+size/2)/size])
    else:
        uv = visual_v[:, :2]/config['texture_tile_m']
    write_dae(folder/'terrain.dae', visual_v, visual_f, uv=uv)
    document = {'heights': visual_h.tolist(), 'base_heights': h.tolist(), 'size_m': size,
                'heightmap_min_m': low, 'heightmap_range_m': float(h.max()-h.min()),
                'maximum_slope_deg': math.degrees(base.maximum_slope()),
                'grid_size': n, 'visual_grid_size': visual_n, 'spawn_z_m': float(visual.height(0., 0.))}
    if shared:
        from .common import digest
        document.update(profile=PROFILE,profile_version=config.get('profile_version',1),shared_surface=True,
                        surface_sha256=digest({'heights':h.tolist(),'size_m':size}),
                        geometric_ripples=False,collision_mesh='terrain.dae')
    write_json(folder/'terrain.json', document)
    if config.get('profile_version')==2:
        from .environment_v2 import apron_mesh
        vertices,faces=apron_mesh(base,config['apron_m'])
        write_dae(folder/'terrain_apron.dae',vertices,faces,uv=vertices[:,:2]/config['texture_tile_m'])
        (folder/'context_ground.png').write_bytes((folder/'ground.png').read_bytes())
        document['context_geometry']=dict(mesh='terrain_apron.dae',width_m=config['apron_m'],
            extent_m=80.,shared_surface=True,evaluator_only=True,
            geometry_sha256=__import__('hashlib').sha256((folder/'terrain_apron.dae').read_bytes()).hexdigest())
        write_json(folder/'terrain.json',document)
    return visual, base, document
