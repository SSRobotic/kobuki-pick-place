"""Pack the official CAD meshes into one GLB for the website.

Kobuki main body + wheel: Yujin Robot COLLADA from kobuki_ros/kobuki_description (textured).
SO-100: STL parts from MuJoCo Menagerie trs_so_arm100 (each part stays in its own body frame).
Usage: py -I cad2glb.py <cad_dl dir> <out.glb | out.json>
"""
import io
import json
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

SRC, OUT = sys.argv[1], sys.argv[2]
SO100_PARTS = ["Base", "Base_Motor", "Rotation_Pitch", "Rotation_Pitch_Motor", "Upper_Arm", "Upper_Arm_Motor",
               "Lower_Arm", "Lower_Arm_Motor", "Wrist_Pitch_Roll", "Wrist_Pitch_Roll_Motor", "Fixed_Jaw",
               "Fixed_Jaw_Motor", "Moving_Jaw"]
CREASE = np.cos(np.radians(35))


def read_stl(fn):
    b = open(fn, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(np.float64)


def crease_normals(tri):
    """Per-corner normals averaged over neighbouring faces within the crease angle (keeps CAD edges sharp)."""
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    fn = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
    corners = tri.reshape(-1, 3)
    _, vid = np.unique(np.round(corners, 6), axis=0, return_inverse=True)
    vid = vid.ravel()
    face = np.repeat(np.arange(len(tri)), 3)
    order = np.argsort(vid, kind="stable")
    bounds = np.flatnonzero(np.diff(vid[order])) + 1
    out = np.empty_like(corners)
    for grp in np.split(order, bounds):
        f = face[grp]
        dots = fn[f] @ fn[f].T
        acc = (dots > CREASE).astype(float) @ cross[f]
        out[grp] = acc / np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-12)
    return corners, out


def weld(*cols):
    """Merge identical corners into an indexed mesh."""
    key = np.round(np.hstack(cols), 5)
    uniq, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return [c[first] for c in cols], inv.ravel().astype(np.uint32)


def read_dae(fn):
    r = ET.parse(fn).getroot()
    ns = {"c": r.tag.split("}")[0][1:]}
    mesh = r.find(".//c:mesh", ns)
    srcs = {s.get("id"): np.array(s.find("c:float_array", ns).text.split(), float).reshape(-1, int(s.find(".//c:accessor", ns).get("stride")))
            for s in mesh.findall("c:source", ns)}
    verts = mesh.find("c:vertices", ns)
    vsrc = verts.find("c:input[@semantic='POSITION']", ns).get("source")[1:]
    tris = mesh.find("c:triangles", ns)
    inputs = {i.get("semantic"): (int(i.get("offset")), i.get("source")[1:]) for i in tris.findall("c:input", ns)}
    stride = max(o for o, _ in inputs.values()) + 1
    p = np.array(tris.find("c:p", ns).text.split(), int).reshape(-1, stride)
    pos = srcs[vsrc][p[:, inputs["VERTEX"][0]]]
    nor = srcs[inputs["NORMAL"][1]][p[:, inputs["NORMAL"][0]]]
    uv = srcs[inputs["TEXCOORD"][1]][p[:, inputs["TEXCOORD"][0]]][:, :2]
    M = np.array(r.find(".//c:visual_scene//c:node/c:matrix", ns).text.split(), float).reshape(4, 4)
    pos = pos @ M[:3, :3].T + M[:3, 3]
    nor = nor @ M[:3, :3].T
    nor /= np.maximum(np.linalg.norm(nor, axis=1, keepdims=True), 1e-12)
    uv = np.column_stack([uv[:, 0], 1.0 - uv[:, 1]])  # glTF UV origin is top-left
    return pos, nor, uv


class GLB:
    def __init__(self):
        self.bin = bytearray()
        self.j = {"asset": {"version": "2.0", "generator": "cad2glb.py"}, "scene": 0, "scenes": [{"nodes": []}],
                  "nodes": [], "meshes": [], "materials": [], "accessors": [], "bufferViews": [],
                  "buffers": [], "images": [], "textures": [], "samplers": [{"magFilter": 9729, "minFilter": 9987}]}

    def view(self, data, target=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        v = {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data)}
        if target:
            v["target"] = target
        self.bin += data
        self.j["bufferViews"].append(v)
        return len(self.j["bufferViews"]) - 1

    def acc(self, arr, kind, target):
        comp = 5126 if arr.dtype == np.float32 else (5123 if arr.dtype == np.uint16 else 5125)
        a = {"bufferView": self.view(arr.tobytes(), target), "componentType": comp, "count": len(arr), "type": kind}
        if kind == "VEC3" and arr.dtype == np.float32:
            a["min"], a["max"] = arr.min(0).tolist(), arr.max(0).tolist()
        self.j["accessors"].append(a)
        return len(self.j["accessors"]) - 1

    def material(self, name, color, rough, metal=0.0, image=None):
        m = {"name": name, "pbrMetallicRoughness": {"baseColorFactor": color + [1.0], "roughnessFactor": rough, "metallicFactor": metal}}
        if image is not None:
            buf = io.BytesIO()
            image.convert("RGB").save(buf, "JPEG", quality=82, optimize=True)
            self.j["images"].append({"bufferView": self.view(buf.getvalue()), "mimeType": "image/jpeg"})
            self.j["textures"].append({"sampler": 0, "source": len(self.j["images"]) - 1})
            m["pbrMetallicRoughness"]["baseColorTexture"] = {"index": len(self.j["textures"]) - 1}
        self.j["materials"].append(m)
        return len(self.j["materials"]) - 1

    def mesh(self, name, pos, nor, idx, mat, uv=None):
        attrs = {"POSITION": self.acc(pos.astype(np.float32), "VEC3", 34962),
                 "NORMAL": self.acc(nor.astype(np.float32), "VEC3", 34962)}
        if uv is not None:
            attrs["TEXCOORD_0"] = self.acc(uv.astype(np.float32), "VEC2", 34962)
        idx = idx.astype(np.uint16 if len(pos) < 65536 else np.uint32)
        prim = {"attributes": attrs, "indices": self.acc(idx, "SCALAR", 34963), "material": mat}
        self.j["meshes"].append({"name": name, "primitives": [prim]})
        self.j["nodes"].append({"name": name, "mesh": len(self.j["meshes"]) - 1})
        self.j["scenes"][0]["nodes"].append(len(self.j["nodes"]) - 1)
        print(f"{name:24s} verts {len(pos):6d} tris {len(idx) // 3:6d}")

    def save_json(self, fn):
        """Same model as a .gltf JSON with the buffer inlined (for hosts that only serve .json)."""
        import base64
        j = dict(self.j, buffers=[{"byteLength": len(self.bin), "uri": "data:application/octet-stream;base64," + base64.b64encode(bytes(self.bin)).decode()}])
        open(fn, "w").write(json.dumps(j, separators=(",", ":")))

    def save(self, fn):
        self.j["buffers"] = [{"byteLength": len(self.bin)}]
        js = json.dumps(self.j, separators=(",", ":")).encode()
        js += b" " * (-len(js) % 4)
        while len(self.bin) % 4:
            self.bin.append(0)
        with open(fn, "wb") as f:
            f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(self.bin)))
            f.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
            f.write(struct.pack("<II", len(self.bin), 0x004E4942) + self.bin)


g = GLB()
# Kobuki: body placed like kobuki.urdf.xacro (base_joint z 0.0102 + visual origin 0.001, 0, 0.05199) so z=0 is the floor
for name, off, tex_size in (("kobuki_body", (0.001, 0.0, 0.0102 + 0.05199), 1024), ("kobuki_wheel", (0.0, 0.0, 0.0), 512)):
    stem = "main_body" if name == "kobuki_body" else "wheel"
    pos, nor, uv = read_dae(f"{SRC}/kobuki/{stem}.dae")
    pos = pos + np.array(off)
    (pos, nor, uv), idx = weld(pos, nor, uv)
    img = Image.open(f"{SRC}/kobuki/images/{stem}.jpg").resize((tex_size, tex_size), Image.LANCZOS)
    g.mesh(name, pos, nor, idx, g.material(name, [1.0, 1.0, 1.0], 0.6, 0.0, img), uv)

printed = g.material("so100_printed", [0.93, 0.40, 0.08], 0.72)
motor = g.material("so100_servo", [0.06, 0.06, 0.07], 0.45, 0.1)
for p in SO100_PARTS:
    pos, nor = crease_normals(read_stl(f"{SRC}/so100/{p}.stl"))
    (pos, nor), idx = weld(pos, nor)
    g.mesh("so100_" + p, pos, nor, idx, motor if p.endswith("_Motor") else printed)

g.save(OUT) if OUT.endswith(".glb") else g.save_json(OUT)
print("wrote", OUT, round(len(g.bin) / 1e6, 2), "MB")
