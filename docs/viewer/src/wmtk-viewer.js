// <wmtk-viewer>: the documentation's interactive mesh viewer.
//
//   <wmtk-viewer input="assets/tetwild_octocat/input.wmv.gz"
//                output="assets/tetwild_octocat/output.wmv.gz"
//                labels="Input|TetWild" color="quality" cut="x"></wmtk-viewer>
//
// input and output are .wmv files, gzipped, written by docs/tools/build_gallery.py. With both,
// the input is always on the left and the output on the right, with one camera; with one of them
// (or src), a single view. A triangle mesh is drawn with its edges; a tet mesh with a cut: the
// tets whose centroid is on the near side of a plane are hidden, and the faces the cut exposes
// are colored by element quality or by region. The viewer starts when it scrolls into view and
// stops (releasing its WebGL context) when it leaves.
//
// Built into docs/assets/viewer/wmtk-viewer.min.js by `npm run build` in docs/viewer.

import {
    BufferAttribute,
    BufferGeometry,
    Color,
    LineBasicMaterial,
    LineSegments,
    Mesh,
    OrthographicCamera,
    PerspectiveCamera,
    Scene,
    ShaderMaterial,
    Vector3,
    WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

// ---------------------------------------------------------------------------------------------
// The asset: "WMV1", a uint32 header length, a JSON header, padding to 4 bytes, then the arrays
// the header lists by offset (relative to the end of the padding) and type.

const TYPES = {
    uint8: Uint8Array,
    uint16: Uint16Array,
    uint32: Uint32Array,
    int32: Int32Array,
    float32: Float32Array,
};

async function fetchAsset(url) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`${url}: ${response.status} ${response.statusText}`);
    let bytes = new Uint8Array(await response.arrayBuffer());
    // GitHub Pages serves .gz files as they are; a server that sets Content-Encoding has
    // already decompressed them.
    if (bytes[0] === 0x1f && bytes[1] === 0x8b) {
        const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("gzip"));
        bytes = new Uint8Array(await new Response(stream).arrayBuffer());
    }
    return parseAsset(bytes.buffer);
}

function parseAsset(buffer) {
    const magic = new TextDecoder().decode(new Uint8Array(buffer, 0, 4));
    if (magic !== "WMV1") throw new Error("not a .wmv file");
    const headerLength = new DataView(buffer).getUint32(4, true);
    const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 8, headerLength)));
    const base = Math.ceil((8 + headerLength) / 4) * 4;
    const arrays = {};
    for (const [name, a] of Object.entries(header.arrays)) {
        arrays[name] = new TYPES[a.type](buffer, base + a.offset, a.count);
    }
    // Positions are quantized to uint16 per axis over the bounding box.
    const [lo, hi] = header.bbox;
    const q = arrays.positions;
    const positions = new Float32Array(q.length);
    for (let i = 0; i < q.length; ++i) {
        const k = i % 3;
        positions[i] = lo[k] + (q[i] / 65535) * (hi[k] - lo[k]);
    }
    return { header, arrays, positions };
}

// ---------------------------------------------------------------------------------------------
// Colors

const SURFACE = new Color("#c9d3de");
const BOUNDARY = new Color("#d8d8d8");
const WIRE = new Color("#30363d");
// Element quality, good to bad: pale yellow to dark red, so that the bad elements stand out.
const QUALITY_RAMP = ["#ffffcc", "#fed976", "#fd8d3c", "#e31a1c", "#800026"].map((c) => new Color(c));
const REGIONS = ["#4e79a7", "#f28e2b", "#59a14f", "#e15759", "#76b7b2", "#edc948", "#b07aa1", "#9c755f"]
    .map((c) => new Color(c));

function ramp(t, out) {
    t = Math.min(1, Math.max(0, t)) * (QUALITY_RAMP.length - 1);
    const i = Math.min(QUALITY_RAMP.length - 2, Math.floor(t));
    return out.copy(QUALITY_RAMP[i]).lerp(QUALITY_RAMP[i + 1], t - i);
}

// Quality on a log scale between the asset's min and max energy.
function qualityColors(asset) {
    const q = asset.arrays.cell_quality;
    if (!q) return null;
    const { min, max } = asset.header.quality;
    const lmin = Math.log(min), span = Math.max(1e-12, Math.log(max) - lmin);
    const colors = new Float32Array(q.length * 3);
    const c = new Color();
    for (let i = 0; i < q.length; ++i) {
        ramp((Math.log(q[i]) - lmin) / span, c);
        colors.set([c.r, c.g, c.b], 3 * i);
    }
    return colors;
}

function regionColors(asset) {
    const r = asset.arrays.cell_region;
    if (!r) return null;
    const colors = new Float32Array(r.length * 3);
    for (let i = 0; i < r.length; ++i) {
        const c = r[i] === 255 ? BOUNDARY : REGIONS[r[i] % REGIONS.length];
        colors.set([c.r, c.g, c.b], 3 * i);
    }
    return colors;
}

// ---------------------------------------------------------------------------------------------
// The material: flat shading from screen-space derivatives (no normals needed), a headlight lit
// from both sides, and the triangles' edges from barycentric coordinates.

function makeMaterial() {
    return new ShaderMaterial({
        uniforms: { wire: { value: 1.0 }, wireColor: { value: WIRE } },
        vertexShader: /* glsl */ `
            attribute vec3 bary;
            attribute vec3 color;
            varying vec3 vBary;
            varying vec3 vColor;
            varying vec3 vView;
            void main() {
                vBary = bary;
                vColor = color;
                vec4 mv = modelViewMatrix * vec4(position, 1.0);
                vView = mv.xyz;
                gl_Position = projectionMatrix * mv;
            }`,
        fragmentShader: /* glsl */ `
            uniform float wire;
            uniform vec3 wireColor;
            varying vec3 vBary;
            varying vec3 vColor;
            varying vec3 vView;
            void main() {
                vec3 n = normalize(cross(dFdx(vView), dFdy(vView)));
                float light = abs(dot(n, normalize(-vView)));
                vec3 c = vColor * (0.4 + 0.6 * light);
                vec3 d = fwidth(vBary);
                vec3 a = smoothstep(vec3(0.0), d * 1.2, vBary);
                float edge = 1.0 - min(min(a.x, a.y), a.z);
                // Fade the edges of triangles a few pixels wide, which would otherwise fill
                // dense regions with the edge color.
                float fade = 1.0 - smoothstep(0.08, 0.35, max(max(d.x, d.y), d.z));
                gl_FragColor = vec4(mix(c, wireColor, edge * wire * 0.85 * fade), 1.0);
            }`,
    });
}

// Non-indexed triangles: per corner, its position, its barycentric unit vector and its face's
// color.
function triangleGeometry(positions, tris, faceColors) {
    const n = tris.length / 3;
    const pos = new Float32Array(9 * n);
    const bary = new Float32Array(9 * n);
    const col = new Float32Array(9 * n);
    for (let f = 0; f < n; ++f) {
        for (let k = 0; k < 3; ++k) {
            const v = tris[3 * f + k];
            pos.set(positions.subarray(3 * v, 3 * v + 3), 9 * f + 3 * k);
            bary[9 * f + 3 * k + k] = 1;
            col.set(faceColors.subarray(3 * f, 3 * f + 3), 9 * f + 3 * k);
        }
    }
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(pos, 3));
    g.setAttribute("bary", new BufferAttribute(bary, 3));
    g.setAttribute("color", new BufferAttribute(col, 3));
    g.computeBoundingSphere();
    return g;
}

// The four faces of a tet, each opposite one corner.
const TET_FACES = [[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]];

// ---------------------------------------------------------------------------------------------
// One panel: an asset, its scene, and what is drawn of it.

class Panel {
    constructor(asset, label) {
        this.asset = asset;
        this.label = label;
        this.scene = new Scene();
        this.scene.background = null;
        this.material = makeMaterial();
        this.mesh = null;
        const a = asset.arrays;
        this.isTet = !!a.tets;
        if (this.isTet) {
            const T = a.tets, P = asset.positions, m = T.length / 4;
            this.centroids = new Float32Array(3 * m);
            for (let t = 0; t < m; ++t) {
                for (let k = 0; k < 4; ++k) {
                    const v = T[4 * t + k];
                    for (let d = 0; d < 3; ++d) this.centroids[3 * t + d] += 0.25 * P[3 * v + d];
                }
            }
        }
        if (a.lines) {
            const L = a.lines, P = asset.positions;
            const pos = new Float32Array(L.length * 3);
            for (let i = 0; i < L.length; ++i) pos.set(P.subarray(3 * L[i], 3 * L[i] + 3), 3 * i);
            const g = new BufferGeometry();
            g.setAttribute("position", new BufferAttribute(pos, 3));
            this.scene.add(new LineSegments(g, new LineBasicMaterial({ color: "#c0392b" })));
        }
    }

    // "6,968 tets", "37,884 triangles": what the asset holds.
    count() {
        const a = this.asset.arrays;
        const [n, what] = a.tets ? [a.tets.length / 4, "tets"] : [a.triangles.length / 3, "triangles"];
        return `${n.toLocaleString("en-US")} ${what}`;
    }

    // color: "quality", "region" or "plain"; cut: null or { axis, t } with t in [0, 1].
    update(color, cut) {
        const a = this.asset.arrays, P = this.asset.positions;
        const cellColors = color === "quality" ? qualityColors(this.asset)
            : color === "region" ? regionColors(this.asset) : null;
        let tris, faceColors;
        if (this.isTet) {
            const T = a.tets, N = a.tet_neighbors, m = T.length / 4;
            const keep = new Uint8Array(m);
            if (cut) {
                const [lo, hi] = this.asset.header.bbox;
                const plane = lo[cut.axis] + cut.t * (hi[cut.axis] - lo[cut.axis]);
                for (let t = 0; t < m; ++t) keep[t] = this.centroids[3 * t + cut.axis] <= plane;
            } else {
                keep.fill(1);
            }
            const out = [], cols = [];
            for (let t = 0; t < m; ++t) {
                if (!keep[t]) continue;
                for (let f = 0; f < 4; ++f) {
                    const nb = N[4 * t + f];
                    if (nb >= 0 && keep[nb]) continue;
                    const [i, j, k] = TET_FACES[f];
                    out.push(T[4 * t + i], T[4 * t + j], T[4 * t + k]);
                    // A face of the mesh's boundary is plain; a face the cut exposes shows its tet.
                    const c = nb < 0 || !cellColors ? BOUNDARY
                        : new Color(cellColors[3 * t], cellColors[3 * t + 1], cellColors[3 * t + 2]);
                    cols.push(c.r, c.g, c.b);
                }
            }
            tris = Uint32Array.from(out);
            faceColors = Float32Array.from(cols);
        } else {
            tris = a.triangles;
            faceColors = cellColors || new Float32Array(3 * (tris.length / 3)).map((_, i) =>
                [SURFACE.r, SURFACE.g, SURFACE.b][i % 3]);
        }
        if (this.mesh) {
            this.scene.remove(this.mesh);
            this.mesh.geometry.dispose();
        }
        this.mesh = new Mesh(triangleGeometry(P, tris, faceColors), this.material);
        this.scene.add(this.mesh);
    }
}

// ---------------------------------------------------------------------------------------------
// The element

const STYLE = `
:host { display: block; position: relative; font: 13px/1.4 var(--md-text-font, sans-serif); }
.frame { position: absolute; inset: 0; border-radius: 4px; overflow: hidden;
         background: linear-gradient(#fafbfc, #e9edf1); }
:host-context([data-md-color-scheme="slate"]) .frame { background: linear-gradient(#2b2f36, #1d2026); }
canvas { display: block; width: 100%; height: 100%; touch-action: none; }
.message { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
           color: #666; text-align: center; padding: 1em; }
.labels { position: absolute; top: 6px; left: 0; right: 0; display: flex; pointer-events: none; }
.labels span { flex: 1; text-align: center; color: #555; font-weight: 600; }
.controls { position: absolute; left: 6px; right: 6px; bottom: 6px; display: flex; flex-wrap: wrap;
            gap: 6px 12px; align-items: center; padding: 4px 8px; border-radius: 4px;
            background: rgba(255, 255, 255, 0.85); color: #222; }
.controls label { display: inline-flex; gap: 4px; align-items: center; }
.controls input[type=range] { width: 120px; }
.legend { display: inline-flex; gap: 4px; align-items: center; }
.legend .bar { width: 90px; height: 10px; border-radius: 2px;
               background: linear-gradient(to right, #ffffcc, #fed976, #fd8d3c, #e31a1c, #800026); }
.poster { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain; }
`;

class WmtkViewer extends HTMLElement {
    connectedCallback() {
        if (this.shadowRoot) return;
        const root = this.attachShadow({ mode: "open" });
        root.innerHTML = `<style>${STYLE}</style><div class="frame">
            ${this.getAttribute("poster") ? `<img class="poster" alt="${this.getAttribute("alt") || ""}" src="${this.getAttribute("poster")}">` : ""}
            <div class="message">Loading…</div></div>`;
        this.frame = root.querySelector(".frame");
        this.message = root.querySelector(".message");
        this.observer = new IntersectionObserver((entries) => {
            for (const e of entries) e.isIntersecting ? this.start() : this.stop();
        }, { rootMargin: "200px" });
        this.observer.observe(this);
    }

    disconnectedCallback() {
        this.observer?.disconnect();
        this.stop();
    }

    async start() {
        if (this.renderer) return;
        try {
            if (!this.panels) {
                // Input on the left, output on the right.
                const views = this.hasAttribute("src") ? [["src", ""]]
                    : [["input", "Input"], ["output", "Output"]].filter(([a]) => this.hasAttribute(a));
                if (!views.length) throw new Error("no input, output or src attribute");
                const labels = (this.getAttribute("labels") || "").split("|");
                const assets = await Promise.all(views.map(([a]) =>
                    fetchAsset(new URL(this.getAttribute(a), document.baseURI).href)));
                this.panels = assets.map((a, i) => new Panel(a, labels[i] || views[i][1]));
                this.state = {
                    color: this.getAttribute("color") || "quality",
                    cut: this.getAttribute("cut") || (this.panels.some((p) => p.isTet) ? "x" : "off"),
                    t: parseFloat(this.getAttribute("cut-position") || "0.5"),
                    wire: true,
                };
                this.update();
            }
            this.createRenderer();
        } catch (error) {
            this.message.textContent = `Could not load the mesh: ${error.message}. ` +
                "Generate the gallery's assets with docs/tools/build_gallery.py.";
            console.error(error);
        }
    }

    stop() {
        if (!this.renderer) return;
        this.controls.dispose();
        this.renderer.dispose();
        this.renderer.forceContextLoss();
        this.renderer.domElement.remove();
        this.renderer = null;
        this.resizeObserver.disconnect();
    }

    update() {
        const cut = this.state.cut === "off" ? null : { axis: "xyz".indexOf(this.state.cut), t: this.state.t };
        for (const p of this.panels) {
            p.update(this.state.color, cut);
            p.material.uniforms.wire.value = this.state.wire ? 1 : 0;
        }
        this.render();
    }

    createRenderer() {
        const header = this.panels[0].asset.header;
        const [lo, hi] = header.bbox;
        const center = new Vector3().fromArray(lo).add(new Vector3().fromArray(hi)).multiplyScalar(0.5);
        const diagonal = new Vector3().fromArray(hi).sub(new Vector3().fromArray(lo)).length();
        this.is2d = header.dim === 2;

        this.renderer = new WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setScissorTest(true);
        this.frame.prepend(this.renderer.domElement);
        this.message.remove();
        this.frame.querySelector(".poster")?.remove();

        if (!this.camera) {
            if (this.is2d) {
                this.camera = new OrthographicCamera(-1, 1, 1, -1, -diagonal * 10, diagonal * 10);
                this.camera.position.copy(center).add(new Vector3(0, 0, diagonal));
                this.camera.zoom = 1;
                this.halfHeight = 0.6 * diagonal;
            } else {
                this.camera = new PerspectiveCamera(30, 1, diagonal * 0.01, diagonal * 20);
                const view = header.camera ? new Vector3().fromArray(header.camera) : new Vector3(1, 0.6, 1.6);
                this.camera.position.copy(center).add(view.normalize().multiplyScalar(diagonal * 1.6));
            }
            this.target = center;
        }
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.copy(this.target);
        this.controls.enableRotate = !this.is2d;
        this.controls.addEventListener("change", () => this.render());
        this.controls.update();

        this.buildUi();
        this.resizeObserver = new ResizeObserver(() => this.resize());
        this.resizeObserver.observe(this);
        this.resize();
    }

    buildUi() {
        if (this.frame.querySelector(".controls")) return;
        const hasTet = this.panels.some((p) => p.isTet);
        const hasQuality = this.panels.some((p) => p.asset.arrays.cell_quality);
        const hasRegions = this.panels.some((p) => p.asset.arrays.cell_region);
        const ui = document.createElement("div");
        ui.className = "controls";
        const option = (v, l) => `<option value="${v}" ${this.state.color === v ? "selected" : ""}>${l}</option>`;
        ui.innerHTML = `
            <label><input type="checkbox" name="wire" checked> Edges</label>
            ${hasQuality || hasRegions ? `<label>Color <select name="color">
                ${hasQuality ? option("quality", "Quality") : ""}${hasRegions ? option("region", "Regions") : ""}
                ${option("plain", "Plain")}</select></label>` : ""}
            ${hasTet ? `<label>Cut <select name="cut">${["off", "x", "y", "z"].map((a) =>
                `<option ${this.state.cut === a ? "selected" : ""}>${a}</option>`).join("")}</select></label>
                <input type="range" name="t" min="0" max="1" step="0.005" value="${this.state.t}">` : ""}
            ${hasQuality ? `<span class="legend" title="Element quality (AMIPS energy, log scale)">good
                <span class="bar"></span> bad</span>` : ""}`;
        ui.addEventListener("input", (e) => {
            const { name, value, checked } = e.target;
            if (name === "wire") this.state.wire = checked;
            else if (name === "t") this.state.t = parseFloat(value);
            else this.state[name] = value;
            this.update();
        });
        this.frame.append(ui);
        if (this.panels.some((p) => p.label)) {
            const labels = document.createElement("div");
            labels.className = "labels";
            labels.innerHTML = this.panels.map((p) => `<span>${p.label} · ${p.count()}</span>`).join("");
            this.frame.append(labels);
        }
    }

    resize() {
        if (!this.renderer) return;
        const { width, height } = this.getBoundingClientRect();
        this.renderer.setSize(width, height, false);
        const aspect = width / this.panels.length / height;
        if (this.is2d) {
            this.camera.top = this.halfHeight;
            this.camera.bottom = -this.halfHeight;
            this.camera.left = -this.halfHeight * aspect;
            this.camera.right = this.halfHeight * aspect;
        } else {
            this.camera.aspect = aspect;
        }
        this.camera.updateProjectionMatrix();
        this.render();
    }

    render() {
        if (!this.renderer) return;
        const { width, height } = this.renderer.domElement.getBoundingClientRect();
        const w = width / this.panels.length;
        this.panels.forEach((p, i) => {
            this.renderer.setViewport(i * w, 0, w, height);
            this.renderer.setScissor(i * w, 0, w, height);
            this.renderer.render(p.scene, this.camera);
        });
        this.dispatchEvent(new Event("rendered"));
    }
}

customElements.define("wmtk-viewer", WmtkViewer);
