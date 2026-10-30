#!/usr/bin/env python3
"""学習データの「コーンを避ける区間」を確認・修正する Web ツール。

SmolVLA/training/data/lap_annotation_web.py と同じ操作感にしてある。
ここで印を付けたフレームを、train_curve_weight.py が多めに引く (pylon_oversample)。

画面:
  - 画像 (パイロンマスクを赤で重ねる。M で切り替え)
  - 正解経路 (path/<stem>.csv、上から見た図。前が上、左が左)
  - タイムライン: 上段 = 避ける区間 (オレンジ)、中段 = パイロンが写っているフレーム
    (濃い = 進路の前、薄い = 脇)、下段の緑の線 = 確認済み。クリックで移動、ドラッグで範囲を選ぶ

避ける区間は人が全部引く。初めて開いた bag は全フレーム「避けない・未確認」から始まる
(自動の案は入れない)。タイムライン中段のパイロンの表示は目安だけ。

キー: ←/→ 1 フレーム，Ctrl+←/→ 10 フレーム，Shift+←/→ 範囲を伸ばす (押し始めが始点)，
      Space 再生/停止，[ / ] 範囲の始点/終点を今のフレームに，
      Enter 範囲を「避ける区間」にして確認済みにし次の未確認へ，
      Backspace 範囲を「避けない」にして確認済みにし次の未確認へ，M マスクの表示切り替え，Ctrl+S 保存
保存: <bag>/pylon_scenes.json {"scene": [0/1...], "reviewed": [0/1...], "stems": [...]}

    python3 scripts/pylon_scene_web.py ~/e2e_data/pylon_review --port 8020
    → ブラウザで http://localhost:8020
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

import cv2
import numpy as np

AHEAD_X = (200, 440)  # 進路の前とみなす枠の中心 x (640x360 座標)。表示の目安だけ

ARGS = None


def bag_dirs() -> list[Path]:
    return sorted(p for p in ARGS.root.iterdir() if (p / 'pylon_masks').is_dir() and (p / 'path').is_dir())


def bag_dir(name: str) -> Path:
    # bag はシンボリックリンクでもよい。名前でフォルダの外へ出ることだけ防ぐ
    if name in ('', '.', '..') or '/' in name or '\\' in name:
        raise ValueError(name)
    return ARGS.root / name


def stems(d: Path) -> list[str]:
    return sorted(f.stem for f in (d / 'path').glob('*.csv') if (d / 'images' / f'{f.stem}.png').exists())


def pylon_boxes(mask: np.ndarray) -> list[tuple[float, float]]:
    """塗られた枠ごとの (中心 x, 下端 y)。重なった枠は 1 つにまとまる。"""
    n, _, st, _ = cv2.connectedComponentsWithStats((mask > 127).astype(np.uint8))
    return [(x + w / 2, y + h) for x, y, w, h, _ in st[1:]]


def pylon_state(d: Path, ss: list[str]) -> list[int]:
    """0 = 写っていない、1 = 脇に写っている、2 = 進路の前に写っている。"""
    out = []
    for s in ss:
        m = cv2.imread(str(d / 'pylon_masks' / f'{s}.png'), cv2.IMREAD_GRAYSCALE)
        bs = pylon_boxes(m) if m is not None else []
        out.append(2 if any(AHEAD_X[0] <= x <= AHEAD_X[1] for x, _ in bs) else (1 if bs else 0))
    return out


def read_labels(d: Path, ss: list[str]) -> tuple[list[int], list[int]]:
    f = d / 'pylon_scenes.json'
    if f.exists():
        j = json.loads(f.read_text())
        if j.get('stems') == ss:
            return j['scene'], j['reviewed']
    return [0] * len(ss), [0] * len(ss)


def read_path(d: Path, s: str) -> list[list[float]]:
    return [[round(float(r['x']), 3), round(float(r['y']), 3)] for r in csv.DictReader(open(d / 'path' / f'{s}.csv'))]


def bag_summary(p: Path) -> dict:
    ss = stems(p)
    f = p / 'pylon_scenes.json'
    if f.exists() and json.loads(f.read_text()).get('stems') == ss:
        j = json.loads(f.read_text())
        return {'name': p.name, 'n': len(ss), 'scene': sum(j['scene']), 'reviewed': sum(j['reviewed'])}
    return {'name': p.name, 'n': len(ss), 'scene': None, 'reviewed': 0}


def bag_detail(name: str) -> dict:
    d = bag_dir(name)
    ss = stems(d)
    state = pylon_state(d, ss)
    scene, reviewed = read_labels(d, ss)
    meta = {}
    if (d / 'meta.csv').exists():
        meta = {f"{int(r['index']):05d}": r for r in csv.DictReader(open(d / 'meta.csv'))}
    t0 = float(meta[ss[0]]['timestamp']) if ss and ss[0] in meta else None
    return {'name': name, 'n': len(ss), 'stems': ss, 'state': state, 'scene': scene, 'reviewed': reviewed,
            'paths': [read_path(d, s) for s in ss],
            't': [round(float(meta[s]['timestamp']) - t0, 2) if t0 is not None and s in meta else None for s in ss],
            'speed': [round(float(meta[s]['speed_mps']), 2) if s in meta else None for s in ss]}


def save(name: str, body: dict) -> dict:
    d = bag_dir(name)
    ss = stems(d)
    sc, rv = body['scene'], body['reviewed']
    if len(sc) != len(ss) or len(rv) != len(ss):
        raise ValueError('フレーム数が合わない')
    f = d / 'pylon_scenes.json'
    tmp = f.with_suffix('.json.tmp')
    tmp.write_text(json.dumps({'stems': ss, 'scene': [int(bool(x)) for x in sc],
                               'reviewed': [int(bool(x)) for x in rv]}))
    os.replace(tmp, f)
    return bag_summary(d)


def image(name: str, stem: str, with_mask: bool) -> bytes:
    d = bag_dir(name)
    im = cv2.resize(cv2.imread(str(d / 'images' / f'{stem}.png')), (640, 360))
    if with_mask:
        m = cv2.imread(str(d / 'pylon_masks' / f'{stem}.png'), cv2.IMREAD_GRAYSCALE) > 127
        im[m] = (im[m] * 0.5 + np.array([0, 0, 255]) * 0.5).astype(np.uint8)
        cv2.line(im, (AHEAD_X[0], 0), (AHEAD_X[0], 359), (80, 200, 80), 1)
        cv2.line(im, (AHEAD_X[1], 0), (AHEAD_X[1], 359), (80, 200, 80), 1)
    return cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body: bytes, ctype: str, cache: bool = False):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'max-age=3600' if cache else 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, obj, code=200):
        self.send(code, json.dumps(obj, ensure_ascii=False).encode(), 'application/json; charset=utf-8')

    def do_GET(self):
        path = unquote(self.path.split('?')[0])
        try:
            if path == '/':
                self.send(200, PAGE.encode(), 'text/html; charset=utf-8')
            elif path == '/api/bags':
                self.send_json({'bags': [bag_summary(p) for p in bag_dirs()]})
            elif m := re.fullmatch(r'/api/bag/([^/]+)', path):
                self.send_json(bag_detail(m.group(1)))
            elif m := re.fullmatch(r'/img/([^/]+)/(\d+)_(0|1)\.jpg', path):
                self.send(200, image(m.group(1), m.group(2), m.group(3) == '1'), 'image/jpeg', cache=True)
            else:
                self.send(404, b'not found', 'text/plain')
        except Exception as e:  # noqa: BLE001
            self.send_json({'error': str(e)}, 500)

    def do_POST(self):
        path = unquote(self.path)
        try:
            m = re.fullmatch(r'/api/bag/([^/]+)/save', path)
            if not m:
                return self.send(404, b'not found', 'text/plain')
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            self.send_json(save(m.group(1), body))
        except Exception as e:  # noqa: BLE001
            self.send_json({'error': str(e)}, 500)


PAGE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><title>コーンを避ける区間</title>
<style>
:root{--bg:#16181d;--panel:#20232a;--line:#343842;--text:#e6e6e6;--muted:#9aa0aa;--accent:#4ea1ff;--ok:#3fb950;--scene:#f0883e}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 system-ui,"Noto Sans JP",sans-serif}
header{display:flex;gap:12px;align-items:center;padding:8px 14px;background:var(--panel);border-bottom:1px solid var(--line)}
header select{background:var(--bg);color:var(--text);border:1px solid var(--line);padding:4px 6px;font-size:14px}
#status{margin-left:auto;color:var(--muted)}
main{display:grid;grid-template-columns:minmax(0,calc(max(100vh - 200px,360px) * 16 / 9)) minmax(300px,1fr);gap:12px;padding:12px 14px}
#imgwrap{position:relative;background:#000;border-radius:6px;overflow:hidden}
#img{display:block;width:100%;height:calc(100vh - 200px);min-height:360px;object-fit:contain}
#frameinfo{position:absolute;left:8px;top:6px;background:#000a;padding:2px 8px;border-radius:4px;font-variant-numeric:tabular-nums}
#label{position:absolute;right:8px;top:6px;padding:2px 10px;border-radius:4px;font-weight:700}
aside{display:flex;flex-direction:column;gap:10px}
.box{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px}
#path{width:100%;height:340px;display:block}
.k{color:var(--muted);font-size:12px}
#timeline{width:100%;height:64px;display:block;cursor:crosshair;margin-top:10px;border-radius:4px}
#edit{display:flex;gap:8px;align-items:center;margin-top:10px;flex-wrap:wrap}
button{background:#2b3038;color:var(--text);border:1px solid var(--line);border-radius:4px;padding:7px 12px;font-size:14px;cursor:pointer}
button.primary{background:var(--scene);border-color:var(--scene);color:#1d0f00;font-weight:600}
#range{font-variant-numeric:tabular-nums}
.help{color:var(--muted);font-size:12px;margin-top:6px}
</style></head><body>
<header>
  <strong>コーンを避ける区間</strong>
  <select id="bagsel"></select>
  <span id="bagprog" class="k"></span>
  <span id="status">読み込み中</span>
</header>
<main>
  <section>
    <div id="imgwrap"><img id="img" alt=""><div id="frameinfo"></div><div id="label"></div></div>
    <canvas id="timeline"></canvas>
    <div id="edit">
      <span id="range"></span>
      <button id="yes" class="primary" title="Enter">避ける区間にする → 次へ</button>
      <button id="no" title="Backspace">避けない → 次へ</button>
      <button id="unrev">未確認に戻す</button>
      <label class="k"><input type="checkbox" id="showmask" checked> パイロンマスクを重ねる (M)</label>
    </div>
    <div class="help">←/→ 1フレーム　Ctrl+←/→ 10フレーム　Shift+←/→ 範囲を伸ばす（押し始めが始点）　Space 再生　[ / ] 範囲の始点/終点　Enter 避ける区間にして次へ　Backspace 避けないにして次へ　M マスク　Ctrl+S 保存　タイムライン: クリックで移動、ドラッグで範囲選択。上段オレンジ = 避ける区間、中段 = パイロン（濃い = 進路の前、薄い = 脇）、下段の緑 = 確認済み。画像の緑の縦線の間が「進路の前」</div>
  </section>
  <aside>
    <div class="box">
      <div class="k">正解経路（上から見た図。前が上、1 目盛り 1 m。灰色は直進）</div>
      <canvas id="path"></canvas>
      <div id="pathinfo" class="k"></div>
    </div>
    <div class="box">
      <div class="k">今の区間</div>
      <div id="seginfo"></div>
    </div>
  </aside>
</main>
<script>
let bags=[],B=null,f=0,selA=0,selB=0,anchor=null,dirty=false,playing=null,drag=null;
const $=id=>document.getElementById(id);
const setStatus=t=>$("status").textContent=t;
const mask=()=>$("showmask").checked?1:0;
function segAt(i){const v=B.scene[i];let a=i,b=i;while(a>0&&B.scene[a-1]===v)a--;while(b<B.n-1&&B.scene[b+1]===v)b++;return[a,b]}

async function loadBags(keep){
  const r=await (await fetch("/api/bags")).json();bags=r.bags;
  const sel=$("bagsel"),cur=keep||sel.value;
  sel.innerHTML=bags.map(b=>`<option value="${b.name}">${b.name}　${b.n} フレーム　確認 ${Math.round(100*b.reviewed/b.n)}%　避ける ${b.scene===null?"未作成":b.scene}</option>`).join("");
  if(cur)sel.value=cur;
}
async function openBag(name){
  if(dirty&&!confirm("保存していない変更があります。破棄して開きますか？"))return;
  setStatus("読み込み中");B=await (await fetch("/api/bag/"+encodeURIComponent(name))).json();dirty=false;
  const first=B.reviewed.indexOf(0);
  go(first<0?0:first,true);setStatus("保存済み");
}
function go(i,resetSel){
  f=Math.max(0,Math.min(B.n-1,i));
  if(resetSel){[selA,selB]=segAt(f);anchor=null}
  $("img").src=`/img/${B.name}/${B.stems[f]}_${mask()}.jpg`;
  for(let k=1;k<=6;k++){if(f+k<B.n)(new Image).src=`/img/${B.name}/${B.stems[f+k]}_${mask()}.jpg`}
  render();
}
function render(){
  const t=B.t[f]!==null?`　${B.t[f].toFixed(1)} s`:"",v=B.speed[f]!==null?`　${B.speed[f].toFixed(1)} m/s`:"";
  $("frameinfo").textContent=`${f} / ${B.n-1}（${B.stems[f]}）${t}${v}`;
  const lb=$("label");lb.textContent=(B.scene[f]?"避ける区間":"避けない")+(B.reviewed[f]?"（確認済み）":"（未確認）");
  lb.style.background=B.scene[f]?"var(--scene)":"#2b3038";lb.style.color=B.scene[f]?"#1d0f00":"var(--text)";
  const[a,b]=segAt(f);let fr=0,sd=0;for(let i=a;i<=b;i++){fr+=B.state[i]===2;sd+=B.state[i]===1}
  $("seginfo").textContent=`${a}–${b}（${b-a+1} フレーム）${B.scene[f]?"避ける区間":"避けない"}　パイロンが前 ${fr} / 脇 ${sd}`;
  $("range").textContent=`範囲 ${Math.min(selA,selB)}–${Math.max(selA,selB)}`;
  const rv=B.reviewed.reduce((s,x)=>s+x,0),sc=B.scene.reduce((s,x)=>s+x,0);
  $("bagprog").textContent=`確認 ${rv}/${B.n}（${Math.round(100*rv/B.n)}%）　避ける区間 ${sc} フレーム`;
  drawTimeline();drawPath();
}
function sizeCanvas(c){const r=c.getBoundingClientRect(),dpr=devicePixelRatio||1;c.width=r.width*dpr;c.height=r.height*dpr;const g=c.getContext("2d");g.setTransform(dpr,0,0,dpr,0,0);return[g,r.width,r.height]}
function drawTimeline(){
  const[g,W,Hh]=sizeCanvas($("timeline"));g.fillStyle="#0d0f12";g.fillRect(0,0,W,Hh);
  const x=i=>i/B.n*W,w=Math.max(1,W/B.n);
  for(let i=0;i<B.n;i++){
    if(B.scene[i]){g.fillStyle="#f0883e";g.fillRect(x(i),6,w,24)}
    if(B.state[i]){g.fillStyle=B.state[i]===2?"#f85149":"#6e2f2c";g.fillRect(x(i),34,w,12)}
    if(B.reviewed[i]){g.fillStyle="#3fb950";g.fillRect(x(i),50,w,6)}
  }
  const a=Math.min(selA,selB),b=Math.max(selA,selB);
  g.strokeStyle="#fff";g.lineWidth=2;g.strokeRect(x(a),3,x(b+1)-x(a),46);
  g.fillStyle="#fff";g.fillRect(x(f)-1,0,2,Hh);
}
function drawPath(){
  const[g,W,Hh]=sizeCanvas($("path"));g.fillStyle="#0d0f12";g.fillRect(0,0,W,Hh);
  // 前 0〜11 m を縦、左右 ±6 m を横に。左 (+y) が画面の左
  const s=Math.min((Hh-20)/11,(W-20)/12),cx=W/2,cy=Hh-10;
  const P=(px,py)=>[cx-py*s,cy-px*s];
  g.strokeStyle="#2a2e36";g.lineWidth=1;
  for(let m=-6;m<=6;m++){const[u]=P(0,m);g.beginPath();g.moveTo(u,cy);g.lineTo(u,cy-11*s);g.stroke()}
  for(let m=0;m<=11;m++){const[,v]=P(m,0);g.beginPath();g.moveTo(cx-6*s,v);g.lineTo(cx+6*s,v);g.stroke()}
  g.strokeStyle="#555";g.lineWidth=2;g.beginPath();g.moveTo(...P(0,0));g.lineTo(...P(10,0));g.stroke();
  const p=B.paths[f];
  g.strokeStyle=B.scene[f]?"#f0883e":"#4ea1ff";g.lineWidth=3;g.beginPath();g.moveTo(...P(0,0));for(const q of p)g.lineTo(...P(q[0],q[1]));g.stroke();
  g.fillStyle="#fff";g.beginPath();g.arc(...P(0,0),4,0,7);g.fill();
  const y5=p[9]?p[9][1]:0,y10=p[p.length-1][1];
  $("pathinfo").textContent=`5 m 先の横位置 ${y5>=0?"左":"右"} ${Math.abs(y5).toFixed(2)} m　10 m 先 ${y10>=0?"左":"右"} ${Math.abs(y10).toFixed(2)} m`;
}
function setRange(v){
  const a=Math.min(selA,selB),b=Math.max(selA,selB);
  for(let i=a;i<=b;i++){B.scene[i]=v;B.reviewed[i]=1}
  dirty=true;save();
  let n=B.reviewed.indexOf(0,b+1);if(n<0)n=B.reviewed.indexOf(0);
  if(n<0){go(b,true);setStatus("この bag はすべて確認済み");return}go(n,true);
}
async function save(){
  if(!B)return;setStatus("保存中");
  const r=await (await fetch(`/api/bag/${encodeURIComponent(B.name)}/save`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({scene:B.scene,reviewed:B.reviewed})})).json();
  if(r.error){setStatus("保存に失敗: "+r.error);return}
  dirty=false;setStatus("保存済み");loadBags(B.name);
}
function togglePlay(){
  if(playing){clearInterval(playing);playing=null;return}
  playing=setInterval(()=>{if(f>=B.n-1){togglePlay();return}go(f+1,false)},250);
}
document.addEventListener("keydown",e=>{
  if(!B)return;
  if((e.ctrlKey||e.metaKey)&&e.key==="s"){e.preventDefault();save();return}
  const k=e.key;
  if(k==="Enter"){e.preventDefault();setRange(1)}
  else if(k==="Backspace"){e.preventDefault();setRange(0)}
  else if(k==="ArrowRight"||k==="ArrowLeft"){
    const step=(e.ctrlKey||e.metaKey?10:1)*(k==="ArrowRight"?1:-1);
    if(e.shiftKey){if(anchor===null)anchor=f;go(f+step,false);selA=Math.min(anchor,f);selB=Math.max(anchor,f);render()}
    else{anchor=null;go(f+step,false)}
    e.preventDefault()}
  else if(k===" "){togglePlay();e.preventDefault()}
  else if(k==="m"||k==="M"){$("showmask").checked=!$("showmask").checked;go(f,false)}
  else if(k==="["){selA=f;if(selB<selA)selB=selA;anchor=null;render()}
  else if(k==="]"){selB=f;if(selA>selB)selA=selB;anchor=null;render()}
});
const tl=$("timeline");
const tlIdx=e=>{const r=tl.getBoundingClientRect();return Math.max(0,Math.min(B.n-1,Math.floor((e.clientX-r.left)/r.width*B.n)))};
tl.addEventListener("mousedown",e=>{if(!B)return;drag=tlIdx(e);go(drag,true)});
tl.addEventListener("mousemove",e=>{if(drag===null)return;const i=tlIdx(e);if(Math.abs(i-drag)>1){selA=Math.min(drag,i);selB=Math.max(drag,i);go(i,false)}});
addEventListener("mouseup",()=>{drag=null});
$("yes").onclick=()=>setRange(1);$("no").onclick=()=>setRange(0);
$("unrev").onclick=()=>{const a=Math.min(selA,selB),b=Math.max(selA,selB);for(let i=a;i<=b;i++)B.reviewed[i]=0;dirty=true;save()};
$("showmask").onchange=()=>go(f,false);
$("bagsel").onchange=e=>openBag(e.target.value);
addEventListener("resize",()=>B&&render());
addEventListener("beforeunload",e=>{if(dirty){e.preventDefault();e.returnValue=""}});
loadBags().then(()=>bags.length&&openBag($("bagsel").value));
</script></body></html>
"""


def main() -> None:
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path, help='bag のディレクトリ (images/ path/ pylon_masks/) が並ぶ所。シンボリックリンク可')
    ap.add_argument('--port', type=int, default=8020)
    ap.add_argument('--host', default='127.0.0.1')
    ARGS = ap.parse_args()
    print(f'http://localhost:{ARGS.port}  ({len(bag_dirs())} bag)', flush=True)
    ThreadingHTTPServer((ARGS.host, ARGS.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
