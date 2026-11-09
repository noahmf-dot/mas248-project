"""Write a standalone six-stage image-processing slideshow for a web browser.

No OpenCV HighGUI, Tkinter, Qt or ROS dependencies. The resulting HTML file
references PNG stage images in the same directory, works offline, and can be
opened after the ROS process exits.
"""
from pathlib import Path
import json
import shutil
import subprocess
import webbrowser


def write_viewer(stage_paths, stage_titles, stages_dir):
    folder = Path(stages_dir).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    filenames = [Path(p).name for p in stage_paths]
    if len(filenames) != len(stage_titles):
        raise ValueError('Stage image and title counts do not match')
    for name in filenames:
        if not (folder / name).is_file():
            raise FileNotFoundError('Stage image missing: ' + str(folder / name))
    entries = [{'file': name, 'title': title} for name, title in zip(filenames, stage_titles)]
    # JSON is transported as inert text in an application/json script element.
    data = json.dumps(entries).replace('<', '\\u003c').replace('&', '\\u0026')
    html = '''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MAS248 | Image to TCP targets</title>
<style>
body{font-family:system-ui,Arial,sans-serif;background:#17202a;color:#f7f7f7;margin:0}
main{max-width:1100px;margin:auto;padding:24px;text-align:center}
h1{font-size:26px;margin:0 0 5px}p{color:#bdc8d5}
#stage{font-size:20px;margin:18px 0 10px}
#pic{display:block;margin:10px auto;max-width:100%;max-height:68vh;object-fit:contain;background:white;border-radius:5px}
nav{display:flex;gap:14px;justify-content:center;align-items:center;padding:15px}
button{border:0;border-radius:5px;padding:12px 22px;background:#dce8ff;color:#142234;cursor:pointer;font-weight:600}
button:disabled{opacity:.35;cursor:default}#num{min-width:95px}
small{color:#bbc5d0}
</style></head><body><main><h1>MAS248 · Image → TCP targets</h1>
<p>Six processing stages. Use ← / → or the buttons. Images remain available after ROS exits.</p>
<h2 id="stage"></h2><img id="pic" alt="Image processing stage">
<nav><button id="prev" onclick="step(-1)">← Previous</button><span id="num"></span>
<button id="next" onclick="step(1)">Next →</button></nav>
<small>This is image processing and waypoint generation; no robot motion.</small>
<script id="data" type="application/json">''' + data + '''</script>
<script>
const stages=JSON.parse(document.getElementById('data').textContent);let k=0;
function render(){document.getElementById('stage').textContent=stages[k].title;
document.getElementById('pic').src=encodeURIComponent(stages[k].file);
document.getElementById('num').textContent=(k+1)+' / '+stages.length;
document.getElementById('prev').disabled=k===0;
document.getElementById('next').disabled=k===stages.length-1;}
function step(d){k=Math.max(0,Math.min(stages.length-1,k+d));render();}
document.addEventListener('keydown',e=>{if(e.key==='ArrowRight')step(1);if(e.key==='ArrowLeft')step(-1);});render();
</script></main></body></html>'''
    out = folder / 'index.html'
    out.write_text(html, encoding='utf-8')
    return out


def open_viewer(viewer_path):
    """Open the HTML viewer without blocking conversion or the ROS publisher.

    `xdg-open` is launched asynchronously on native Ubuntu. The browser is not
    managed by OpenCV and does not prevent Ctrl+C handling in ROS.
    """
    path = Path(viewer_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    opener = shutil.which('xdg-open')
    if opener:
        try:
            subprocess.Popen([opener, path.as_uri()],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
            return True
        except OSError:
            pass
    return webbrowser.open_new_tab(path.as_uri())
