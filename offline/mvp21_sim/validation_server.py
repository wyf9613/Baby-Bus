"""Read-only replay server; optional local watcher runs the normal validation CLI."""
from functools import partial
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.parse import unquote,urlsplit

from .validation_workspace import ROOT,index,read,write,source_snapshot,digest


def watched_fingerprint(workspace):
    paths=[workspace/'validation.json',*sorted((workspace/'roads').rglob('*.json'))]
    profile=read(workspace/'validation.json');model=Path(profile.get('model_path',''))
    if not model.is_absolute():model=ROOT/model
    if model.is_file():paths.append(model)
    request=read(workspace/'last-request.json') if (workspace/'last-request.json').exists() else {}
    if request.get('profile_path') and Path(request['profile_path']).is_file():paths.append(Path(request['profile_path']))
    return digest(dict(source=source_snapshot()['sha256'],inputs={str(p):p.read_bytes().hex() for p in paths}))


def watch(workspace,stop):
    last=watched_fingerprint(workspace);child=None;pending=None;changed_at=None;affected=[]
    previous_roads={p.stem:p.read_bytes() for p in (workspace/'roads').rglob('*.json')}
    log=None
    try:
        while not stop.wait(.5):
            try:current=watched_fingerprint(workspace)
            except Exception as error:
                write(workspace/'watch-status.json',dict(state='invalid_input',error=str(error)));continue
            if current!=last:
                last=current;pending=current;changed_at=time.monotonic()
                current_roads={p.stem:p.read_bytes() for p in (workspace/'roads').rglob('*.json')}
                affected=list(set(affected)|{key for key,value in current_roads.items() if previous_roads.get(key)!=value})
                previous_roads=current_roads
            if child is not None and child.poll() is not None:
                write(workspace/'watch-status.json',dict(state='idle' if child.returncode==0 else 'error',exit_code=child.returncode))
                child=None
                if log:log.close();log=None
            if pending and child is None and time.monotonic()-changed_at>=1.:
                request=read(workspace/'last-request.json') if (workspace/'last-request.json').exists() else {}
                cmd=[sys.executable,'-m','offline.mvp21_sim.validation','run','--workspace',str(workspace)]
                for road in (affected or request.get('selected',[])):cmd+=['--road',road]
                if not affected and request.get('all_roads'):cmd+=['--all']
                if request.get('max_steps') is not None:cmd+=['--max-steps',str(request['max_steps'])]
                if request.get('profile_path'):cmd+=['--config',request['profile_path']]
                log=(workspace/'watch-run.log').open('a',encoding='utf-8')
                child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                write(workspace/'watch-status.json',dict(state='running',input_revision=pending,pid=child.pid))
                pending=None;affected=[]
    finally:
        if child is not None and child.poll() is None:child.terminate();child.wait(timeout=15)
        if log:log.close()


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        path=unquote(urlsplit(self.path).path)
        if path=='/api/index':
            data=index(Path(self.directory));status=Path(self.directory)/'watch-status.json'
            data['watch']=read(status) if status.exists() else dict(state='manual')
            return self.send_bytes(json.dumps(data,ensure_ascii=False).encode(),'application/json; charset=utf-8')
        if path in ('/','/viewer.html'):
            data=index(Path(self.directory))
            html=(ROOT/'offline/mvp21_sim/runtime_viewer.html').read_text(encoding='utf-8')
            html=html.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</','<\\/'))
            return self.send_bytes(html.encode(),'text/html; charset=utf-8')
        if path in ('/KNOWN_LIMITATIONS_CN.md','/workflow'):
            target=ROOT/('docs/SIMULATION_LIMITATIONS_CN.md' if path.endswith('.md') else 'offline/mvp21_sim/VALIDATION_WORKFLOW_CN.md')
            return self.send_bytes(target.read_bytes(),'text/plain; charset=utf-8')
        if path.startswith('/archive/'):
            root=ROOT/'offline/lattice_training/artifacts';target=(root/path.removeprefix('/archive/')).resolve()
            try:target.relative_to(root.resolve())
            except ValueError:return self.send_error(403)
            if target.is_file():return self.send_bytes(target.read_bytes(),self.guess_type(str(target)))
            return self.send_error(404)
        return super().do_GET()

    def send_bytes(self,data,mime):
        self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)


def serve(workspace,port=8792,watch_enabled=True):
    workspace=Path(workspace).resolve();stop=threading.Event();thread=None
    if watch_enabled:
        write(workspace/'watch-status.json',dict(state='idle'))
        thread=threading.Thread(target=watch,args=(workspace,stop),daemon=True);thread.start()
    server=ThreadingHTTPServer(('127.0.0.1',port),partial(Handler,directory=str(workspace)))
    write(workspace/'server-process.json',dict(pid=os.getpid(),port=port,workspace=str(workspace)))
    print(f'http://127.0.0.1:{port}/viewer.html | watch={watch_enabled}',flush=True)
    try:server.serve_forever()
    finally:
        stop.set();server.server_close()
        if thread:thread.join(timeout=20)
