"""Evaluate a road with highest-score lattice selection and inspect its candidates.

python -m offline.lattice_training.visualize --road route.json --output evaluation --serve
The optional local viewer supports clicking road positions to request candidates.
"""
import argparse
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import math
from pathlib import Path
from threading import Lock

import numpy as np

from .cstar import SolverSettings
from .dataset import write_json, json_safe
from .evaluation import RoadEvaluation,road_environment
from .model import SpatialPathPolicy
from .objective import CStarTrackingSettings
from .roads import generate_random_road_spec,RoadRandomizationSettings,obstacle_passage_report,road_length


def scene_data(spec):
    from dreamgym.envs import Road
    road=Road(road_spec=spec)
    progression=np.linspace(0,road_length(spec),max(2,int(road_length(spec)/.05)+1))
    centers,headings=road.convert_progression_to_coordinates(progression)
    bounds=road.get_reference_lane_bounds_at_progresses(progression)
    normal=np.column_stack((-np.sin(headings),np.cos(headings)))
    return {'center':centers.tolist(),'left':(centers+normal*np.asarray(bounds['left_bound_m'])[:,None]).tolist(),
            'right':(centers+normal*np.asarray(bounds['right_bound_m'])[:,None]).tolist(),
            'progression':progression.tolist(),'obstacles':road.get_obstacle_polygons().tolist()}


def plot_evaluation(report,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from dreamgym.envs import Road
    figure,axes=plt.subplots(1,2,figsize=(13,5),gridspec_kw={'width_ratios':[2,1]})
    axis=axes[0]; Road(road_spec=report['road_spec']).plot_road_on_matplotlib_axis(axis,cone_rng_seed=report['seed'])
    for segment in report['segments']:
        p=np.asarray(segment['reference_points']);axis.plot(p[:,0],p[:,1],color='steelblue',alpha=.45,linewidth=1)
    trace=report['trace']
    if trace:
        axis.plot([r['x_m'] for r in trace],[r['y_m'] for r in trace],color='#e45618',linewidth=2,label='Actual C* trace')
    axis.scatter(*report['initial_pose'][:2],marker='x',color='black',label='Start')
    for i,p in enumerate(report.get('inspection_panels',[])):
        axis.scatter(*p['pose'][:2],color='purple',s=30);axis.annotate(str(i+1),p['pose'][:2])
    axis.set_aspect('equal',adjustable='box');axis.set(xlabel='World x (m)',ylabel='World y (m)')
    axis.set_title(f"{report['final_progress_m']:.2f}/{report['road_length_m']:.2f}m | {report['completion'] or report['fault']}",fontsize=10)
    axis.legend(fontsize=8);axis.grid(alpha=.15)
    if trace:
        t=[r['timestamp_s']-trace[0]['timestamp_s'] for r in trace]
        axes[1].plot(t,[r['speed_mps'] for r in trace],color='#e45618')
    axes[1].set(xlabel='Simulation time (s)',ylabel='Speed (m/s)',title='Actual speed')
    axes[1].grid(alpha=.2);figure.tight_layout();figure.savefig(Path(output)/'evaluation.png',dpi=160);plt.close(figure)
    panels=report.get('inspection_panels',[])
    if panels:
        columns=min(3,len(panels));nrows=math.ceil(len(panels)/columns)
        figure,axes=plt.subplots(nrows,columns,figsize=(5*columns,4*nrows),squeeze=False)
        for axis,panel in zip(axes.flat,panels):
            Road(road_spec=report['road_spec']).plot_road_on_matplotlib_axis(axis,cone_rng_seed=report['seed'])
            for c in panel['candidates']:
                p=np.asarray(c['points'])
                axis.plot(p[:,0],p[:,1],color='green' if c['selected'] else 'steelblue' if c['admitted'] else '#cb5964',
                          linestyle='-' if c['admitted'] else ':',linewidth=2 if c['selected'] else .7,alpha=.8)
            x,y=panel['pose'][:2];axis.scatter(x,y,color='purple',s=35)
            points=np.asarray([[x,y],*[p for c in panel['candidates'] for p in c['points']]])
            axis.set_xlim(points[:,0].min()-.5,points[:,0].max()+.5)
            axis.set_ylim(points[:,1].min()-.5,points[:,1].max()+.5);axis.set_aspect('equal',adjustable='box')
            axis.set_title(f"s={panel['progress_m']:.2f}m: {panel['admitted_count']} admitted / {panel['rejected_count']} rejected",fontsize=9)
        for axis in list(axes.flat)[len(panels):]: axis.set_visible(False)
        figure.tight_layout();figure.savefig(Path(output)/'candidates.png',dpi=160);plt.close(figure)


def save_viewer(report,output):
    template=Path(__file__).with_name('viewer.html').read_text(encoding='utf-8')
    data=json.dumps(json_safe(report),ensure_ascii=False,allow_nan=False).replace('</','<\\/')
    Path(output,'evaluation.html').write_text(template.replace('__REPORT_JSON__',data),encoding='utf-8')


def serve(evaluator,report,output,port):
    lock=Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            if self.path.split('?')[0] not in ('/','/evaluation.html'):
                self.send_error(404);return
            data=Path(output,'evaluation.html').read_bytes()
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8')
            self.end_headers();self.wfile.write(data)
        def do_POST(self):
            with lock: self._post()
        def _post(self):
            if self.path!='/candidates': self.send_error(404);return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<10000: raise ValueError('Invalid request size')
                query=json.loads(self.rfile.read(size))
                p=float(query['progress_m'])
                speed=float(query.get('speed',.2));offset=float(query.get('offset',0.));heading=float(query.get('heading',0.))
                if not np.isfinite([p,speed,offset,heading]).all() or not 0<=speed<=.5 or abs(offset)>.4 or abs(heading)>.5:
                    raise ValueError('Point state is outside the configured range')
                panel=evaluator.probe(p,speed=speed,offset=offset,heading_error=heading,use_trace=query.get('use_trace',True) is True)
                report['inspection_panels'].append(panel)
                write_json(Path(output,'evaluation.json'),report);save_viewer(report,output)
                data=json.dumps(panel,ensure_ascii=False).encode()
                self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8')
                self.end_headers();self.wfile.write(data)
            except (ValueError,KeyError,TypeError) as error:
                self.send_error(400,str(error))
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    print(json.dumps({'stage':'viewer_ready','url':f'http://127.0.0.1:{server.server_port}/'}),flush=True)
    try: server.serve_forever()
    finally: server.server_close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--road',type=Path,help='Raw road_spec JSON or an exported environment configuration')
    parser.add_argument('--road-seed',type=int,default=0)
    parser.add_argument('--road-settings',type=Path)
    parser.add_argument('--model',type=Path,help='Full-path policy checkpoint; omitted uses native rule score')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',choices=('standard','conservative','wide'),default='wide')
    parser.add_argument('--cruise-speed',type=float,default=.3,help='Desired native planner speed within vehicle limits')
    parser.add_argument('--start-progress',type=float,default=.25)
    parser.add_argument('--initial-speed',type=float,default=.2)
    parser.add_argument('--inspect-progress',default='1.0,4.0')
    parser.add_argument('--max-segments',type=int,default=100)
    parser.add_argument('--population',type=int,default=12);parser.add_argument('--iterations',type=int,default=4)
    parser.add_argument('--max-evaluations',type=int,default=64);parser.add_argument('--solver-wall-s',type=float,default=10)
    parser.add_argument('--serve',action='store_true');parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    if args.road:
        data=json.loads(args.road.read_text(encoding='utf-8'));spec=data.get('road_spec',data)
    else:
        settings=RoadRandomizationSettings(**json.loads(args.road_settings.read_text(encoding='utf-8'))) if args.road_settings else None
        spec=generate_random_road_spec(args.road_seed,settings)
    model=SpatialPathPolicy.load(args.model) if args.model else None
    solver=SolverSettings(population=args.population,iterations=args.iterations,max_evaluations=args.max_evaluations,max_wall_s=args.solver_wall_s)
    args.output.mkdir(parents=True,exist_ok=True);write_json(args.output/'road.json',spec)
    env=road_environment(spec,args.road_seed,progress=args.start_progress,speed=args.initial_speed)
    evaluator=RoadEvaluation(env,model=model,profile=args.profile,solver_settings=solver,seed=args.road_seed,
                             cruise_speed=args.cruise_speed)
    try:
        report=evaluator.run(max_segments=args.max_segments,progress=lambda r:print(json.dumps(r),flush=True))
        report['scene']=scene_data(spec)
        report['inspection_panels']=[evaluator.probe(float(p)) for p in args.inspect_progress.split(',') if p.strip()]
        report['solver_settings']=asdict(solver)
        report['passage']=obstacle_passage_report(spec) if spec.get('obstacles') else None
        report['source_model']=str(args.model) if args.model else None
        write_json(args.output/'evaluation.json',report);plot_evaluation(report,args.output);save_viewer(report,args.output)
        print(json.dumps({'stage':'evaluation_complete','road_completed':report['road_completed'],'fault':report['fault'],
                          'trace_points':len(report['trace']),'wall_s':report['wall_s'],'output':str(args.output)}),flush=True)
        if args.serve: serve(evaluator,report,args.output,args.port)
    finally: env.close()


if __name__=='__main__': main()
