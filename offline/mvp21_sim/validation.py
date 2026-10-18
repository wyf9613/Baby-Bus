"""One entry: init roads/config, run current modules, index evidence, serve/watch, package."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__);commands=parser.add_subparsers(dest='command',required=True)
    default=Path('offline/mvp21_sim/workspace')
    for name in ('init','run','index','serve','add-road'):
        sub=commands.add_parser(name);sub.add_argument('--workspace',type=Path,default=default)
        if name=='init':
            sub.add_argument('--empty',action='store_true');sub.add_argument('--no-history',action='store_true')
        if name=='run':
            sub.add_argument('--road',action='append',default=[]);sub.add_argument('--all',action='store_true')
            sub.add_argument('--config',type=Path);sub.add_argument('--max-steps',type=int)
        if name=='serve':
            sub.add_argument('--port',type=int,default=8792);sub.add_argument('--no-watch',action='store_true')
        if name=='add-road':sub.add_argument('path',type=Path)
    sub=commands.add_parser('package');sub.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from . import validation_workspace as workspace
    if args.command=='init':value=workspace.initialize(args.workspace,seed_roads=not args.empty,import_history=not args.no_history)
    elif args.command=='add-road':value=dict(path=str(workspace.ingest(args.workspace,args.path)))
    elif args.command=='index':value=workspace.index(args.workspace);workspace.write(args.workspace/'index.json',value)
    elif args.command=='run':
        value=workspace.run(args.workspace,selected=args.road,all_roads=args.all,profile_path=args.config,max_steps=args.max_steps)
        value=dict(runs=len(value),completed=sum(r['case']['summary']['completed'] for r in value))
    elif args.command=='serve':
        from .validation_server import serve
        serve(args.workspace,args.port,not args.no_watch);return
    else:
        from .validation_package import package
        value=package(args.output)
    print(json.dumps(value if args.command not in ('init','index') else dict(cases=len(value['cases']),errors=value['errors']),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
