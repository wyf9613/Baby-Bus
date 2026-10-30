"""Replay only recorded executed branches; all candidate scores share one state."""
import numpy as np

SEQUENCE_FIELDS=('episode_ids','sequence_steps','behavior_selected')


def sequence_layout(groups,context):
    if context is None or not set(SEQUENCE_FIELDS)<=context.keys():
        raise ValueError('Recurrent training requires recorded episode/step/executed-candidate fields')
    n=len(groups)
    if any(np.asarray(context[k]).shape!=(n,) for k in SEQUENCE_FIELDS):raise ValueError('Invalid recurrent row arrays')
    episodes={}
    for group in dict.fromkeys(groups):
        rows=np.flatnonzero(groups==group)
        identities=set(context['episode_ids'][rows]);steps=set(context['sequence_steps'][rows])
        if len(identities)!=1 or len(steps)!=1:raise ValueError('Decision crosses recurrent episode or step')
        selected=rows[np.asarray(context['behavior_selected'][rows],dtype=bool)]
        if len(selected)>1:raise ValueError('Decision has multiple executed candidates')
        episodes.setdefault(next(iter(identities)),[]).append((int(next(iter(steps))),rows,
                                                             int(selected[0]) if len(selected) else None))
    layout=[]
    for decisions in episodes.values():
        decisions.sort(key=lambda d:d[0])
        if [d[0] for d in decisions]!=list(range(len(decisions))):
            raise ValueError('Recurrent episode must start at zero and have contiguous decisions')
        if any(d[2] is None for d in decisions[:-1]):raise ValueError('Missing executed branch before next decision')
        layout.append([(rows,selected) for _,rows,selected in decisions])
    return layout


def forward_sequence(network,matrices,layout):
    outputs=[None]*len(matrices)
    for decisions in layout:
        hidden=np.zeros(network.hidden)
        for rows,selected in decisions:
            for row in rows:outputs[row]=network.forward(matrices[row],hidden)
            if selected is not None:hidden=outputs[selected][1][7].copy()
    return outputs


def backward_sequence(network,outputs,derivatives,layout):
    gradients={name:np.zeros_like(value) for name,value in network.parameters.items()}
    for decisions in layout:
        future=np.zeros(network.hidden)
        for rows,selected in reversed(decisions):
            previous=np.zeros(network.hidden)
            for row in rows:
                values,dh=network.backward(outputs[row][1],derivatives[row],
                    future if row==selected else None,return_hidden_gradient=True)
                previous+=dh
                for name,value in values.items():gradients[name]+=value
            future=previous
    return gradients
