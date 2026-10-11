"""Export ordinary research plots; no interactive window or robot connection."""
from pathlib import Path


def plot_training(training, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    history = training["history"]
    figure, axis = plt.subplots(figsize=(7, 4))
    axis.plot([r["epoch"] for r in history], [r["training_rmse"] for r in history], label="Train")
    axis.plot([r["epoch"] for r in history], [r["validation_rmse"] for r in history], label="Validation")
    sigmoid = training.get("rmse_space") == "sigmoid_of_aggregated_cstar_return"
    axis.set(xlabel="Epoch", ylabel="Sigmoid score RMSE" if sigmoid else "Return prediction RMSE",
             title="Supervised lattice candidate scoring" if sigmoid else "Supervised lattice return prediction")
    if training.get("model", {}).get("best_epoch"):
        axis.axvline(training["model"]["best_epoch"], color="grey", linestyle="--", label="Saved checkpoint")
    axis.legend()
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(output / "training.png", dpi=160)
    plt.close(figure)


def plot_closed_loop(evaluation, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from dreamgym.envs import Road

    identities = list(dict.fromkeys(item["scene_id"] for item in evaluation["episodes"]))
    nrows = (len(identities)+2)//3
    figure, axes = plt.subplots(nrows, 3, figsize=(13, 3.5*nrows), squeeze=False)
    for axis, identity in zip(axes.flat, identities):
        rows = [item for item in evaluation["episodes"] if item["scene_id"] == identity]
        road = Road(road_spec=rows[0]["environment_configuration"]["road_spec"])
        road.plot_road_on_matplotlib_axis(axis, cone_rng_seed=rows[0]["render_seed"])
        for row in rows:
            if row["trace"]:
                axis.plot([item["x_m"] for item in row["trace"]], [item["y_m"] for item in row["trace"]],
                          label=f"{row['selector']}: {row['steps']} steps", linewidth=2)
            else:
                axis.text(0.03, 0.93, "No feasible native candidate", transform=axis.transAxes, fontsize=8)
        if rows[0].get("initial_pose"):
            x, y = rows[0]["initial_pose"][:2]
            axis.set_xlim(x-1.2, x+1.2)
            axis.set_ylim(y-1.0, y+1.0)
        else:
            axis.set_xlim(-0.3, 2.1)
            axis.set_ylim(-0.8, 0.8)
        axis.set(title=identity, xlabel="World x (m)", ylabel="World y (m)")
        axis.set_aspect("equal", adjustable="box")
        if axis.get_legend_handles_labels()[0]:
            axis.legend(fontsize=8)
    for axis in list(axes.flat)[len(identities):]:
        axis.set_visible(False)
    figure.suptitle("Actual env execution with shared C* objective; failures are retained")
    figure.tight_layout()
    figure.savefig(Path(output) / "closed_loop.png", dpi=160)
    plt.close(figure)


def plot_road_coverage(collection, output):
    """Draw actual full traces separately per chain; expose incomplete coverage."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from dreamgym.envs import Road
    from .pipeline import collection_coverage

    identities = list(collection["base_road_splits"])
    coverage=collection_coverage(collection,require_road_end=collection['pipeline_settings'].get('until_road_end',False))
    by_id={r['scene_id']:r for r in coverage['roads']}
    nrows = (len(identities)+2)//3
    figure, axes = plt.subplots(nrows, 3, figsize=(13, 3.7*nrows), squeeze=False)
    for axis, identity in zip(axes.flat, identities):
        rows = [row for row in collection["scenes"] if row["scene_id"] == identity]
        road = Road(road_spec=rows[0]["environment_configuration"]["road_spec"])
        road.plot_road_on_matplotlib_axis(axis, cone_rng_seed=0)
        starts = list(dict.fromkeys(tuple(row["initial_pose"]) for row in rows if "initial_pose" in row))
        for number, (x, y, _) in enumerate(starts):
            axis.scatter(x, y, marker="x", s=65, color="black", zorder=5,
                         label="Initial draw" if number == 0 else None)
            axis.annotate(f"initial {number+1}", (x, y), xytext=(5, 7), textcoords="offset points", fontsize=8)
        for chain,row in enumerate(rows):
            colour=plt.get_cmap('tab10')(chain%10)
            labelled=False
            for segment in row.get("segments", []):
                if segment["accepted"]:
                    transition = segment["attempts"][-1]
                    points=[transition['start_state'][:2],*[[p['x_m'],p['y_m']] for p in transition.get('trace',[])]]
                    if len(points)<2:continue
                    ending=('GOAL' if row.get('road_completed') else 'STOP' if row.get('terminal_behavior')=='planned_stop_reference_complete' else 'FAIL' if row.get('faults') else 'BUDGET')
                    index=row['episode_id'].split(':')[3]
                    label=f"I{int(index)+1}/{row['lattice_profile']}: {row['decisions']} groups, {ending}"
                    axis.plot([p[0] for p in points],[p[1] for p in points],color=colour,linewidth=1.5,
                              alpha=.8,label=label if not labelled else None,zorder=3)
                    axis.scatter(points[-1][0],points[-1][1],color=colour,s=18,zorder=4)
                    labelled=True
        rejected = sum(row["decisions"] == 0 for row in rows)
        stats=by_id[identity];seed=rows[0]['episode_id'].split(':')[1]
        axis.set_title(f"{rows[0]['split']} seed {seed}: {rows[0]['road_length_m']:.1f}m; {len(rows)} chains\n"
                       f"{stats['decisions']} groups; {stats['visited_fraction']:.1%} visited; {rejected} empty; {stats['completed_sequences']} goals", fontsize=9)
        axis.set(xlabel="World x (m)", ylabel="World y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.2)
        axis.legend(fontsize=6.5)
    for axis in list(axes.flat)[len(identities):]:
        axis.set_visible(False)
    figure.suptitle("Actual C* traces by random chain; empty and incomplete road coverage remain visible")
    figure.tight_layout()
    figure.savefig(Path(output) / "road_coverage.png", dpi=160)
    plt.close(figure)
