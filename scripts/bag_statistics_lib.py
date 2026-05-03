"""Offline bag metrics: mode, visibility, safety distance, control smoothness.

Shared by `statistics.py` (single run) and `statistics_all.py` (all runs).
Requires messages on `/target_nav_mode`, `/target_visible`, `/scan`, `/cmd_vel`.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')

import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib as mpl
except ImportError as exc:
    sys.stderr.write('matplotlib is required: pip install matplotlib\n')
    raise

try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except ImportError as exc:
    sys.stderr.write('rosbags is required: pip install rosbags\n')
    raise


@dataclass
class Config:
    # I/O
    project_root: Path = Path(__file__).resolve().parents[1]
    bag_root_rel: str = 'logs/rosbag'
    output_root_rel: str = 'logs/stats'
    stats_all_runs_rel: str = 'logs/stats/all_runs'

    rosbag_default_typestore: Stores = Stores.LATEST

    topic_mode: str = '/target_nav_mode'
    topic_visible: str = '/target_visible'
    topic_cmd_vel: str = '/cmd_vel'
    topic_scan: str = '/scan'

    d_collision_m: float = 0.3
    d_potential_m: float = 0.5
    event_release_seconds: float = 0.5

    figure_dpi: int = 140
    figure_size: Tuple[float, float] = (10.0, 5.0)
    font_size: int = 12

    mode_labels_ru: dict = field(default_factory=lambda: {
        'Chase target': 'Сопровождение цели',
        "Go to last target's pose": 'К последней позиции цели',
        'Spining': 'Поиск',
    })
    mode_palette: dict = field(default_factory=lambda: {
        'Chase target': '#2ca02c',
        "Go to last target's pose": '#ff7f0e',
        'Spining': '#1f77b4',
    })


CFG = Config()


def _ns_to_s(ns: int, t0_ns: int) -> float:
    return (ns - t0_ns) * 1e-9


def _is_valid_rosbag2_dir(path: Path) -> bool:
    return path.is_dir() and (path / 'metadata.yaml').is_file()


def _bag_sort_key(path: Path) -> Tuple[int, int, str]:
    """Sort runs: ``run1``, ``run2``, … (numeric) first; then other dirs (e.g. ``run_YYYYMMDD_…``)."""
    m = re.match(r'^run(\d+)$', path.name)
    if m:
        return (0, int(m.group(1)), '')
    return (1, 0, path.name)


def _resolve_bag_path(arg_path: Optional[str]) -> Path:
    if arg_path:
        p = Path(arg_path)
        if not p.is_absolute():
            p = CFG.project_root / p
        if not p.exists():
            raise FileNotFoundError(f'Bag path does not exist: {p}')
        if not _is_valid_rosbag2_dir(p):
            hint = (
                f'There is no metadata.yaml in {p}. '
                'Rosbag2 writes this file when recording stops cleanly; '
                'if the container was killed or recording crashed, the folder '
                'may be empty/incomplete — remove it or pick another run.'
            )
            raise FileNotFoundError(hint)
        return p
    bag_root = CFG.project_root / CFG.bag_root_rel
    if not bag_root.exists():
        raise FileNotFoundError(
            f'Bag root not found: {bag_root}. Run the simulation first.'
        )
    candidates = [p for p in bag_root.iterdir() if _is_valid_rosbag2_dir(p)]
    if not candidates:
        any_dirs = [p for p in bag_root.iterdir() if p.is_dir()]
        if any_dirs:
            raise FileNotFoundError(
                f'No complete rosbag2 under {bag_root} '
                f'(found {len(any_dirs)} folder(s) without metadata.yaml). '
                'After make run, detach with Ctrl+b d so the container can stop '
                'the recorder and write metadata.yaml; or stop ros2 bag record '
                'with Ctrl+C in the bag pane.'
            )
        raise FileNotFoundError(f'No bag runs found under {bag_root}.')
    # Latest finished recording: ``run_*`` from interactive ``make run`` or ``runN`` from ``make stats``.
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _ensure_output_dir(bag_path: Path) -> Path:
    out_dir = CFG.project_root / CFG.output_root_rel / bag_path.name
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


@dataclass
class BagData:
    t0_ns: int = 0
    t_end_s: float = 0.0
    mode_t: np.ndarray = field(default_factory=lambda: np.array([]))
    mode_v: List[str] = field(default_factory=list)
    vis_t: np.ndarray = field(default_factory=lambda: np.array([]))
    vis_v: np.ndarray = field(default_factory=lambda: np.array([]))
    cmd_t: np.ndarray = field(default_factory=lambda: np.array([]))
    cmd_vx: np.ndarray = field(default_factory=lambda: np.array([]))
    cmd_vy: np.ndarray = field(default_factory=lambda: np.array([]))
    cmd_wz: np.ndarray = field(default_factory=lambda: np.array([]))
    scan_t: np.ndarray = field(default_factory=lambda: np.array([]))
    scan_min_r: np.ndarray = field(default_factory=lambda: np.array([]))


def validate_bag_for_plots(data: BagData, bag_label: str) -> None:
    """Fail fast if a bag cannot produce the standard plot set."""
    if data.mode_t.size == 0:
        raise RuntimeError(f'{bag_label}: bag has no messages on {CFG.topic_mode}')
    if data.vis_t.size == 0:
        raise RuntimeError(f'{bag_label}: bag has no messages on {CFG.topic_visible}')
    if data.scan_t.size == 0:
        raise RuntimeError(f'{bag_label}: bag has no messages on {CFG.topic_scan}')
    if data.cmd_t.size < 2:
        raise RuntimeError(f'{bag_label}: need at least 2 samples on {CFG.topic_cmd_vel}')


def _read_bag(bag_path: Path) -> BagData:
    data = BagData()

    mode_t, mode_v = [], []
    vis_t, vis_v = [], []
    cmd_t, cmd_vx, cmd_vy, cmd_wz = [], [], [], []
    scan_t, scan_min_r = [], []

    with AnyReader(
        [bag_path],
        default_typestore=get_typestore(CFG.rosbag_default_typestore),
    ) as reader:
        if reader.start_time:
            data.t0_ns = reader.start_time
        wanted = {
            CFG.topic_mode,
            CFG.topic_visible,
            CFG.topic_cmd_vel,
            CFG.topic_scan,
        }
        conns = [c for c in reader.connections if c.topic in wanted]
        for connection, ts_ns, raw in reader.messages(connections=conns):
            t = _ns_to_s(ts_ns, data.t0_ns)
            topic = connection.topic
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue

            if topic == CFG.topic_mode:
                mode_t.append(t)
                mode_v.append(str(msg.data))
            elif topic == CFG.topic_visible:
                vis_t.append(t)
                vis_v.append(1.0 if msg.data else 0.0)
            elif topic == CFG.topic_cmd_vel:
                cmd_t.append(t)
                cmd_vx.append(msg.linear.x)
                cmd_vy.append(msg.linear.y)
                cmd_wz.append(msg.angular.z)
            elif topic == CFG.topic_scan:
                arr = np.asarray(msg.ranges, dtype=float)
                finite = arr[np.isfinite(arr) & (arr > 0.0)]
                if finite.size:
                    scan_t.append(t)
                    scan_min_r.append(float(np.min(finite)))

    data.mode_t = np.asarray(mode_t)
    data.mode_v = mode_v
    data.vis_t = np.asarray(vis_t)
    data.vis_v = np.asarray(vis_v)
    data.cmd_t = np.asarray(cmd_t)
    data.cmd_vx = np.asarray(cmd_vx)
    data.cmd_vy = np.asarray(cmd_vy)
    data.cmd_wz = np.asarray(cmd_wz)
    data.scan_t = np.asarray(scan_t)
    data.scan_min_r = np.asarray(scan_min_r)

    all_ts = [a for a in (
        data.mode_t, data.vis_t, data.cmd_t, data.scan_t,
    ) if a.size]
    if all_ts:
        data.t_end_s = float(max(a[-1] for a in all_ts))
    return data


def _setup_mpl():
    mpl.rcParams.update({
        'font.size': CFG.font_size,
        'font.family': 'serif',
        'font.serif': ['DejaVu Serif', 'STIXGeneral', 'Times New Roman', 'Times'],
        'mathtext.fontset': 'stix',
        'mathtext.default': 'it',
        'axes.unicode_minus': False,
        'axes.titlesize': CFG.font_size + 1,
        'axes.labelsize': CFG.font_size,
        'legend.fontsize': CFG.font_size - 1,
        'figure.dpi': CFG.figure_dpi,
        'savefig.dpi': CFG.figure_dpi,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
    })


def _save(fig, out_dir: Path, name: str):
    path = out_dir / f'{name}.png'
    fig.savefig(path)
    plt.close(fig)
    print(f'  saved: {path.relative_to(CFG.project_root)}')


def plot_mode_timeline(data: BagData, out_dir: Path) -> dict:
    t = data.mode_t
    v = data.mode_v
    intervals = []
    for i in range(len(v)):
        t_start = t[i]
        t_end = t[i + 1] if i + 1 < len(v) else max(t[-1], data.t_end_s)
        intervals.append((t_start, t_end, v[i]))

    unique_modes = list(dict.fromkeys(v))
    rank = {m: i for i, m in enumerate(unique_modes)}
    durations = {m: 0.0 for m in unique_modes}
    for ts, te, m in intervals:
        durations[m] += max(0.0, te - ts)

    fig, ax = plt.subplots(figsize=CFG.figure_size)
    for ts, te, m in intervals:
        color = CFG.mode_palette.get(m, '#808080')
        ax.barh(rank[m], te - ts, left=ts, color=color, edgecolor='none', height=0.7)

    ax.set_yticks(list(rank.values()))
    ax.set_yticklabels([CFG.mode_labels_ru.get(m, m) for m in unique_modes])
    ax.set_xlabel(r'$t$, с')
    ax.set_title('Временная диаграмма режимов сопровождения')
    ax.set_xlim(0, max(data.t_end_s, intervals[-1][1]))
    fig.tight_layout()
    _save(fig, out_dir, '01_mode_timeline')

    total = sum(durations.values()) or 1.0
    return {
        'total_duration_s': float(total),
        'mode_share': {CFG.mode_labels_ru.get(m, m): durations[m] / total
                       for m in unique_modes},
    }


def plot_visibility(data: BagData, out_dir: Path) -> dict:
    t = data.vis_t
    v = data.vis_v
    if t.size >= 2:
        dt = np.diff(t, append=t[-1])
        ratio = float(np.sum(v[:-1] * dt[:-1]) / max(np.sum(dt[:-1]), 1e-9))
    else:
        ratio = float(v.mean())

    fig, ax = plt.subplots(figsize=CFG.figure_size)
    ax.step(t, v, where='post', color='#1f77b4', linewidth=1.5)
    ax.fill_between(t, 0, v, step='post', alpha=0.25, color='#1f77b4')
    ax.set_ylim(-0.1, 1.1)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(['Вне кадра', 'В кадре'])
    ax.set_xlabel(r'$t$, с')
    ax.set_title(
        'Видимость цели $\\nu(t)$; $\\bar{\\nu}=%.1f\\%%$' % (ratio * 100.0)
    )
    fig.tight_layout()
    _save(fig, out_dir, '02_visibility')

    return {'visibility_ratio': ratio}


def _count_events(t: np.ndarray, below: np.ndarray, release_s: float) -> int:
    if t.size == 0:
        return 0
    events = 0
    in_event = False
    last_below_t = -np.inf
    for ti, b in zip(t, below):
        if b:
            if not in_event:
                events += 1
                in_event = True
            last_below_t = ti
        else:
            if in_event and (ti - last_below_t) >= release_s:
                in_event = False
    return events


def _scan_after_first_clear(t: np.ndarray, r: np.ndarray, d_clear_m: float) -> Tuple[np.ndarray, np.ndarray]:
    if t.size == 0:
        return t, r
    first_clear = np.flatnonzero(r > d_clear_m)
    if first_clear.size == 0:
        return np.array([]), np.array([])
    s = int(first_clear[0])
    return t[s:], r[s:]


def plot_safety(data: BagData, out_dir: Path) -> dict:
    t = data.scan_t
    r = data.scan_min_r
    t_c, r_c = _scan_after_first_clear(t, r, CFG.d_potential_m)
    coll_mask = r_c <= CFG.d_collision_m
    pot_mask = r_c <= CFG.d_potential_m

    n_coll = _count_events(t_c, coll_mask, CFG.event_release_seconds)
    n_pot = _count_events(t_c, pot_mask, CFG.event_release_seconds)
    n_prev = max(0, n_pot - n_coll)
    ratio_prev = (n_prev / n_pot) if n_pot > 0 else 1.0

    t_plot, r_plot = _scan_after_first_clear(t, r, CFG.d_collision_m)

    fig, ax = plt.subplots(figsize=CFG.figure_size)
    ax.plot(t_plot, r_plot, color='#17becf', linewidth=1.0,
            label=r'$d^{\mathrm{lid}}_{\min}(t)$')
    ax.axhline(CFG.d_potential_m, color='#ff7f0e', linestyle='--',
               label=rf'$d_\mathrm{{pot}}={CFG.d_potential_m:.2f}\,$м')
    ax.axhline(CFG.d_collision_m, color='#d62728', linestyle='--',
               label=rf'$d_\mathrm{{col}}={CFG.d_collision_m:.2f}\,$м')
    ax.set_xlabel(r'$t$, с')
    ax.set_ylabel(r'$d_{\min}^{\mathrm{obs}}$, м')
    ax.set_title(
        r'Безопасность: '
        rf'$N_\mathrm{{c}}={n_coll}$, $N_\mathrm{{r}}={n_pot}$, '
        rf'$N_\mathrm{{p}}={n_prev}$, $\rho={ratio_prev:.2f}$'
    )
    ax.legend(loc='upper right')
    fig.tight_layout()
    _save(fig, out_dir, '03_safety_distance')

    return {
        'collisions_count': int(n_coll),
        'potential_conflicts_count': int(n_pot),
        'prevented_count': int(n_prev),
        'prevented_ratio': float(ratio_prev),
    }


def plot_control_smoothness(data: BagData, out_dir: Path) -> dict:
    u = np.stack([data.cmd_vx, data.cmd_vy, data.cmd_wz], axis=1)
    du = np.diff(u, axis=0)
    du_norm = np.linalg.norm(du, axis=1)
    rms = float(np.sqrt(np.mean(du_norm ** 2)))
    mean_abs = float(du_norm.mean())
    peak = float(du_norm.max())

    fig, ax = plt.subplots(figsize=CFG.figure_size)
    ax.plot(data.cmd_t[1:], du_norm, color='#2ca02c', linewidth=1.0)
    ax.axhline(rms, color='#333333', linestyle='--',
               label=r'$\mathrm{RMS} = %.3f$' % rms)
    ax.set_xlabel(r'$t$, с')
    ax.set_ylabel(r'$\|\Delta\mathbf{u}_k\|_2 = \|\mathbf{u}_k - \mathbf{u}_{k-1}\|_2$')
    ax.set_title(r'Плавность управления')
    ax.legend(loc='upper right')
    fig.tight_layout()
    _save(fig, out_dir, '04_control_smoothness')

    return {
        'smoothness_rms': rms,
        'smoothness_mean_abs': mean_abs,
        'smoothness_peak': peak,
    }


def aggregate_metrics(data: BagData) -> dict:
    """Scalar metrics for multi-run summaries (no figure IO)."""
    m: dict = {}
    t = data.mode_t
    v = data.mode_v
    intervals = []
    for i in range(len(v)):
        t_start = t[i]
        t_end = t[i + 1] if i + 1 < len(v) else max(t[-1], data.t_end_s)
        intervals.append((t_start, t_end, v[i]))
    unique_modes = list(dict.fromkeys(v))
    durations = {mode: 0.0 for mode in unique_modes}
    for ts, te, mode in intervals:
        durations[mode] += max(0.0, te - ts)
    total = sum(durations.values()) or 1.0
    m['mode_timeline'] = {
        'total_duration_s': float(total),
        'mode_share': {
            CFG.mode_labels_ru.get(x, x): durations[x] / total
            for x in unique_modes
        },
    }

    vt = data.vis_t
    vv = data.vis_v
    if vt.size >= 2:
        dt = np.diff(vt, append=vt[-1])
        ratio = float(np.sum(vv[:-1] * dt[:-1]) / max(np.sum(dt[:-1]), 1e-9))
    else:
        ratio = float(vv.mean())
    m['visibility'] = {'visibility_ratio': ratio}

    st = data.scan_t
    sr = data.scan_min_r
    st_c, sr_c = _scan_after_first_clear(st, sr, CFG.d_potential_m)
    coll_mask = sr_c <= CFG.d_collision_m
    pot_mask = sr_c <= CFG.d_potential_m
    n_coll = _count_events(st_c, coll_mask, CFG.event_release_seconds)
    n_pot = _count_events(st_c, pot_mask, CFG.event_release_seconds)
    n_prev = max(0, n_pot - n_coll)
    ratio_prev = (n_prev / n_pot) if n_pot > 0 else 1.0
    m['safety'] = {
        'collisions_count': int(n_coll),
        'potential_conflicts_count': int(n_pot),
        'prevented_count': int(n_prev),
        'prevented_ratio': float(ratio_prev),
    }

    u = np.stack([data.cmd_vx, data.cmd_vy, data.cmd_wz], axis=1)
    du = np.diff(u, axis=0)
    du_norm = np.linalg.norm(du, axis=1)
    m['control_smoothness'] = {
        'smoothness_rms': float(np.sqrt(np.mean(du_norm ** 2))),
        'smoothness_mean_abs': float(du_norm.mean()),
        'smoothness_peak': float(du_norm.max()),
    }
    return m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--bag', type=str, default=None,
        help='Rosbag2 directory; default = latest complete bag under logs/rosbag (max mtime: run_… or runN).',
    )
    args = parser.parse_args()

    bag_path = _resolve_bag_path(args.bag)
    out_dir = _ensure_output_dir(bag_path)
    print(f'Bag:    {bag_path}')
    print(f'Output: {out_dir}')

    _setup_mpl()
    data = _read_bag(bag_path)
    validate_bag_for_plots(data, str(bag_path))

    metrics = {}
    metrics['mode_timeline'] = plot_mode_timeline(data, out_dir)
    metrics['visibility'] = plot_visibility(data, out_dir)
    metrics['safety'] = plot_safety(data, out_dir)
    metrics['control_smoothness'] = plot_control_smoothness(data, out_dir)


if __name__ == '__main__':
    main()
