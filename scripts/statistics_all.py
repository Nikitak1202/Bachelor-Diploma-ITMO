#!/usr/bin/env python3
"""Aggregate plots across all completed rosbag2 runs under logs/rosbag/."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib

matplotlib.use('Agg')

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    sys.stderr.write('matplotlib is required: pip install matplotlib\n')
    raise


def _load_bag_lib():
    root = Path(__file__).resolve().parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import bag_statistics_lib

    return bag_statistics_lib


def _discover_bags(mod) -> List[Path]:
    cfg = mod.CFG
    bag_root = cfg.project_root / cfg.bag_root_rel
    if not bag_root.is_dir():
        return []
    bags = [p for p in bag_root.iterdir() if mod._is_valid_rosbag2_dir(p)]
    return sorted(bags, key=lambda p: p.name)


def _save(fig, out_dir: Path, name: str, project_root: Path):
    path = out_dir / f'{name}.png'
    fig.savefig(path)
    plt.close(fig)
    print(f'  saved: {path.relative_to(project_root)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output', type=str, default=None,
        help='Output directory (default: Config.stats_all_runs_rel under project root).',
    )
    args = parser.parse_args()

    mod = _load_bag_lib()
    cfg = mod.CFG
    project_root = cfg.project_root
    bags = _discover_bags(mod)
    if not bags:
        print(f'No completed rosbags under {project_root / cfg.bag_root_rel}')
        sys.exit(1)

    out_dir = (
        Path(args.output) if args.output
        else project_root / cfg.stats_all_runs_rel
    )
    if not out_dir.is_absolute():
        out_dir = project_root / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f'Runs: {len(bags)}  →  {out_dir.relative_to(project_root)}')
    rows: List[Tuple[str, Dict[str, Any]]] = []
    for bag in bags:
        print(f'  reading {bag.name} ...', flush=True)
        data = mod._read_bag(bag)
        mod.validate_bag_for_plots(data, bag.name)
        rows.append((bag.name, mod.aggregate_metrics(data)))

    mod._setup_mpl()
    runs = [name for name, _ in rows]

    inv_ru = {v: k for k, v in cfg.mode_labels_ru.items()}

    # --- all_01: stacked mode share (% time) per run ---
    all_mode_labels: set = set()
    for _, m in rows:
        all_mode_labels.update(m['mode_timeline']['mode_share'].keys())
    mode_order = sorted(all_mode_labels)
    shares_per_run = [m['mode_timeline']['mode_share'] for _, m in rows]

    fig, ax = plt.subplots(figsize=(max(10.0, 0.45 * len(runs)), cfg.figure_size[1]))
    x = np.arange(len(runs))
    bottom = np.zeros(len(runs))
    for mode in mode_order:
        en = inv_ru.get(mode, mode)
        color = cfg.mode_palette.get(en, '#888888')
        h = np.array([
            shares_per_run[i].get(mode, 0.0) * 100.0
            for i in range(len(runs))
        ])
        ax.bar(x, h, bottom=bottom, label=mode, color=color)
        bottom += h
    ax.set_xticks(x)
    ax.set_xticklabels(runs, rotation=35, ha='right', fontsize=max(8, cfg.font_size - 2))
    ax.set_ylabel('Доля времени, %')
    ax.set_title('Режимы сопровождения по запускам (нормировано на 100%)')
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0))
    fig.tight_layout()
    _save(fig, out_dir, 'all_01_mode_shares', project_root)

    # --- all_02: visibility ratio ---
    vis_vals = [m['visibility']['visibility_ratio'] for _, m in rows]
    fig, ax = plt.subplots(figsize=(max(10.0, 0.45 * len(runs)), cfg.figure_size[1]))
    x = np.arange(len(runs))
    ax.bar(x, [100.0 * v for v in vis_vals], color='#1f77b4')
    ax.set_xticks(x)
    ax.set_xticklabels(runs, rotation=35, ha='right', fontsize=max(8, cfg.font_size - 2))
    ax.set_ylabel('Доля времени видимости, %')
    ax.set_title('Видимость цели по запускам')
    fig.tight_layout()
    _save(fig, out_dir, 'all_02_visibility', project_root)

    # --- all_03: safety event counts (collisions, risk zone, prevented) ---
    cvals = [m['safety']['collisions_count'] for _, m in rows]
    pvals = [m['safety']['potential_conflicts_count'] for _, m in rows]
    prvals = [m['safety']['prevented_count'] for _, m in rows]
    fig, ax = plt.subplots(figsize=(max(10.0, 0.5 * len(runs)), cfg.figure_size[1]))
    x = np.arange(len(runs))
    w = 0.25
    ax.bar(x - w, cvals, w, label='Столкновения', color='#d62728')
    ax.bar(x, pvals, w, label='Конфликты (риск)', color='#ff7f0e')
    ax.bar(x + w, prvals, w, label='Предотвращено', color='#2ca02c')
    ax.set_xticks(x)
    ax.set_xticklabels(runs, rotation=35, ha='right', fontsize=max(8, cfg.font_size - 2))
    ax.set_ylabel('Количество событий')
    ax.set_title('Безопасность: события по запускам')
    ax.legend()
    fig.tight_layout()
    _save(fig, out_dir, 'all_03_safety_events', project_root)

    # --- all_04: control smoothness RMS ---
    sm_rms = [m['control_smoothness']['smoothness_rms'] for _, m in rows]
    fig, ax = plt.subplots(figsize=(max(10.0, 0.45 * len(runs)), cfg.figure_size[1]))
    x = np.arange(len(runs))
    ax.bar(x, sm_rms, color='#2ca02c')
    ax.set_xticks(x)
    ax.set_xticklabels(runs, rotation=35, ha='right', fontsize=max(8, cfg.font_size - 2))
    ax.set_ylabel(r'СКЗ $\|\mathbf{u}_k-\mathbf{u}_{k-1}\|_2$')
    ax.set_title('Плавность управления по запускам')
    fig.tight_layout()
    _save(fig, out_dir, 'all_04_control_smoothness', project_root)

    print('Done.')


if __name__ == '__main__':
    main()
