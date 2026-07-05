# Data & large artifacts (off-machine)

This repo tracks only small files (code, configs, the paper, small NN
checkpoints, and force-added figures/CSVs). The **large, regenerate-or-recover
artifacts are stored off-machine** as tar-split **GitHub Release assets** on a
private companion repo, so they are backed up and fetchable anywhere without
bloating git:

- **Data repo:** `ksha23/terrain-aware-offroad-control-data` (private)
- **Tool:** [`data_sync/data_sync.sh`](data_sync/data_sync.sh) (uses `gh`)

## Restore the backed-up data

```bash
# into the current repo tree (default), from the latest snapshot tag:
data_sync/data_sync.sh list                         # see snapshot tags
data_sync/data_sync.sh pull snapshot-2026-07-02     # download + reassemble + extract
```

`pull` recreates `data/`, the paper-supporting `benchmarking/results/…` folders,
`paper_figure_data/`, and `experiment_results/` under the repo root.

## Take a new snapshot

```bash
# tar+split every path in data_sync/data_snapshot.list and upload as release assets
DATA_ROOT=/path/to/populated/SCM_Final data_sync/data_sync.sh push          # tag = snapshot-YYYYMMDD
DATA_ROOT=/path/to/populated/SCM_Final data_sync/data_sync.sh push my-tag
```

`DATA_ROOT` is the tree the listed paths are relative to (defaults to this repo
root). Files over 2 GB are split into ≤1.9 GB parts automatically (the Release
asset size limit); `pull` reassembles them.

## What is / isn't in the snapshot

**In the snapshot** (`data_sync/data_snapshot.list`): the training datasets
(`data/`), the raw result folders that back every figure/table in the paper
(the `publish_manifest.json`- and provenance-cited sweeps plus the
teleoperation counterfactual runs), and the figure source data.

**Not in the snapshot (local-only, restore-on-demand or regenerate):**
- any local `archive/<YYYY-MM-DD_label>/` of superseded work (create it when
  retiring files) — snapshot it separately if needed:
  `printf 'archive\n' > /tmp/l; LIST=/tmp/l data_sync/data_sync.sh push archive-<date>`
- the full `benchmarking/results/` (~25 GB, mostly superseded debug runs) — only
  the paper-cited subset is snapshotted; the rest is regenerable via
  `benchmarking/run.py`.

The snapshot set is deliberately curated (~12 GB) rather than the full ~51 GB
on disk, so a restore gives you exactly what reproduces the paper.

## Limitation: this is snapshotting, not differential sync

`push` re-tars and re-uploads the **whole** backup set under a new tag; `pull`
downloads a tag's tarballs and **extracts over** the destination tree. There is
**no per-file delta and no merge**:

- Adding one file still re-uploads the full set on the next `push`.
- `pull` **overwrites** — it does not merge, so it can clobber un-pushed local
  changes. Treat snapshots as periodic backups, not a live two-machine sync.

For a differential, multi-machine, edit-and-sync workflow (only changed files
transferred, versioned with code), use **git-LFS** (transparent, ~\$5/mo above
the 1 GB free tier) or **DVC** (free, bring-your-own cloud bucket) instead. The
Release-asset approach here is chosen for zero-setup, free, infrequent full
backups.
