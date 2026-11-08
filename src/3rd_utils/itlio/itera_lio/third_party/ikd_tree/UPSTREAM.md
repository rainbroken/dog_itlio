# Vendored ikd-Tree

This directory contains the subset of ikd-Tree compiled by IteraLIO.

- Upstream: https://github.com/hku-mars/ikd-Tree
- Base commit: `c0e36a16b6e4d557d3783b16911207f6398dd478`
- License: GPL-2.0; see `LICENSE`

The vendored sources include local IteraLIO changes for point metadata,
configurable rebuild policies, bounded count-only box queries, insertion
reports, rebuild statistics, and search-lock wait metrics. Update this note
when rebasing the vendored copy onto another upstream revision.

Nearest-neighbor queries retain caller result capacities, fill results in
reverse heap-pop order, and reuse a per-thread search heap. The traversal,
distance comparisons, and tie ordering are unchanged.
