# SNITHPilot WSL Development

Use the WSL-native checkout as the active development repo:

```bash
cd ~/snithpilot-ui-openpilot
```

Do not build or run openpilot from the Windows-mounted checkout under `/mnt/c` or `/mnt/d`. That path is slow for this repo and can break Git symlinks. Windows can still edit the WSL repo through:

```text
\\wsl$\Ubuntu\home\sannihith\snithpilot-ui-openpilot
```

## Browser Preview

One-time WSL setup:

```bash
./scripts/setup_wsl_ui_dev.sh
```

Start the browser bridge:

```bash
./scripts/start_raylib_ui_browser.sh
```

Open:

```text
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

Run the actual StarPilot Raylib onroad UI:

```bash
./scripts/run_onroad_browser.sh --demo
```

For a route replay, replace `--demo` with the route arguments accepted by `./onroad`.

## Useful Local Checks

Targeted Carnival longitudinal checks:

```bash
./dev python analysis/run_targeted_longitudinal_pytest.py
./dev python analysis/scan_longitudinal_quality.py <qlog-or-rlog-path>
```

General state check:

```bash
git status --short --branch
```

Generated route-analysis JSON files should stay local unless they are intentionally being preserved as fixtures.
