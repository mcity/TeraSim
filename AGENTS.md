# Repository Guidelines

## Project Structure & Module Organization
- `packages/`: Python packages — `terasim` (core), `terasim-nde-nade`, `terasim-service` (FastAPI), `terasim-envgen`, `terasim-datazoo`, `terasim-vis`.
- `tests/`: Pytest suites (`test_core/`, `test_envgen/`, `test_service/`, `test_nde_nade/`, `test_integration/`).
- `examples/`: Ready-to-run scenarios and maps.
- `docs/`: Architecture, figures, and reference docs.
- `apps/`: Deployment assets (Docker, scripts).
- `scripts/`: Utilities, e.g., `run_tests.sh`.

## Build, Test, and Development Commands
- Environment: `./setup_environment.sh` (installs editable packages + dev tools).
- Run tests: `uv run pytest` or `./scripts/run_tests.sh --skip-slow`.
  - Markers: `-m "not slow"`, `-m integration`, `-m "not requires_sumo"`.
- Lint/format: `uv run ruff check .`, `uv run black .`.
- Type check: `uv run mypy packages/` (strict on untyped defs).
- Service (API): `uv run terasim-service` (FastAPI on `http://localhost:8000`).

## Coding Style & Naming Conventions
- Indentation: 4 spaces; prefer type hints for public APIs.
- Formatting: Black (line length 100 at repo root). Import order via isort profile used in packages where configured.
- Linting: Ruff rules `E,F,W,C90,I`; keep commits lint‑clean.
- Naming: `snake_case` for modules/functions, `PascalCase` for classes, `UPPER_SNAKE_CASE` for constants. Filenames should be lowercase with underscores.

## Testing Guidelines
- Framework: Pytest; tests live under `tests/` and follow `test_*.py` naming.
- Coverage: Enabled by default via `pyproject.toml` (HTML with `--cov-report=html`).
- Markers: use `@pytest.mark.integration`, `requires_sumo`, `requires_gui`, `slow` appropriately.
- Run subsets: `uv run pytest tests/test_core/ -m "not slow"`.

## Commit & Pull Request Guidelines
- Commits: imperative and concise (e.g., `fix: handle SUMO path resolution`). One logical change per commit.
- PRs: include a clear summary, linked issues, reproduction steps, test coverage notes, and screenshots/logs for service/UI changes.
- Requirements: CI green (lint, type checks, tests), update docs/examples when behavior or APIs change.

## Environment & Configuration Tips
- SUMO: set `SUMO_HOME` for tests/features that require SUMO. The setup script can install SUMO 1.23.1.
- Service: `redis` should be running locally for `terasim-service` features that need it.
- Data/outputs: transient artifacts go to `logs/` and `outputs/`; avoid committing large files.

